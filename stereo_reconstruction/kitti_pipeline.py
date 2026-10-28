"""KITTI stereo → 직접 SGM/DLT → 실제 pose → voxel 신규점 → STP1.

실행/스트리밍 모두 같은 Pipeline.step()을 사용. 학습모델/외부 stereo library 없음.
"""
import argparse
import json
from pathlib import Path
import time
import hashlib
import numpy as np
import cv2
from kitti_io import KittiRaw,KittiOdometry,transform,apply_pose
from dense_stereo import rectification_geometry,rectify_image
from sgm_stereo import sgm_correspondences
from camera import build_projection_matrix
from triangulation import triangulate_points_dlt
from point_stream import VoxelMap,RECORD,packetize
from exporter import export_ply
from surface_attributes import estimate_surface,make_records,export_surface,SURFACE_RECORD,packetize_surface


def write_json(path,data):
    Path(path).write_text(json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')


def scaled_camera(K,old_size,new_size):
    """cv2.resize pixel-centre 규약에 맞는 intrinsic: u'=(u+.5)*sx-.5."""
    sx,sy=new_size[0]/old_size[0],new_size[1]/old_size[1]
    A=np.array([[sx,0,(sx-1)/2],[0,sy,(sy-1)/2],[0,0,1.]])
    return A@K


def lidar_projection(scan,T_camera_velo,K,size):
    """독립 검증용 LiDAR→정렬 영상 직접 투영 및 nearest-depth z-buffer.

    LiDAR는 SGM 추정/필터/누적에 사용하지 않는다. 반사도는 색으로 쓰지 않는다.
    """
    xyz=apply_pose(T_camera_velo,np.asarray(scan[:,:3],float))
    positive=xyz[:,2]>0; xyz=xyz[positive]
    image=xyz@K.T; uv=image[:,:2]/image[:,2:3]
    finite=np.isfinite(uv).all(1); xyz,uv=xyz[finite],uv[finite]
    uv=np.rint(uv).astype(np.int64); w,h=size
    valid=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
    xyz,uv=xyz[valid],uv[valid]
    order=np.argsort(xyz[:,2],kind='stable'); keys=uv[:,1]*w+uv[:,0]
    _,first=np.unique(keys[order],return_index=True); keep=order[first]
    return uv[keep],xyz[keep,2]


class Pipeline:
    def __init__(self,dataset,frames,output,scale=.5,max_disparity=96,min_depth=2.,max_depth=40.,voxel=.15,max_points=2_000_000,chunk_points=4096):
        if not frames or list(frames)!=sorted(set(frames)) or min(frames)<0: raise ValueError('frame는 중복 없는 증가 순서여야 합니다.')
        if not np.isfinite([scale,min_depth,max_depth]).all() or not 0<scale<=1 or not 0<min_depth<max_depth: raise ValueError('scale/depth 범위 오류')
        if not 1<=chunk_points<=65536: raise ValueError('chunk_points 범위 오류')
        self.dataset=dataset; self.frames=list(frames); self.output=Path(output)
        # 새 실행에서 과거 결과를 조용히 혼합하지 않는다.
        self.output.mkdir(parents=True,exist_ok=True)
        if any(self.output.iterdir()): raise ValueError('output은 비어 있는 새 디렉터리여야 합니다.')
        self.scale=scale; self.max_disparity=max_disparity; self.min_depth=min_depth; self.max_depth=max_depth
        self.map=VoxelMap(voxel,max_points); self.chunk_points=chunk_points
        self.summaries=[]; self.next_index=0; self.started=time.perf_counter(); self.lidar_reports=[]
        for frame in frames: dataset.validate_frame(frame)
        self.config=dict(dataset=dataset.name,kind=dataset.kind,frames=list(frames),scale=scale,max_disparity=max_disparity,
            min_depth_m=min_depth,max_depth_m=max_depth,voxel_size_m=voxel,max_points=max_points,chunk_points=chunk_points,
            algorithm='custom_census_guided_8path_sgm',confidence_threshold=.35,pose_source=dataset.pose_source,
            coordinate_frame='world_first_left_camera',world_reference_frame=0,units='metre',
            point_order='append-only, global point_id; selected original observations, no centroid',
            schema='STP1',point_stride=32,pose_applied=True,surface_schema='SRF1',surface_stride=32)
        write_json(self.output/'config.json',self.config)

    def step(self):
        if self.next_index>=len(self.frames): raise StopIteration
        start=time.perf_counter(); frame=self.frames[self.next_index]
        original_left,original_right=self.dataset.images(frame)
        old=(original_left.shape[1],original_left.shape[0])
        size=(max(8,int(round(old[0]*self.scale))),max(8,int(round(old[1]*self.scale))))
        if max(size)>65535: raise ValueError('STP1 uint16 pixel size 초과')
        left=cv2.resize(original_left,size,interpolation=cv2.INTER_AREA)
        right=cv2.resize(original_right,size,interpolation=cv2.INTER_AREA)
        kl=scaled_camera(self.dataset.K_left,old,size); kr=scaled_camera(self.dataset.K_right,old,size)
        R=self.dataset.T_right_left[:3,:3]; t=self.dataset.T_right_left[:3,3]
        kn,ql,qr,baseline=rectification_geometry(kl,kr,R,t,size)
        left,vl,_,_=rectify_image(left,kl,ql,kn,size)
        right,vr,_,_=rectify_image(right,kr,qr,kn,size)
        match_start=time.perf_counter()
        result=sgm_correspondences(left,right,vl,vr,min_disparity=0,max_disparity=self.max_disparity)
        matching_seconds=time.perf_counter()-match_start
        y,x=np.nonzero(result['valid']); d=result['disparity'][y,x]
        pl=np.column_stack((x,y)); pr=np.column_stack((x-d,y))
        P0=build_projection_matrix(kn,np.eye(3),np.zeros(3)); P1=build_projection_matrix(kn,np.eye(3),[-baseline,0,0])
        xyz=triangulate_points_dlt(pl,pr,P0,P1)
        keep=np.isfinite(xyz).all(1)&(xyz[:,2]>=self.min_depth)&(xyz[:,2]<=self.max_depth)
        xyz,pl,d=xyz[keep],pl[keep],d[keep]; x,y=x[keep],y[keep]
        if len(xyz): np.testing.assert_allclose(xyz[:,2],kn[0,0]*baseline/d.astype(float),rtol=1e-7,atol=1e-6)
        confidence=result['confidence'][y,x]; rgb=left[y,x,::-1]
        T_world_cam=self.dataset.pose(frame)
        T_world_rect=T_world_cam@transform(ql.T)
        world=apply_pose(T_world_rect,xyz)
        records,indices,stats=self.map.add(world,rgb,pl,d,confidence)
        folder=self.output/f'frames/{frame:010d}'; folder.mkdir(parents=True)
        (folder/'new_points.bin').write_bytes(records.tobytes())
        surface=estimate_surface(xyz,pl,size)
        selected={k:v[indices] for k,v in surface.items()}
        surface_records=make_records(selected,records['point_id'],T_world_rect[:3,:3])
        surface_meta=export_surface(folder,surface_records,records['xyz'],records['rgba'][:,:3],'world_first_left_camera')
        for i,packet in enumerate(packetize_surface(surface_records,frame,self.chunk_points)):
            (folder/f'surface_{i:05d}.srf').write_bytes(packet)
        for i,packet in enumerate(packetize(records,frame,self.chunk_points)):
            (folder/f'chunk_{i:05d}.stp').write_bytes(packet)
        cv2.imwrite(str(folder/'left_rectified.png'),left); cv2.imwrite(str(folder/'right_rectified.png'),right)
        final_mask=np.zeros((size[1],size[0]),bool); final_mask[y,x]=True
        cv2.imwrite(str(folder/'valid_mask.png'),final_mask.astype(np.uint8)*255)
        np.savez_compressed(folder/'observations.npz',camera_xyz_m=xyz,pixel_left=pl,disparity_px=d,
            confidence=confidence,rgb=rgb,selected_indices=indices,point_id=records['point_id'],
            disparity_map_px=result['disparity'],valid_mask=final_mask,T_world_from_observation_camera=T_world_rect,K_rect=kn)
        meta=dict(type='frame_begin',frame_id=frame,timestamp_seconds=self.dataset.timestamp(frame),
            coordinate_frame='world_first_left_camera',position_unit='metre',pose_applied=True,
            observation_camera='current custom-rectified left camera',image_size=list(size),
            K_rect=kn.tolist(),baseline_m=float(baseline),
            T_world_from_observation_camera=T_world_rect.tolist(),
            T_world_from_observation_camera_column_major=T_world_rect.T.reshape(-1).tolist(),
            surface_schema='SRF1',surface_valid=surface_meta['valid_normals'],
            point_count=len(records),total_map_points=self.map.count,
            first_point_id=int(records['point_id'][0]) if len(records) else None,
            chunk_count=max(1,(len(records)+self.chunk_points-1)//self.chunk_points),
            processing_seconds=time.perf_counter()-start,matching_seconds=matching_seconds,
            matching_points=int(result['valid'].sum()),**stats)
        write_json(folder/'frame.json',meta)
        scan=self.dataset.lidar(frame)
        if scan is not None and self.dataset.T_camera_velo is not None:
            uv,z=lidar_projection(scan,transform(ql)@self.dataset.T_camera_velo,kn,size)
            cv2.imwrite(str(folder/'lidar_projection.png'),self._lidar_overlay(left,uv))
            ok=final_mask[uv[:,1],uv[:,0]]&(z>=self.min_depth)&(z<=self.max_depth)
            u,v=uv[ok].T; lidar_d=kn[0,0]*baseline/z[ok]
            error=np.abs(result['disparity'][v,u]-lidar_d)
            report=dict(frame_id=frame,projected_lidar_pixels=len(uv),compared_pixels=len(error),
                median_disparity_error_px=float(np.median(error)) if len(error) else None,
                median_relative_depth_error_percent=float(np.median(np.abs(lidar_d/result['disparity'][v,u]-1))*100) if len(error) else None,
                note='Sparse LiDAR sanity check only: sensor timing, occlusions and moving objects differ. Not dense stereo ground truth; never used by SGM or map filtering.')
            self.lidar_reports.append(report); write_json(folder/'lidar_check.json',report)
        self.summaries.append(meta); self.next_index+=1
        print(f'frame {frame}: {len(xyz)} depth-valid → {len(records)} new voxels; map={self.map.count}',flush=True)
        return meta,records

    @staticmethod
    def _lidar_overlay(left,uv):
        overlay=left.copy(); overlay[uv[:,1],uv[:,0]]=(0,255,0); return overlay

    def finish(self):
        if self.next_index!=len(self.frames): raise RuntimeError('모든 frame을 처리한 뒤 완료할 수 있습니다.')
        # 각 frame 신규점만 순서대로 연결: server 수신 결과와 byte-for-byte 같음.
        with (self.output/'map_points.bin').open('wb') as handle:
            for frame in self.frames: handle.write((self.output/f'frames/{frame:010d}/new_points.bin').read_bytes())
        points=np.frombuffer((self.output/'map_points.bin').read_bytes(),dtype=RECORD)
        assert len(points)==self.map.count
        np.testing.assert_array_equal(points['point_id'],np.arange(len(points)))
        export_ply(np.column_stack((points['xyz'],points['rgba'][:,:3])),self.output/'map_points_m.ply')
        surface_records=np.concatenate([np.frombuffer((self.output/f'frames/{frame:010d}/surface.bin').read_bytes(),dtype=SURFACE_RECORD) for frame in self.frames])
        export_surface(self.output,surface_records,points['xyz'],points['rgba'][:,:3],'world_first_left_camera')
        report=dict(surface_sha256=hashlib.sha256(surface_records.tobytes()).hexdigest(),config=self.config,frames=self.summaries,map_points=len(points),
            total_depth_valid=sum(m['input_points'] for m in self.summaries),
            binary_sha256=hashlib.sha256((self.output/'map_points.bin').read_bytes()).hexdigest(),
            elapsed_seconds=time.perf_counter()-self.started,lidar_checks=self.lidar_reports,
            caveat='Provided GPS/IMU or dataset poses, no SLAM estimation; no dynamic-object removal, no voxel replacement, no real-time 10Hz claim.')
        write_json(self.output/'summary.json',report)
        return report


def add_arguments(p):
    p.add_argument('--root',required=True); p.add_argument('--format',choices=['raw','odometry'],default='raw')
    p.add_argument('--date',default='2011_09_26'); p.add_argument('--drive',default='0001')
    p.add_argument('--sequence',default='00'); p.add_argument('--poses')
    p.add_argument('--frames',type=int,nargs='+',default=[0,5,10,15,20]); p.add_argument('--output',required=True)
    p.add_argument('--scale',type=float,default=.5); p.add_argument('--max-disparity',type=int,default=96)
    p.add_argument('--min-depth',type=float,default=2.); p.add_argument('--max-depth',type=float,default=40.)
    p.add_argument('--voxel',type=float,default=.15); p.add_argument('--max-points',type=int,default=2_000_000)
    p.add_argument('--chunk-points',type=int,default=4096)


def from_arguments(a):
    dataset=KittiRaw(a.root,a.date,a.drive) if a.format=='raw' else KittiOdometry(a.root,a.sequence,a.poses)
    return Pipeline(dataset,a.frames,a.output,a.scale,a.max_disparity,a.min_depth,a.max_depth,a.voxel,a.max_points,a.chunk_points)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__); add_arguments(p); a=p.parse_args()
    pipeline=from_arguments(a)
    for _ in a.frames: pipeline.step()
    print(json.dumps(pipeline.finish(),indent=2))
