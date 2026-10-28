"""검증된 SGM 결과를 A→B 전달용 metre 단위 binary/PLY/NPZ로 패키징.

사용: python pack_handoff.py --results .../final_results --data-root .../data --output team_handoff_v1
원본 복원을 재실행하거나 점을 추가/삭제하지 않는다.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import struct
import numpy as np
from benchmark_sgm import read_calibration
from exporter import export_ply

SCENES = ('Motorcycle', 'Pipes', 'Adirondack')
RECORD = np.dtype(dict(names=['xyz','rgba','uv','disparity','confidence','point_id'],
    formats=[('<f4',3),('u1',4),('<u2',2),'<f4','<f4','<u4'],
    offsets=[0,12,16,20,24,28],itemsize=32))


def save_json(path,data):
    Path(path).write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def pack(results,data_root,output):
    results,data_root,output=map(Path,(results,data_root,output))
    output.mkdir(parents=True,exist_ok=True)
    entries=[]; checks=[]
    for scene_id,scene in enumerate(SCENES):
        source=results/scene/'sgm'; target=output/'scenes'/scene; target.mkdir(parents=True,exist_ok=True)
        cloud=np.load(source/'point_cloud.npy',allow_pickle=False)
        with np.load(source/'point_attributes.npz',allow_pickle=False) as f:
            a={key:f[key] for key in f.files}
        calib=read_calibration(data_root/scene/'calib.txt')
        n=len(cloud); h,w=a['valid_mask'].shape
        assert n < 2**32 and max(h,w)<2**16
        assert cloud.shape==(n,6) and np.isfinite(cloud).all()
        assert np.all(cloud[:,2]>0) and np.all((cloud[:,3:]>=0)&(cloud[:,3:]<=255))
        assert np.all(cloud[:,3:]==np.floor(cloud[:,3:]))
        y,x=np.nonzero(a['valid_mask'])
        np.testing.assert_array_equal(a['pixel_left'],np.column_stack((x,y)))
        assert n==len(x)==len(a['confidence'])==len(a['disparity'])
        np.testing.assert_array_equal(a['confidence'],a['confidence_map'][y,x])
        np.testing.assert_array_equal(a['disparity'],a['disparity_map'][y,x])
        np.testing.assert_allclose(a['pixel_right'],np.column_stack((x-a['disparity'],y)))
        assert np.isfinite(a['confidence']).all() and ((a['confidence']>=0)&(a['confidence']<=1)).all()
        records=np.zeros(n,dtype=RECORD)
        records['xyz']=(cloud[:,:3]*.001).astype('<f4')
        records['rgba'][:,:3]=cloud[:,3:].astype('u1'); records['rgba'][:,3]=255
        records['uv']=a['pixel_left'].astype('<u2')
        records['disparity']=a['disparity']; records['confidence']=a['confidence']
        records['point_id']=np.arange(n,dtype='<u4')
        (target/'points.bin').write_bytes(records.tobytes())
        # PLY도 binary와 동일한 float32 metre 값을 double 필드로 기록한다.
        export_ply(np.column_stack((records['xyz'],cloud[:,3:])),target/'points_m.ply')
        np.savez_compressed(target/'analysis.npz',xyz_m=records['xyz'],rgb=records['rgba'][:,:3],
            point_id=records['point_id'],pixel_left=a['pixel_left'],pixel_right=a['pixel_right'],
            disparity_px=a['disparity'],confidence=a['confidence'],valid_mask=a['valid_mask'],
            disparity_map_px=a['disparity_map'],confidence_map=a['confidence_map'])
        for src,name in [('im0.png','left.png'),('im1.png','right.png')]:
            shutil.copy2(data_root/scene/src,target/name)
        shutil.copy2(results/scene/'metrics.json',target/'evaluation.json')
        for name in ['shape_comparison.png','disparity_comparison.png']:
            shutil.copy2(results/scene/name,target/name)
        k0,k1=calib['cam0'],calib['cam1']
        manifest=dict(schema='stereo-points/1',scene_id=scene_id,scene=scene,count=n,
            algorithm='custom Census + edge-guided 3x3 + 8-path SGM; confidence >=0.35',
            units=dict(position='metre',disparity='pixel',pixel_coordinates='pixel',confidence='unitless heuristic, not probability'),
            coordinate_frame=dict(name='left_rectified_camera',origin='left camera optical centre',
                x='right',y='down',z='forward',handedness='right',
                pose_status='unavailable; independent scene, not a temporal frame',
                T_world_from_camera=None,
                suggested_viewer_from_camera_column_major=[1,0,0,0,0,-1,0,0,0,0,-1,0,0,0,0,1]),
            image=dict(width=w,height=h,left='left.png',right='right.png',state='rectified',
                pixel_origin='top-left pixel centre (0,0)',linear_index='v*width+u'),
            calibration=dict(K_left=k0.tolist(),K_right=k1.tolist(),R=np.eye(3).tolist(),
                t_m=[-calib['baseline']*.001,0,0],baseline_m=calib['baseline']*.001,
                doffs_px=float(k1[0,2]-k0[0,2]),pose_convention='X_right = R @ X_left + t_m',
                note='Provided Middlebury Q calibration; no scaling of K or image. Small residual vertical disparity may remain.'),
            point_order='original accepted pixel row-major order; point_id equals zero-based record index within this scene',
            binary=dict(file='points.bin',byte_order='little-endian',header_bytes=0,stride_bytes=32,byte_length=n*32,
                fields=[dict(name='xyz_m',offset=0,type='float32',components=3),
                        dict(name='rgb',offset=12,type='uint8',components=3,color_encoding='sRGB encoded RGB'),
                        dict(name='alpha',offset=15,type='uint8',components=1,constant=255),
                        dict(name='uv_left',offset=16,type='uint16',components=2),
                        dict(name='disparity_px',offset=20,type='float32',components=1),
                        dict(name='confidence',offset=24,type='float32',components=1),
                        dict(name='point_id',offset=28,type='uint32',components=1)]),
            files=dict(ply='points_m.ply',analysis='analysis.npz',preview='points_preview.json',evaluation='evaluation.json'),
            bounds_m=dict(min=records['xyz'].min(0).tolist(),max=records['xyz'].max(0).tolist()),
            source=dict(dataset='Middlebury trainingQ',url='https://vision.middlebury.edu/stereo/submit3/',
                original_position_unit='mm',position_conversion='xyz_m = float32(xyz_mm * 0.001)',
                is_lidar=False,is_kitti=False,world_pose_applied=False,downsampled=False,
                normals_available=False,triangles_available=False))
        save_json(target/'manifest.json',manifest)
        save_json(target/'points_preview.json',dict(note='FIRST 32 POINTS ONLY: human-readable preview, not complete cloud',
            columns=['point_id','X_m','Y_m','Z_m','R','G','B','u','v','disparity_px','confidence'],
            full_count=n,points=[[int(r['point_id']),*map(float,r['xyz']),*map(int,r['rgba'][:3]),
                *map(int,r['uv']),float(r['disparity']),float(r['confidence'])] for r in records[:32]]))
        # 한 scene의 4096점 packet 예시. 전체 stream/server가 아니다.
        sample_count=min(n,4096)
        flags=1 if sample_count==n else 0
        header=struct.pack('<4sHHIIIIII',b'STP1',1,32,scene_id,0,sample_count,32,flags,0)
        assert len(header)==32
        (target/'packet_sample.stp').write_bytes(header+records[:sample_count].tobytes())
        # 원본→binary→PLY/NPZ의 손실/대응 검증. float32 좌표만 허용오차를 둔다.
        back=np.fromfile(target/'points.bin',dtype=RECORD)
        assert len(back)==n and (target/'points.bin').stat().st_size==n*32
        error=float(np.max(np.abs(back['xyz'].astype(float)-cloud[:,:3]*.001)))
        assert error < 2e-6
        np.testing.assert_array_equal(back['disparity'],a['disparity'])
        np.testing.assert_array_equal(back['confidence'],a['confidence'])
        with (target/'points_m.ply').open() as handle:
            for line in handle:
                if line.strip()=='end_header': break
            ply=np.loadtxt(handle)
        np.testing.assert_array_equal(ply[:,:3],back['xyz'].astype(float))
        np.testing.assert_array_equal(ply[:,3:],back['rgba'][:,:3])
        with np.load(target/'analysis.npz') as data:
            np.testing.assert_array_equal(data['xyz_m'],back['xyz'])
            np.testing.assert_array_equal(data['point_id'],back['point_id'])
        # JS reader용 첫/중간/마지막 point oracle.
        samples=[]
        for i in (0,n//2,n-1):
            r=records[i]
            samples.append(dict(index=i,xyz=list(map(float,r['xyz'])),rgb=list(map(int,r['rgba'][:3])),
                uv=list(map(int,r['uv'])),disparity=float(r['disparity']),confidence=float(r['confidence'])))
        save_json(target/'decoder_test_vectors.json',dict(count=n,samples=samples))
        checks.append(dict(scene=scene,count=n,binary_bytes=n*32,max_position_rounding_error_m=error,
            record_order_rgb_uv_disparity_confidence='PASS',binary_ply_npz_roundtrip='PASS'))
        entries.append(dict(scene_id=scene_id,scene=scene,manifest=f'scenes/{scene}/manifest.json',count=n))
        print('Packed',scene,n,'points',flush=True)
    save_json(output/'manifest.json',dict(schema='stereo-handoff/1',recommended_first_scene='Motorcycle',
        scenes_are_independent=True,merge_scenes=False,scenes=entries,
        documents=['README_KO.md','DATA_FORMAT_KO.md','INTEGRATION_KO.md'],
        decoder='reader/read_points.mjs',packet_protocol='optional proposed protocol; no live WebSocket server included'))
    save_json(output/'validation_python.json',dict(status='PASS',checks=checks))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--results',required=True); p.add_argument('--data-root',required=True); p.add_argument('--output',required=True)
    a=p.parse_args(); pack(a.results,a.data_root,a.output)
