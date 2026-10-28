"""보정된 이미지 두 장에서 custom dense stereo → custom DLT → XYZRGB."""
import argparse,json,time
from pathlib import Path
import cv2
import numpy as np
from camera import load_calibration,build_projection_matrix
from geometry import from_homogeneous,to_homogeneous
from image_io import load_stereo_images,save_image
from dense_stereo import rectification_geometry,rectify_image,dense_correspondences
from sgm_stereo import sgm_correspondences
from triangulation import triangulate_points_dlt
from filtering import filter_reconstructed_points
from point_cloud import assign_colors
from exporter import export_json,export_ply
from visualization import side_by_side
from surface_attributes import estimate_surface,make_records,export_surface


def run(left_path,right_path,calibration_path,output,min_disparity=1,max_disparity=160,
        window=9,uniqueness=.15,lr_threshold=1.,min_texture=3.,max_cost=9.,
        min_depth=1e-6,max_depth=1000.,max_distance=1000.,method='window',
        p1=2.,p2=24.,edge_scale=10.,min_confidence=.35,
        sgm_uniqueness=.04,sgm_max_cost=16.,meters_per_unit=None):
    start=time.perf_counter()
    left,right=load_stereo_images(left_path,right_path)
    kl,kr,R,t,units=load_calibration(calibration_path,left.shape,right.shape)
    if left.shape!=right.shape: raise ValueError('dense 모드는 동일 크기의 좌우 이미지가 필요합니다.')
    size=(left.shape[1],left.shape[0]); output=Path(output); output.mkdir(parents=True,exist_ok=True)
    kn,ql,qr,baseline=rectification_geometry(kl,kr,R,t,size)
    rect_l,vl,_,_=rectify_image(left,kl,ql,kn,size)
    rect_r,vr,_,_=rectify_image(right,kr,qr,kn,size)
    save_image(output/'01_rectified_left.png',rect_l); save_image(output/'02_rectified_right.png',rect_r)
    save_image(output/'rectification_valid_left.png',vl.astype(np.uint8)*255)
    save_image(output/'rectification_valid_right.png',vr.astype(np.uint8)*255)
    guide=side_by_side(rect_l,rect_r)
    for y in range(20,size[1],40): cv2.line(guide,(0,y),(guide.shape[1]-1,y),(0,200,255),1)
    save_image(output/'03_rectified_pair.jpg',guide)
    print(f'Rectified; computing custom {method} matching...',flush=True)
    if method=='window':
        result=dense_correspondences(rect_l,rect_r,vl,vr,min_disparity,max_disparity,window,
                                     uniqueness,lr_threshold,min_texture,max_cost)
    elif method=='sgm':
        result=sgm_correspondences(rect_l,rect_r,vl,vr,min_disparity,max_disparity,
                  p1,p2,edge_scale,sgm_uniqueness,lr_threshold,sgm_max_cost,min_confidence)
    else:
        raise ValueError('method는 window 또는 sgm이어야 합니다.')
    y,x=np.nonzero(result['valid']); d=result['disparity'][y,x]
    pl_rect=np.column_stack((x,y)); pr_rect=np.column_stack((x-d,y))
    # rectified pixel→original pinhole pixel. DLTと保存XYZは元の左camera座標。
    pl=from_homogeneous(to_homogeneous(pl_rect)@(kl@ql.T@np.linalg.inv(kn)).T)
    pr=from_homogeneous(to_homogeneous(pr_rect)@(kr@qr.T@np.linalg.inv(kn)).T)
    P1=build_projection_matrix(kl,np.eye(3),np.zeros(3)); P2=build_projection_matrix(kr,R,t)
    print(f'Confident dense correspondences: {len(pl)}; custom DLT...',flush=True)
    xyz,details=triangulate_points_dlt(pl,pr,P1,P2,return_details=True)
    filtered=filter_reconstructed_points(xyz,pl,pr,P1,P2,R,t,details['numerical_valid'],
                      left.shape,right.shape,min_depth,max_depth,max_distance,2.)
    accepted=filtered['final_mask']; cloud=assign_colors(xyz[accepted],left,pl[accepted])
    export_ply(cloud,output/'06_point_cloud.ply'); export_json(cloud,output/'07_point_cloud.json',units)
    np.save(output/'point_cloud.npy',cloud)
    unit_scale=meters_per_unit if meters_per_unit is not None else {'m':1.,'meter':1.,'metre':1.,'cm':.01,'mm':.001}.get(units)
    if unit_scale is not None:
        if not np.isfinite(unit_scale) or unit_scale<=0: raise ValueError('meters_per_unit must be positive')
        surface=estimate_surface((xyz[accepted]@ql.T)*unit_scale,pl_rect[accepted],size)
        surface_records=make_records(surface,np.arange(len(cloud)),ql.T)
        export_surface(output,surface_records,cloud[:,:3]*unit_scale,cloud[:,3:6],'original_left_camera')
    else:
        for name in ['surface.bin','surface.npz','surface.json','surface.ply']:
            path=output/name
            if path.exists(): path.unlink()
        print('Surface export skipped: unknown physical units; supply --meters-per-unit using measured calibration scale.',flush=True)
    accepted_mask=np.zeros(result['valid'].shape,bool); accepted_mask[y[accepted],x[accepted]]=True
    if method=='sgm':
        confidence=np.where(accepted_mask,result['confidence'],0)
        save_image(output/'08_confidence.png',np.rint(confidence*255).astype(np.uint8))
        np.savez_compressed(output/'point_attributes.npz',
            pixel_left=pl[accepted],pixel_right=pr[accepted],
            rectified_pixel_left=pl_rect[accepted],disparity=d[accepted],
            confidence=result['confidence'][y[accepted],x[accepted]],
            confidence_map=confidence,valid_mask=accepted_mask)
    np.savez_compressed(output/'dense_data.npz',disparity=result['disparity'],matching_mask=result['valid'],
          final_mask=accepted_mask,best_cost=result['best_cost'],left_right_error=result['left_right_error'],
          points_left=pl[accepted],points_right=pr[accepted],rectified_left=pl_rect[accepted],
          rectified_right=pr_rect[accepted],K_rect=kn,Q_left=ql,Q_right=qr,
          reprojection_left=filtered['error_left'][accepted],reprojection_right=filtered['error_right'][accepted])
    colored=cv2.applyColorMap(np.clip((result['disparity']-min_disparity)/(max_disparity-min_disparity)*255,0,255).astype(np.uint8),cv2.COLORMAP_TURBO)
    colored[~accepted_mask]=0; save_image(output/'04_disparity.png',colored)
    save_image(output/'05_valid_mask.png',accepted_mask.astype(np.uint8)*255)
    summary=dict(surface_exported=unit_scale is not None,surface_meters_per_unit=unit_scale,mode='custom_census_'+method,points=len(cloud),units=units,baseline=float(baseline),
        image_size=list(size),coverage_fraction=float(accepted_mask.mean()),
        parameters=dict(min_disparity=min_disparity,max_disparity=max_disparity,window=window,
                        uniqueness=uniqueness,lr_threshold=lr_threshold,min_texture=min_texture,max_cost=max_cost,
                        min_depth=min_depth,max_depth=max_depth,max_distance=max_distance),
        matching_before_support=result['before_support'],matching_after_support=len(pl),
        geometry_removal_counts=filtered['removal_counts'],
        median_left_right_error_px=float(np.median(result['left_right_error'][accepted_mask])) if len(cloud) else None,
        mean_left_reprojection_error_px=float(filtered['error_left'][accepted].mean()) if len(cloud) else None,
        mean_right_reprojection_error_px=float(filtered['error_right'][accepted].mean()) if len(cloud) else None,
        elapsed_seconds=time.perf_counter()-start,
        caveat='Reprojection is near zero by construction for rectified disparity. It is not independent evidence of correspondence correctness. No hole filling or point duplication.')
    if method=='sgm':
        summary['parameters']=dict(min_disparity=min_disparity,max_disparity=max_disparity,
            p1=p1,p2=p2,edge_scale=edge_scale,uniqueness=sgm_uniqueness,
            max_cost=sgm_max_cost,min_confidence=min_confidence,lr_threshold=lr_threshold,
            min_depth=min_depth,max_depth=max_depth,max_distance=max_distance,
            directions=8,support=2)
        summary['confidence_note']='0..1 heuristic score, not calibrated probability; aligned with PLY/JSON rows in point_attributes.npz'
    (output/'diagnostics.json').write_text(json.dumps(summary,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(summary,indent=2),flush=True)
    if not len(cloud): raise ValueError('유효한 dense 점이 없습니다. 정렬/시차 범위/매칭 threshold를 확인하세요.')
    return cloud


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--left',required=True); p.add_argument('--right',required=True)
    p.add_argument('--calibration',required=True); p.add_argument('--output',default='dense_results')
    p.add_argument('--meters-per-unit',type=float,help='Known metres per calibration unit; never guess checkerboard scale')
    p.add_argument('--method',choices=['window','sgm'],default='window')
    p.add_argument('--p1',type=float,default=2.); p.add_argument('--p2',type=float,default=24.)
    p.add_argument('--edge-scale',type=float,default=10.)
    p.add_argument('--min-confidence',type=float,default=.35)
    p.add_argument('--sgm-uniqueness',type=float,default=.04)
    p.add_argument('--sgm-max-cost',type=float,default=16.)
    p.add_argument('--min-disparity',type=int,default=1); p.add_argument('--max-disparity',type=int,default=160)
    p.add_argument('--window',type=int,default=9); p.add_argument('--uniqueness',type=float,default=.15)
    p.add_argument('--lr-threshold',type=float,default=1.); p.add_argument('--min-texture',type=float,default=3.)
    p.add_argument('--max-cost',type=float,default=9.); p.add_argument('--min-depth',type=float,default=1e-6)
    p.add_argument('--max-depth',type=float,default=1000.); p.add_argument('--max-3d-distance',type=float,default=1000.)
    a=p.parse_args()
    try: run(a.left,a.right,a.calibration,a.output,a.min_disparity,a.max_disparity,a.window,
             a.uniqueness,a.lr_threshold,a.min_texture,a.max_cost,a.min_depth,a.max_depth,a.max_3d_distance,
             a.method,a.p1,a.p2,a.edge_scale,a.min_confidence,a.sgm_uniqueness,a.sgm_max_cost,a.meters_per_unit)
    except (ValueError,OSError,cv2.error) as error: p.exit(2,f'오류: {error}\n')


if __name__=='__main__': main()
