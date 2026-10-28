"""보정에 쓰지 않은 체커보드 쌍으로 기존 직접 DLT/재투영 구현을 검증."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from calibration_tools import load_raw_model,read_pairs,detect_corners
from camera import build_projection_matrix,skew_symmetric
from image_io import load_stereo_images
from triangulation import triangulate_points_dlt
from reprojection import project_points,compute_reprojection_error
from epipolar import compute_epipolar_distances


def verify(model_path,pairs_path,output):
    model=load_raw_model(model_path)
    cols,rows=model['board_inner_corners']; square=model['square_size']
    kl,kr=np.asarray(model['K_left']),np.asarray(model['K_right'])
    dl,dr=np.asarray(model['D_left']),np.asarray(model['D_right'])
    R,t=np.asarray(model['R']),np.asarray(model['t'])
    P1=build_projection_matrix(kl,np.eye(3),np.zeros(3)); P2=build_projection_matrix(kr,R,t)
    F=np.linalg.inv(kr).T@skew_symmetric(t)@R@np.linalg.inv(kl)
    results=[]
    for lp,rp in read_pairs(pairs_path):
        left,right=load_stereo_images(lp,rp)
        for image,side in ((left,'left'),(right,'right')):
            if [image.shape[1],image.shape[0]]!=model['image_size_'+side]:
                raise ValueError('validation 이미지 해상도가 다릅니다.')
        lc,rc=detect_corners(left,(cols,rows)),detect_corners(right,(cols,rows))
        if lc is None or rc is None:
            raise ValueError('검증 이미지의 코너를 찾지 못했습니다.')
        ul=cv2.undistortPoints(lc.reshape(-1,1,2),kl,dl,P=kl).reshape(-1,2)
        ur=cv2.undistortPoints(rc.reshape(-1,1,2),kr,dr,P=kr).reshape(-1,2)
        xyz=triangulate_points_dlt(ul,ur,P1,P2)
        if not np.isfinite(xyz).all():
            raise ValueError('체커보드 코너 DLT에 무효점이 있습니다.')
        if not ((xyz[:,2]>0).all() and ((xyz@R.T+t)[:,2]>0).all()):
            raise ValueError('체커보드 코너가 카메라 뒤에 복원되었습니다.')
        grid=xyz.reshape(rows,cols,3)
        edges=np.concatenate([np.linalg.norm(np.diff(grid,axis=0),axis=2).ravel(),
                              np.linalg.norm(np.diff(grid,axis=1),axis=2).ravel()])
        centered=xyz-xyz.mean(axis=0); normal=np.linalg.svd(centered)[2][-1]
        planarity=float(np.sqrt(np.mean((centered@normal)**2))/square)
        el=compute_reprojection_error(ul,project_points(P1,xyz))
        er=compute_reprojection_error(ur,project_points(P2,xyz))
        _,epipolar=compute_epipolar_distances(F,ul,ur)
        detected=detect_corners(cv2.undistort(left,kl,dl,None,kl),(cols,rows))
        delta=None
        if detected is not None:
            # 비교 시 검출기의 전체 순서 반전만 정렬. calibration 추정에는 사용 안 함.
            delta=float(min(np.median(np.linalg.norm(detected-ul,axis=1)),
                            np.median(np.linalg.norm(detected[::-1]-ul,axis=1))))
        record=dict(left=str(lp),right=str(rp),corners=len(xyz),
                    median_edge_in_squares=float(np.median(edges)/square),
                    edge_relative_rmse=float(np.sqrt(np.mean((edges/square-1)**2))),
                    planarity_rms_in_squares=planarity,
                    mean_left_reprojection_px=float(el.mean()),mean_right_reprojection_px=float(er.mean()),
                    mean_right_epipolar_px=float(epipolar.mean()),undistorted_corner_median_delta_px=delta)
        record['passed']=bool(.9<record['median_edge_in_squares']<1.1 and record['edge_relative_rmse']<.15 and
                              planarity<.1 and max(el.mean(),er.mean())<1 and delta is not None and delta<1)
        results.append(record)
    report=dict(passed=all(r['passed'] for r in results),units=model['units'],results=results,
                note='Board-scale geometric consistency only; no independently known metric baseline.')
    output=Path(output); output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(report,indent=2))
    if not report['passed']:
        raise ValueError('공개 샘플 검증 기준을 통과하지 못했습니다. 보고서를 확인하세요.')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',required=True); p.add_argument('--pairs',required=True)
    p.add_argument('--output',default='sample_validation.json')
    args=p.parse_args()
    try:
        verify(args.model,args.pairs,args.output)
    except (ValueError,OSError,cv2.error) as error:
        p.exit(2,f'오류: {error}\n')
