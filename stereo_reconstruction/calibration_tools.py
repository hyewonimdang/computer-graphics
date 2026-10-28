"""보정 준비 공통 함수. 복원 코어의 DLT/재투영 구현과 분리한다."""
import json
from pathlib import Path
import cv2
import numpy as np
from camera import validate_intrinsics, validate_pose
from image_io import load_stereo_images, save_image
from visualization import side_by_side


def board_points(cols, rows, square_size):
    if cols < 3 or rows < 3 or not np.isfinite(square_size) or square_size <= 0:
        raise ValueError('내부 코너 가로/세로는 3 이상, square-size는 유한한 양수여야 합니다.')
    points=np.zeros((cols*rows,3),np.float32)
    points[:,:2]=np.mgrid[0:cols,0:rows].T.reshape(-1,2)*square_size
    return points


def read_pairs(path):
    """JSON [{left,right},...] 경로는 JSON 파일 위치 기준. 순서 매칭 금지."""
    path=Path(path)
    data=json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(data,list) or not data:
        raise ValueError('pairs JSON은 비어 있지 않은 [{left,right},...] 목록이어야 합니다.')
    pairs=[]
    for row in data:
        if not isinstance(row,dict) or not all(isinstance(row.get(k),str) for k in ('left','right')):
            raise ValueError('각 pair에는 문자열 left/right 경로가 필요합니다.')
        pair=tuple((path.parent/row[k]).resolve() for k in ('left','right'))
        pairs.append(pair)
    validate_pairs(pairs)
    return pairs


def pairs_from_directories(left_dir,right_dir):
    """양쪽 동일 파일명으로만 매칭. 누락된 파일이 있으면 명시적 오류."""
    suffixes={'.png','.jpg','.jpeg','.bmp','.tif','.tiff'}
    def files(directory):
        directory=Path(directory)
        if not directory.is_dir():
            raise ValueError(f'이미지 폴더가 없습니다: {directory}')
        return {p.name:p.resolve() for p in directory.iterdir() if p.is_file() and p.suffix.lower() in suffixes}
    left,right=files(left_dir),files(right_dir)
    if not left or left.keys()!=right.keys():
        raise ValueError('좌우 폴더에 같은 이름의 이미지가 필요합니다. 다른 이름은 --pairs JSON을 사용하세요.')
    pairs=[(left[name],right[name]) for name in sorted(left)]
    validate_pairs(pairs)
    return pairs


def validate_pairs(pairs):
    flattened=[str(p) for pair in pairs for p in pair]
    if len(flattened)!=len(set(flattened)):
        raise ValueError('같은 이미지가 중복 사용되었습니다 (좌우 동일 파일 포함).')
    for pair in pairs:
        for path in pair:
            if not Path(path).is_file():
                raise ValueError(f'파일이 없습니다: {path}')


def detect_corners(image, pattern):
    """classic corner 검출 + subpixel 정밀화. 코너 순서는 시각화로 확인한다."""
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    ok,corners=cv2.findChessboardCorners(gray,pattern,cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return None
    return cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),
                           (cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,50,1e-4)).reshape(-1,2)


def collect_observations(pairs, pattern, output, expected_size=None):
    left_points,right_points,used,rejected=[],[],[],[]
    size=expected_size
    for index,(lp,rp) in enumerate(pairs):
        left,right=load_stereo_images(lp,rp)
        current=(left.shape[1],left.shape[0])
        if size is None:
            size=current
        if current!=size or (right.shape[1],right.shape[0])!=size:
            raise ValueError(f'모든 좌우 이미지의 해상도가 같아야 합니다: {lp}, {rp}')
        lc,rc=detect_corners(left,pattern),detect_corners(right,pattern)
        canvases=[]
        for image,corners in ((left,lc),(right,rc)):
            canvas=image.copy()
            if corners is not None:
                cv2.drawChessboardCorners(canvas,pattern,corners.reshape(-1,1,2),True)
                cv2.putText(canvas,'0',tuple(np.rint(corners[0]).astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX,.7,(0,0,255),2)
            canvases.append(canvas)
        save_image(Path(output)/f'corners_{index:03d}.jpg',side_by_side(*canvases))
        record=dict(left=str(lp),right=str(rp))
        if lc is None or rc is None:
            rejected.append(dict(record,reason='chessboard_not_detected',left_detected=lc is not None,right_detected=rc is not None))
            continue
        left_points.append(lc.astype(np.float32))
        right_points.append(rc.astype(np.float32))
        used.append(record)
    return left_points,right_points,size,used,rejected


def fit_calibration(objects,left,right,size):
    """독립 intrinsic 보정 후 CALIB_FIX_INTRINSIC으로 stereo R,t 추정."""
    if len(objects)<3 or len(objects)!=len(left) or len(left)!=len(right):
        raise ValueError('최소 3쌍의 동일 길이 observation 목록이 필요합니다.')
    criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,100,1e-8)
    rms_l,kl,dl,_,_=cv2.calibrateCamera(objects,left,size,None,None,criteria=criteria)
    rms_r,kr,dr,_,_=cv2.calibrateCamera(objects,right,size,None,None,criteria=criteria)
    rms,kl,dl,kr,dr,R,t,_,_=cv2.stereoCalibrate(objects,left,right,kl,dl,kr,dr,size,
                                               criteria=criteria,flags=cv2.CALIB_FIX_INTRINSIC)
    validate_intrinsics(kl); validate_intrinsics(kr); validate_pose(R,t)
    if not np.isfinite([rms_l,rms_r,rms]).all() or np.linalg.norm(t)<=1e-12:
        raise ValueError('보정 결과가 유효하지 않습니다.')
    return dict(K_left=kl.tolist(),K_right=kr.tolist(),D_left=dl.reshape(-1).tolist(),
                D_right=dr.reshape(-1).tolist(),R=R.tolist(),t=t.reshape(3).tolist(),
                image_size_left=list(size),image_size_right=list(size),
                camera_model='opencv_pinhole_5',image_state='raw'), dict(left_rms_px=float(rms_l),right_rms_px=float(rms_r),stereo_rms_px=float(rms))


def distorted_project(object_points,R,t,K,D):
    """진단용 5계수 Brown-Conrady 투영 직접 계산. core 투영과 별도.

    정규화 (x,y)에 radial 1+k1*r²+k2*r⁴+k3*r⁶ 및 tangential p1,p2를
    적용하고 K로 픽셀 변환한다. cv2.projectPoints를 사용하지 않는다.
    """
    camera=np.asarray(object_points,float)@np.asarray(R).T+np.asarray(t).reshape(3)
    if (camera[:,2]<=0).any():
        raise ValueError('보드 pose가 카메라 뒤쪽입니다.')
    x,y=(camera[:,:2]/camera[:,2:]).T
    k1,k2,p1,p2,k3=np.asarray(D).reshape(5)
    r2=x*x+y*y
    radial=1+k1*r2+k2*r2*r2+k3*r2*r2*r2
    xd=x*radial+2*p1*x*y+p2*(r2+2*x*x)
    yd=y*radial+p1*(r2+2*y*y)+2*p2*x*y
    h=np.column_stack((xd,yd,np.ones(len(x))))@np.asarray(K).T
    return h[:,:2]/h[:,2:]


def evaluate_observations(model,objects,left,right):
    """left 보드 pose만 PnP로 구하고 고정된 stereo R,t로 right를 예측.

    validation에서 K,D,R,t를 재학습하지 않는다. 보드의 자세만 left에서
    추정하므로 right prediction error로 stereo 일관성을 확인할 수 있다.
    """
    kl,kr=np.asarray(model['K_left']),np.asarray(model['K_right'])
    dl,dr=np.asarray(model['D_left']),np.asarray(model['D_right'])
    R,t=np.asarray(model['R']),np.asarray(model['t'])
    rows=[]
    for obj,lc,rc in zip(objects,left,right):
        ok,rv,tv=cv2.solvePnP(obj,lc,kl,dl,flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            raise ValueError('검증 보드 pose 추정 실패')
        board_R=cv2.Rodrigues(rv)[0]
        board_t=tv.reshape(3)
        lp=distorted_project(obj,board_R,board_t,kl,dl)
        rp=distorted_project(obj,R@board_R,R@board_t+t,kr,dr)
        le=np.linalg.norm(lp-lc,axis=1); re=np.linalg.norm(rp-rc,axis=1)
        rows.append(dict(left_rms_px=float(np.sqrt(np.mean(le**2))),
                         right_prediction_rms_px=float(np.sqrt(np.mean(re**2)))))
    return rows


def load_raw_model(path):
    data=json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data,dict) or data.get('camera_model')!='opencv_pinhole_5' or data.get('image_state')!='raw':
        raise ValueError('보정 도구가 생성한 raw_camera_model.json이 필요합니다. 이미 보정된 모델은 사용하지 마세요.')
    for side in ('left','right'):
        validate_intrinsics(data.get('K_'+side))
        D=np.asarray(data.get('D_'+side),float)
        size=np.asarray(data.get('image_size_'+side))
        if D.shape!=(5,) or not np.isfinite(D).all():
            raise ValueError('D는 유한한 5개 계수여야 합니다.')
        if size.shape!=(2,) or not np.issubdtype(size.dtype,np.integer) or (size<=0).any():
            raise ValueError('image_size는 양의 정수 [width,height]여야 합니다.')
    _,t=validate_pose(data.get('R'),data.get('t'))
    if np.linalg.norm(t)<=1e-12 or not isinstance(data.get('units'),str) or not data['units']:
        raise ValueError('양의 baseline과 문자열 units가 필요합니다.')
    return data


def reconstruction_calibration(model):
    """출력 이미지는 K 유지, crop/resize/rectify 없음. R,t도 그대로 유지."""
    keys=('K_left','K_right','R','t','units','image_size_left','image_size_right')
    return dict({key:model[key] for key in keys},image_state='undistorted',
                note='Use only images undistorted with unchanged K, size, and camera orientation.')
