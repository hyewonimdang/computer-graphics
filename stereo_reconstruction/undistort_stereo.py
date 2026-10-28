"""raw_camera_model.json으로 stereo pair 왜곡 보정; 크기/K/R/t를 유지한다."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from calibration_tools import load_raw_model,reconstruction_calibration
from image_io import load_stereo_images,save_image
from visualization import side_by_side


def undistort_pair(left_path,right_path,model_path,output):
    model=load_raw_model(model_path)
    left,right=load_stereo_images(left_path,right_path)
    output=Path(output)
    source_paths={Path(p).resolve() for p in (left_path,right_path,model_path)}
    targets=[output/name for name in ('left.png','right.png','calibration.json',
             'valid_left.png','valid_right.png','left_before_after.jpg','right_before_after.jpg','undistortion_report.json')]
    if source_paths & {p.resolve() for p in targets}:
        raise ValueError('출력 경로가 입력 파일과 겹칩니다. 별도 output 폴더를 사용하세요.')
    for side,image in (('left',left),('right',right)):
        if [image.shape[1],image.shape[0]]!=model['image_size_'+side]:
            raise ValueError(f'{side} 이미지 해상도가 calibration과 다릅니다. resize/crop하지 않은 같은 설정의 이미지를 사용하세요.')
    report=dict(source_left=str(Path(left_path).resolve()),source_right=str(Path(right_path).resolve()),
                source_model=str(Path(model_path).resolve()),operation='undistort only; K/size/R/t unchanged; no crop/rectification')
    for side,image in (('left',left),('right',right)):
        K=np.asarray(model['K_'+side],float); D=np.asarray(model['D_'+side],float)
        h,w=image.shape[:2]
        # 새 카메라 행렬=기존 K, rectification R=I. 따라서 extrinsic은 그대로.
        map_x,map_y=cv2.initUndistortRectifyMap(K,D,np.eye(3),K,(w,h),cv2.CV_32FC1)
        corrected=cv2.remap(image,map_x,map_y,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
        valid=(map_x>=0)&(map_x<=w-1)&(map_y>=0)&(map_y<=h-1)
        save_image(output/(side+'.png'),corrected)
        save_image(output/('valid_'+side+'.png'),valid.astype(np.uint8)*255)
        save_image(output/(side+'_before_after.jpg'),side_by_side(image,corrected))
        report[side+'_valid_fraction']=float(valid.mean())
    (output/'calibration.json').write_text(json.dumps(reconstruction_calibration(model),indent=2,allow_nan=False),encoding='utf-8')
    (output/'undistortion_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(f'Saved {output}/left.png, right.png, calibration.json; K/size/R/t unchanged')
    print(f'Valid pixels: left={report["left_valid_fraction"]:.2%}, right={report["right_valid_fraction"]:.2%}')
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--left',required=True); p.add_argument('--right',required=True)
    p.add_argument('--model',required=True,help='calibrate_stereo.py가 생성한 raw_camera_model.json')
    p.add_argument('--output',default='undistorted')
    a=p.parse_args()
    try:
        undistort_pair(a.left,a.right,a.model,a.output)
    except (ValueError,OSError,cv2.error) as error:
        p.exit(2,f'오류: {error}\n')


if __name__=='__main__':
    main()
