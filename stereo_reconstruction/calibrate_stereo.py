"""체커보드 이미지로 K,D,R,t를 추정하고 복원용 calibration.json을 생성."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from calibration_tools import (board_points,read_pairs,pairs_from_directories,collect_observations,
                               fit_calibration,evaluate_observations,reconstruction_calibration)


def calibrate(pairs, output, cols, rows, square_size, units, validation_pairs=None, min_pairs=8):
    if min_pairs<3 or not isinstance(units,str) or not units.strip():
        raise ValueError('min-pairs>=3 및 명시적 units가 필요합니다.')
    obj=board_points(cols,rows,square_size)
    validation_pairs=validation_pairs or []
    train_files={str(p.resolve()) for pair in pairs for p in pair}
    test_files={str(p.resolve()) for pair in validation_pairs for p in pair}
    if train_files & test_files:
        raise ValueError('학습과 검증 이미지가 겹칩니다. 독립 검증용 쌍을 지정하세요.')
    output=Path(output); output.mkdir(parents=True,exist_ok=True)
    left,right,size,used,rejected=collect_observations(pairs,(cols,rows),output/'training_corners')
    report=dict(input_pairs=len(pairs),detected_pairs=len(used),rejected_pairs=rejected,used_pairs=used,
                inner_corners=[cols,rows],square_size=square_size,units=units,opencv_version=cv2.__version__)
    report_path=output/'calibration_report.json'
    report_path.write_text(json.dumps(report,indent=2),encoding='utf-8')
    if len(used)<min_pairs:
        raise ValueError(f'검출 성공 {len(used)}쌍, 최소 {min_pairs}쌍 필요. corner 그림을 확인하세요.')
    objects=[obj.copy() for _ in used]
    model,rms=fit_calibration(objects,left,right,size)
    model.update(units=units,board_inner_corners=[cols,rows],square_size=square_size)
    training=evaluate_observations(model,objects,left,right)
    report.update(rms,training_per_pair=training,baseline=float(np.linalg.norm(model['t'])),
                  baseline_units=units,warnings=[])
    if len(used)<15:
        report['warnings'].append('실촬영에서는 다양한 위치/기울기/거리의 20~30쌍 확보를 권장합니다.')
    if rms['stereo_rms_px']>1:
        report['warnings'].append('Stereo RMS가 1px를 넘습니다. corner 순서, 고정 rig, 이미지 품질을 확인하세요.')
    if validation_pairs:
        vl,vr,_,vu,vrej=collect_observations(validation_pairs,(cols,rows),output/'validation_corners',size)
        report.update(validation_used_pairs=vu,validation_rejected_pairs=vrej)
        if not vu:
            raise ValueError('검증 이미지에서 보드를 찾지 못했습니다.')
        metrics=evaluate_observations(model,[obj.copy() for _ in vu],vl,vr)
        report['validation_per_pair']=metrics
        report['validation_right_prediction_rms_px']=float(np.sqrt(np.mean([m['right_prediction_rms_px']**2 for m in metrics])))
    else:
        report['warnings'].append('별도 validation 쌍이 없어 독립 검증은 수행하지 않았습니다.')
    (output/'raw_camera_model.json').write_text(json.dumps(model,indent=2,allow_nan=False),encoding='utf-8')
    (output/'calibration.json').write_text(json.dumps(reconstruction_calibration(model),indent=2,allow_nan=False),encoding='utf-8')
    report_path.write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    np.savez(output/'detected_corners.npz',object_points=obj,points_left=np.asarray(left),points_right=np.asarray(right))
    print(f'Accepted pairs: {len(used)}/{len(pairs)}; skipped: {len(rejected)}')
    print(f'Left RMS: {rms["left_rms_px"]:.6f}px; Right RMS: {rms["right_rms_px"]:.6f}px; Stereo RMS: {rms["stereo_rms_px"]:.6f}px')
    print(f'Baseline: {report["baseline"]:.6f} {units}')
    if validation_pairs:
        print(f'Held-out right prediction RMS: {report["validation_right_prediction_rms_px"]:.6f}px')
    for warning in report['warnings']:
        print('주의:',warning)
    print(f'Saved: {output}/raw_camera_model.json and calibration.json')
    return model,report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pairs',help='[{left,right},...] JSON; 경로는 manifest 기준')
    p.add_argument('--left-dir'); p.add_argument('--right-dir')
    p.add_argument('--validation-pairs',help='학습에 쓰지 않을 별도 pairs JSON')
    p.add_argument('--cols',type=int,default=9); p.add_argument('--rows',type=int,default=6)
    p.add_argument('--square-size',type=float,required=True)
    p.add_argument('--units',required=True,help='예: m, mm, board_square')
    p.add_argument('--min-pairs',type=int,default=8)
    p.add_argument('--output',default='calibration_result')
    a=p.parse_args()
    if bool(a.pairs)==bool(a.left_dir or a.right_dir) or (not a.pairs and not (a.left_dir and a.right_dir)):
        p.error('--pairs 또는 --left-dir/--right-dir 한 쌍을 지정하세요.')
    try:
        pairs=read_pairs(a.pairs) if a.pairs else pairs_from_directories(a.left_dir,a.right_dir)
        validation=read_pairs(a.validation_pairs) if a.validation_pairs else None
        calibrate(pairs,a.output,a.cols,a.rows,a.square_size,a.units,validation,a.min_pairs)
    except (ValueError,OSError,cv2.error) as error:
        p.exit(2,f'오류: {error}\n')


if __name__=='__main__':
    main()
