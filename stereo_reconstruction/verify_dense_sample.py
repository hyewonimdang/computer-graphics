from pathlib import Path
import argparse,json
import numpy as np,cv2
from calibration_tools import detect_corners
from image_io import load_image
from geometry import from_homogeneous,to_homogeneous
parser=argparse.ArgumentParser(description='Known checkerboard corners vs dense disparity; export consistency verification')
parser.add_argument('--comparison-dir',required=True)
parser.add_argument('--model',required=True)
parser.add_argument('--left-raw',required=True)
parser.add_argument('--right-raw',required=True)
args=parser.parse_args()
root=Path(args.comparison_dir)
model=json.loads(Path(args.model).read_text())
with np.load(root/'dense/dense_data.npz') as data:
    K=data['K_rect']; ql=data['Q_left']; qr=data['Q_right']; disp=data['disparity']; mask=data['final_mask']
left=load_image(args.left_raw); right=load_image(args.right_raw)
rect=[]
for image,side,Q in ((left,'left',ql),(right,'right',qr)):
    points=detect_corners(image,tuple(model['board_inner_corners']))
    und=cv2.undistortPoints(points.reshape(-1,1,2),np.array(model['K_'+side]),np.array(model['D_'+side])).reshape(-1,2)
    rect.append(from_homogeneous(to_homogeneous(und)@(K@Q).T))
expected=rect[0][:,0]-rect[1][:,0]
p=np.rint(rect[0]).astype(int)
inbounds=(p[:,0]>=0)&(p[:,0]<mask.shape[1])&(p[:,1]>=0)&(p[:,1]<mask.shape[0])
accepted=inbounds.copy(); accepted[inbounds]&=mask[p[inbounds,1],p[inbounds,0]]
observed=disp[p[accepted,1],p[accepted,0]]
error=np.abs(observed-expected[accepted])
sparse=np.load(root/'sparse/point_cloud.npy'); dense=np.load(root/'dense/point_cloud.npy')
dense_parameters=json.loads((root/'dense/diagnostics.json').read_text())['parameters']
sparse_limits=json.loads((root/'sparse/diagnostics.json').read_text())['filter_limits']
if dense_parameters['max_depth']!=sparse_limits['max_depth'] or dense_parameters['max_distance']!=sparse_limits['max_distance']:
    raise ValueError('공통 depth/거리 한계로 실행한 결과를 비교하세요.')
summary=dict(sparse_points=len(sparse),dense_points=len(dense),increase_factor=len(dense)/len(sparse),
    shared_limits=dict(max_depth=dense_parameters['max_depth'],max_distance=dense_parameters['max_distance'],units=model['units']),
    dense_coverage_percent=float(mask.mean()*100),known_board_corners=len(rect[0]),dense_valid_at_corners=int(accepted.sum()),
    corner_disparity_median_absolute_error_px=float(np.median(error)) if len(error) else None,
    corner_disparity_mean_absolute_error_px=float(np.mean(error)) if len(error) else None,
    corner_disparity_max_absolute_error_px=float(np.max(error)) if len(error) else None,
    corner_bad_over_1px=int((error>1).sum()),
    caveat='Only detected checkerboard corners with valid dense matches are evaluated. Not a full-scene ground-truth accuracy measure. Reprojection errors are not directly comparable.')
(root/'comparison.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print(json.dumps(summary,indent=2))
# 최종 포맷과 통과 픽셀당 한 점인지 검증한다.
for folder,cloud in (('sparse',sparse),('dense',dense)):
    rows=json.loads((root/folder/'07_point_cloud.json').read_text())['points']
    np.testing.assert_array_equal(cloud,np.asarray(rows))
    lines=(root/folder/'06_point_cloud.ply').read_text().splitlines()
    assert f'element vertex {len(cloud)}' in lines
    np.testing.assert_array_equal(cloud,np.array([[float(v) for v in line.split()] for line in lines[lines.index('end_header')+1:]]))
assert mask.sum()==len(dense)
print('Export verification passed; one 3D point per accepted pixel.')
