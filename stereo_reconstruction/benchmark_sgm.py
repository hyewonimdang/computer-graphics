"""Middlebury trainingQ: 동일 입력에서 window/SGM 비교. GT는 평가에만 사용.

다운로드: 공식 ZIP 2개 약43MB, 지정 scene의 필요한 파일만 추출.
사용 예: python benchmark_sgm.py --download --data-root middlebury --output sgm_results
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
import urllib.request
import zipfile
import cv2
import numpy as np
from dense_stereo import dense_correspondences, box_sum
from sgm_stereo import sgm_correspondences
from camera import build_projection_matrix
from triangulation import triangulate_points_dlt
from exporter import export_json, export_ply

SOURCE = 'https://vision.middlebury.edu/stereo/submit3/'


def prepare_data(root, scenes):
    root = Path(root); root.mkdir(parents=True, exist_ok=True)
    records = []
    for name, files in [('MiddEval3-data-Q.zip', ('im0.png', 'im1.png', 'calib.txt')),
                        ('MiddEval3-GT0-Q.zip', ('disp0GT.pfm', 'mask0nocc.png'))]:
        dest = root/name
        url = SOURCE+'zip/'+name
        if not dest.exists():
            temp = dest.with_suffix('.partial')
            with urllib.request.urlopen(url, timeout=120) as response, temp.open('wb') as target:
                while True:
                    block = response.read(1024*1024)
                    if not block: break
                    target.write(block)
            temp.replace(dest)
        records.append(dict(url=url, sha256=hashlib.sha256(dest.read_bytes()).hexdigest()))
        with zipfile.ZipFile(dest) as archive:
            for scene in scenes:
                if not scene.isalnum(): raise ValueError('scene 이름은 영숫자만 허용합니다.')
                target = root/scene; target.mkdir(exist_ok=True)
                for file in files:
                    (target/file).write_bytes(archive.read(f'MiddEval3/trainingQ/{scene}/{file}'))
    (root/'sources.json').write_text(json.dumps(records, indent=2))


def read_pfm(path):
    """Middlebury PFM: scale의 부호는 endian, 데이터 자체가 pixel disparity.

    공식 SDK ImageIO.cpp ReadFilePFM처럼 scale 절댓값은 곱하지 않는다.
    예: -0.003922 header여도 float payload 40은 시차40px다.
    일반 radiance PFM의 scale 처리와 구분한다. 행은 bottom-up 저장.
    """
    with Path(path).open('rb') as handle:
        def line():
            value = handle.readline().strip()
            while value.startswith(b'#'): value = handle.readline().strip()
            return value
        if line() != b'Pf': raise ValueError('grayscale Pf 파일이 필요합니다.')
        w, h = map(int, line().split()); scale = float(line())
        if w <= 0 or h <= 0 or not np.isfinite(scale) or scale == 0: raise ValueError('잘못된 PFM header')
        data = np.frombuffer(handle.read(), dtype='<f4' if scale < 0 else '>f4')
        if data.size != w*h: raise ValueError('PFM 데이터 길이 불일치')
        return np.flipud(data.reshape(h, w)).astype(np.float32)


def read_calibration(path):
    values = {}
    for line in Path(path).read_text().splitlines():
        if '=' not in line: continue
        key, value = line.split('=', 1)
        if key in ('cam0', 'cam1'):
            values[key] = np.array([[float(x) for x in row.split()]
                                    for row in value.strip('[]').split(';')])
        else: values[key] = float(value)
    k0, k1 = values['cam0'], values['cam1']
    if k0.shape != (3,3) or k1.shape != (3,3) or values['baseline'] <= 0:
        raise ValueError('잘못된 Middlebury calibration')
    if not np.isclose(k1[0,2]-k0[0,2],values['doffs'],atol=.01):
        raise ValueError('principal point offset 불일치')
    return values


def boundary_mask(gt):
    """정답의 4-neighbor 시차 차이>1px인 곳에서 2px 주변; 평가 전용."""
    valid = np.isfinite(gt) & (gt > 0)
    safe = np.where(valid, gt, 0)
    edge = np.zeros(gt.shape, bool)
    for axis in (0, 1):
        s0 = [slice(None), slice(None)]; s1 = s0.copy()
        s0[axis] = slice(None, -1); s1[axis] = slice(1, None)
        s0, s1 = tuple(s0), tuple(s1)
        jump = valid[s0] & valid[s1] & (np.abs(safe[s0]-safe[s1]) > 1.)
        edge[s0] |= jump; edge[s1] |= jump
    return (box_sum(edge, 2) > 0) & valid


def metrics(disparity, accepted, gt, region):
    region = region & np.isfinite(gt) & (gt > 0)
    evaluated = region & accepted & np.isfinite(disparity)
    count, n = int(evaluated.sum()), int(region.sum())
    error = np.abs(disparity[evaluated]-gt[evaluated]).astype(float)
    good = int((error <= 2).sum())
    return dict(gt_pixels=n, evaluated_pixels=count,
        coverage_percent=100*count/n if n else None,
        mae_px=float(error.mean()) if count else None,
        rmse_px=float(np.sqrt((error**2).mean())) if count else None,
        bad1_percent=100*float((error>1).mean()) if count else None,
        bad2_percent=100*float((error>2).mean()) if count else None,
        correct_within_2px=good,
        correct_within_2px_percent_of_gt=100*good/n if n else None,
        missing_or_bad2_percent=100*(n-good)/n if n else None)


def export_reconstruction(result, left, calib, output):
    """공개 K0,K1,baseline으로 기존 직접 DLT. GT를 사용하지 않는다."""
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    y, x = np.nonzero(result['valid']); d = result['disparity'][y, x]
    p0 = np.column_stack((x, y)); p1 = np.column_stack((x-d, y))
    K0, K1, B = calib['cam0'], calib['cam1'], calib['baseline']
    P0 = build_projection_matrix(K0, np.eye(3), np.zeros(3))
    P1 = build_projection_matrix(K1, np.eye(3), np.array([-B,0,0]))
    xyz = triangulate_points_dlt(p0, p1, P0, P1)
    keep = np.isfinite(xyz).all(axis=1) & (xyz[:,2] > 0)
    # doffs가 있는 rectified dataset: Z=fB/(d+doffs), fB/d가 아님.
    # 독립적으로 소수점3자리 반올림한 doffs 필드와 K1.cx-K0.cx는 .001 차이 가능.
    expected = K0[0,0]*B/(d.astype(float)+K1[0,2]-K0[0,2])
    np.testing.assert_allclose(xyz[keep,2], expected[keep], rtol=1e-6, atol=1e-6)
    cloud = np.column_stack((xyz[keep], left[y[keep],x[keep],::-1]))
    export_ply(cloud, output/'point_cloud.ply'); export_json(cloud, output/'point_cloud.json', 'mm')
    np.save(output/'point_cloud.npy',cloud)
    final = np.zeros(result['valid'].shape,bool); final[y[keep],x[keep]]=True
    attrs=dict(pixel_left=p0[keep],pixel_right=p1[keep],disparity=d[keep],
               valid_mask=final,disparity_map=result['disparity'])
    if 'confidence' in result:
        attrs['confidence']=result['confidence'][y[keep],x[keep]]
        attrs['confidence_map']=np.where(final,result['confidence'],0)
    np.savez_compressed(output/'point_attributes.npz',**attrs)
    cv2.imwrite(str(output/'valid_mask.png'),final.astype(np.uint8)*255)
    # 저장 행 수·색상·좌표와 픽셀 attribute 대응 검증.
    loaded=json.loads((output/'point_cloud.json').read_text())
    assert loaded['count']==len(cloud)==int(final.sum())==len(attrs['disparity'])
    np.testing.assert_allclose(loaded['points'],cloud)
    with (output/'point_cloud.ply').open() as f:
        header=[]
        for line in f:
            header.append(line.strip())
            if line.strip()=='end_header': break
        assert f'element vertex {len(cloud)}' in header
        assert sum(1 for _ in f)==len(cloud)
    return final


def run_scene(data, output, scene):
    source=Path(data)/scene; dest=Path(output)/scene; dest.mkdir(parents=True,exist_ok=True)
    left=cv2.imread(str(source/'im0.png')); right=cv2.imread(str(source/'im1.png'))
    if left is None or right is None: raise ValueError(f'이미지 없음: {source}')
    calib=read_calibration(source/'calib.txt')
    if left.shape[:2] != (int(calib['height']),int(calib['width'])): raise ValueError('calibration 크기 불일치')
    # ndisp는 공개 입력 metadata; 정답의 min/max로 범위를 조절하지 않는다.
    maximum=int(calib['ndisp'])-1
    results={}; methods={}
    for method in ('window','sgm'):
        print(f'{scene}: {method}, {left.shape[1]}x{left.shape[0]}, d=0..{maximum}',flush=True)
        start=time.perf_counter()
        if method=='window': result=dense_correspondences(left,right,min_disparity=0,max_disparity=maximum)
        else: result=sgm_correspondences(left,right,min_disparity=0,max_disparity=maximum)
        elapsed=time.perf_counter()-start
        result['valid']=export_reconstruction(result,left,calib,dest/method)
        results[method]=result
        methods[method]=dict(points=int(result['valid'].sum()),matching_seconds=elapsed)
    # 모든 예측·출력을 만든 후에만 정답을 읽는다.
    gt=read_pfm(source/'disp0GT.pfm')
    if gt.shape!=left.shape[:2]: raise ValueError('GT 크기 불일치')
    nocc=cv2.imread(str(source/'mask0nocc.png'),cv2.IMREAD_GRAYSCALE)
    regions=dict(all_gt=np.isfinite(gt)&(gt>0),non_occluded=nocc==255,boundary=boundary_mask(gt))
    common=results['window']['valid']&results['sgm']['valid']
    for method,result in results.items():
        methods[method]['evaluation']={name:metrics(result['disparity'],result['valid'],gt,region) for name,region in regions.items()}
        methods[method]['common_pixels']=metrics(result['disparity'],common,gt,regions['all_gt'])
        methods[method]['unfiltered']=metrics(result['disparity'],np.ones(gt.shape,bool),gt,regions['all_gt'])
        selected=regions['all_gt']&result['valid']
        offset=calib['cam1'][0,2]-calib['cam0'][0,2]
        dz=np.abs((gt[selected]+offset)/(result['disparity'][selected]+offset)-1)
        methods[method]['mean_relative_depth_error_percent']=float(dz.mean()*100)
    report=dict(scene=scene,resolution='official trainingQ, unchanged',source=SOURCE,
        parameters=dict(window=dict(window=9,uniqueness=.15,lr_threshold=1.,min_texture=3.,max_cost=9.,support=3),
                        sgm=dict(p1=2.,p2=24.,edge_scale=10.,uniqueness=.04,lr_threshold=1.,max_cost=16.,min_confidence=.35,support=2,directions=8,cost_support='bilateral 3x3, sigma=12')),
        search_range=[0,maximum],methods=methods,
        notes=['GT is used for evaluation only, never matching or filtering.',
               'Filtered MAE is accompanied by coverage, common-pixel MAE, and missing-or-bad2.',
               'These are selected-scene Q-resolution experiments, not an official full-resolution leaderboard score.',
               'SGM defaults selected during Motorcycle development, then fixed across scenes. Adirondack is an additional unchanged-parameter check.',
               'SGM confidence is a heuristic score, not a calibrated probability.'])
    (dest/'metrics.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps(report,indent=2),flush=True)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data-root',default='middlebury'); p.add_argument('--output',default='sgm_results')
    p.add_argument('--scenes',nargs='+',default=['Motorcycle','Pipes','Adirondack']); p.add_argument('--download',action='store_true')
    a=p.parse_args()
    if a.download: prepare_data(a.data_root,a.scenes)
    reports=[run_scene(a.data_root,a.output,s) for s in a.scenes]
    Path(a.output,'summary.json').write_text(json.dumps(reports,indent=2,allow_nan=False))


if __name__=='__main__': main()
