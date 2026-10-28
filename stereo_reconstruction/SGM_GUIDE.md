# 직접 구현한 SGM 스테레오 복원

두 장의 보정된 이미지에서 신뢰할 수 있는 XYZRGB를 얻는 모듈이다. HTML은 포함하지 않는다.
기존 `window` 방식은 그대로 유지하며, `--method sgm`으로 새 모드를 선택한다.

## 구현 내용

1. **매칭 비용:** 기존 5×5 Census24 Hamming + clipped intensity difference를 재사용한다.
2. **경계 보존 비용 집계:** 3×3 이웃에서 좌·우 이미지의 밝기차가 큰 이웃의 비중을 낮춘다.
   `w=exp(-max(|L(p)-L(q)|, |R(p-d)-R(q-d)|)/12)`. 유효한 이웃만 사용한다.
3. **직접 구현한 8방향 SGM:** 가로·세로·대각선 경로에서 다음 식을 누적한다.

   ```text
   Lr(p,d) = C(p,d) + min(
       Lr(p-r,d),
       Lr(p-r,d-1)+P1,
       Lr(p-r,d+1)+P1,
       min_k Lr(p-r,k)+P2(p,p-r)
   ) - min_k Lr(p-r,k)
   S(p,d) = mean_r Lr(p,d)
   ```

   `P1=2`, `P2(p,q)=max(P1+0.001,24/(1+|I(p)-I(q)|/10))`.
   영상 경계에서 깊이 점프를 허용하되 항상 P2>P1이다. NumPy로 독립 scanline들을 동시에 계산하며,
   전체 경로 volume 대신 직전 행/열만 유지한다. 표준 SGM의 경로 근사이며 전역 최적해를 보장하지 않는다.
4. **독립적인 오른쪽 SGM:** 같은 raw 비용을 오른쪽 좌표로 이동한 뒤 오른쪽 영상 경계로 다시 누적한다.
5. **시차 선택:** 최저 비용, 인접 ±1 후보를 제외한 두 번째 최저 비용, 3점 포물선 서브픽셀 보정을 사용한다.
6. **신뢰도 필터:** 양측 uniqueness margin, 좌우 시차 오차, 선택 비용을 결합한다.

   ```text
   margin = min(left_margin, matched_right_margin)
   confidence = margin/(margin+0.08) * exp(-LR_error) * exp(-photo_cost/24)
   ```

   기본 기준은 `confidence>=0.35`, `margin>=0.04`, `LR_error<=1px`, `photo_cost<=16`이다.
   주변 8개 중 색 차이<30, 시차 차이<2px인 유효 이웃 2개 이상을 두 번 확인해 고립점을 제거한다.
   **신뢰도는 보정된 확률이 아닌 휴리스틱 점수**다. 검색 범위 끝의 해는 잘린 minimum일 수 있어 거절한다.
7. **3D:** 기존 직접 DLT(SVD) → 깊이/거리/영상 범위 필터 → 원본 RGB → JSON/PLY를 그대로 사용한다.

SGM, 비용 집계, 서브픽셀 추정, 필터, DLT를 직접 구현했다. NumPy는 배열/SVD,
OpenCV는 이미지 입출력·색변환·기존 정렬의 resampling에 사용한다.
`StereoSGBM`, `StereoBM`, `triangulatePoints`, `reprojectImageTo3D`, 외부 stereo 라이브러리는 호출하지 않는다.
점 복제, 구멍 보간, GT를 사용한 매칭·필터링은 하지 않는다.

## 기존 보정 파이프라인에서 실행

프로젝트 폴더에서 Python 환경을 활성화한 후 실행한다. 입력은 기존 `undistort_stereo.py`로
왜곡을 제거한 이미지와 그 이미지에 맞는 calibration.json이어야 한다.

```bash
python dense_reconstruct.py \
  --method sgm \
  --left sample_undistorted/left.png \
  --right sample_undistorted/right.png \
  --calibration sample_undistorted/calibration.json \
  --min-disparity 1 --max-disparity 160 \
  --max-depth 80 --max-3d-distance 100 \
  --output sgm_sample_results
```

위 깊이/거리 제한은 기존 OpenCV 샘플의 `board_square` 단위 예시다. 실제 촬영에서는 calibration의 단위와
장면 거리에 맞게 바꾼다. max-disparity도 실제 시차 범위를 포함해야 한다. 이미지 순서는 왼쪽/오른쪽이다.
동일 조건의 기존 방식은 `--method window`로 실행한다. 기존 CLI 기본값도 window로 유지했다.

SGM 전용 옵션:

| 옵션 | 기본값 | 의미 |
|---|---:|---|
| `--p1`, `--p2` | 2, 24 | 작은 시차 변화 / 큰 시차 점프 벌점 |
| `--edge-scale` | 10 | 밝기 경계에 따른 P2 감소 정도 |
| `--min-confidence` | 0.35 | 신뢰도 하한; 높일수록 더 적고 보수적인 점 |
| `--sgm-uniqueness` | 0.04 | 독립적인 후보 사이의 비용 margin 하한 |
| `--sgm-max-cost` | 16 | 선택 비용 상한 |
| `--lr-threshold` | 1 | 좌우 시차 일관성 기준(px), 두 방식 공통 |

`--window`, `--uniqueness`, `--min-texture`, `--max-cost`는 기존 window 모드용이다.
SGM은 고정된 3×3 경계 가중 비용 집계를 사용한다. 비용 volume은 최대60M entries로 제한한다.
큰 이미지에서는 이미지·K·시차 범위를 일관되게 축소하거나 메모리를 고려해 범위를 설정해야 한다.

## 공개 정답 데이터 비교 재현

Middlebury 공식 trainingQ 데이터의 Motorcycle, Pipes, Adirondack을 사용한다.
기존 모드와 새 모드는 동일 해상도·보정값·공개 `ndisp` 범위에서 실행한다.
GT를 읽기 전에 모든 예측과 3D 출력을 생성한다. GT는 이후 평가와 reference 그림에만 사용한다.

```bash
python benchmark_sgm.py --download --data-root middlebury --output sgm_benchmark_results
python -m pip install -r requirements-visualization.txt
python visualize_sgm.py --data-root middlebury --results sgm_benchmark_results
python -m unittest discover -s tests -v
```

다운로드는 공식 입력ZIP 약30MB + GT ZIP 약13MB이며 지정된 세 장면만 추출한다.
원본 URL과 ZIP SHA256은 `middlebury/sources.json`에 기록된다.
최종 신뢰도 기준은 Motorcycle 개발 비교 후 선택했으며 다른 장면에 같은 값을 적용했다.
Adirondack은 기준을 고정한 뒤 추가로 실행했다. 세 장면 결과를 전체 데이터셋 성능이나 공식 순위로 일반화하지 않는다.

평가 항목은 `metrics.json`에 기록된다.

- all_gt: 유효한 GT가 있는 영역에서 출력된 점의 MAE, RMSE, >1px/>2px 오류율.
- coverage: GT 유효 픽셀 중 복원한 비율. 전체 출력 포인트 수와 분모가 다르다.
- missing_or_bad2: 미복원 또는 >2px 오류인 픽셀을 전체 GT 유효 영역으로 나눈 비율.
- common_pixels: 두 방식이 모두 복원한 같은 픽셀에서의 오차.
- non_occluded: 공식 `mask0nocc==255` 영역.
- boundary: GT 4-neighbor 차이>1px 지점 주변2px. 경계 마스크는 평가에만 사용한다.
- unfiltered: 필터 전 winner의 오차. 무효 border와 가림 영역도 포함되므로 최종 출력 오차와 구분한다.

모든 pixel 수치는 Q 해상도 기준이다. 2px(Q)는 8px(full)에 해당하므로 공식 full-resolution 점수와 비교하지 않는다.
기존과 새 방식은 서로 다른 고정 필터 설정을 가진 완성된 파이프라인으로 비교한다.

PFM 주의: Middlebury 공식 SDK처럼 header scale은 부호로 endian만 판별하고,
float payload를 pixel 시차로 그대로 읽는다. `-0.003922`를 곱하면 잘못된 평가가 된다.
또한 두 카메라의 cx가 다르므로 `Z=fB/(d+cx_right-cx_left)`이다.
calibration의 baseline은 mm이며 DLT 출력도 mm다. K의 cx 차이와 별도 doffs 필드는 반올림 때문에
0.001px 정도 다를 수 있어 DLT 검증에는 K의 cx 차이를 사용한다.

## 팀원에게 전달할 데이터

### 공통 XYZRGB

- 일반 dense 실행: `06_point_cloud.ply`, `07_point_cloud.json`, `point_cloud.npy`
- Middlebury benchmark: 각 `SCENE/sgm/point_cloud.ply`, `point_cloud.json`, `point_cloud.npy`
- JSON의 `points` 행은 `[X,Y,Z,R,G,B]`, 단위는 JSON `units`에 있다.
- 좌표계는 왼쪽 카메라 기준 **+X 오른쪽, +Y 아래쪽, +Z 앞쪽**.
- 시각화는 보기 편하게 `(X,Z,-Y)`로 표시하고 mm를 m로 변환하지만 저장 좌표는 바꾸지 않는다.

### `point_attributes.npz`

같은 인덱스가 PLY/JSON의 같은 점이다. Python에서 `np.load()`로 읽는다.

| 배열 | 의미 |
|---|---|
| `pixel_left`, `pixel_right` | 각 3D 점의 대응 픽셀 N×2 |
| `disparity` | 각 점의 정렬된 시차 N |
| `confidence` | 각 점의 휴리스틱 신뢰도 N |
| `valid_mask` | 최종 유효 픽셀 H×W |
| `confidence_map` | 유효 영역의 신뢰도 H×W; 무효 영역은0 |
| `rectified_pixel_left` | 일반 dense 실행에서 추가: 정렬 좌표 N×2 |
| `disparity_map` | Middlebury 실행에서 추가: 필터 전 시차 H×W; valid_mask와 함께 사용 |

일반 dense 실행의 전체 시차·정렬행렬은 기존 `dense_data.npz`에 있다.
표면을 연결할 경우 픽셀 이웃 관계와 깊이 불연속을 함께 확인해야 한다. 무효 영역이나
큰 깊이 점프를 가로지르는 삼각형은 만들지 않는 것이 좋다.

## 검증과 한계

테스트는8방향 경로를 느린 scalar reference와 비교하고, 경계 점프 보존·알려진 시차·가림·평탄한 이미지·
신뢰도 기준 강화·PFM endian/행순서·principal point offset·실제 pipeline 출력 정합성을 확인한다.
JSON/PLY/NumPy/attribute의 점 개수, RGB, 좌표 정합성도 benchmark에서 확인한다.

근접0인 재투영 오차는 정렬된 시차로 만든 대응의 기하학적 성질이며 매칭 정확도의 증거가 아니다.
반사·반복무늬·가림·무늬 없는 영역에는 여전히 오차/빈 곳이 남는다. SGM의 영상 경계는 실제 깊이 경계와
항상 일치하지 않는다. 두 사진에서 관측되지 않은 뒷면은 복원하지 않는다.

출처:
- [Middlebury 데이터/보정 형식](https://vision.middlebury.edu/stereo/data/scenes2014/)
- [Middlebury 공식 Q 데이터와 GT](https://vision.middlebury.edu/stereo/submit3/)
- [Middlebury SDK 평가 규약](https://vision.middlebury.edu/stereo/submit3/zip/MiddEval3/README.txt)
- [공식 PFM reader](https://vision.middlebury.edu/stereo/submit3/zip/MiddEval3/code/imageLib/ImageIO.cpp)
- [SGM 원 저자 설명](https://elib.dlr.de/73119/1/180Hirschmueller.pdf)
- Scharstein et al., “High-Resolution Stereo Datasets with Subpixel-Accurate Ground Truth”, GCPR 2014.
