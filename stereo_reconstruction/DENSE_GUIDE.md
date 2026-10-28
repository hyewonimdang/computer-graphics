# 촘촘한 포인트 클라우드 모드

기존 ORB 기반 `main.py`와 별도로 `dense_reconstruct.py`를 추가했습니다.
보정된 이미지와 K/R/t로 매칭 가능한 픽셀마다 하나의 점을 생성합니다.
기존 sparse 파이프라인/API는 변경하지 않았습니다.

## 실행

기존 보정 도구로 만든 undistorted pair와 그 calibration.json을 사용합니다.
공개 샘플 결과가 이미 있다면 다음 명령을 실행하세요:

```bash
.venv/bin/python dense_reconstruct.py \
  --left sample_undistorted/left.png --right sample_undistorted/right.png \
  --calibration sample_undistorted/calibration.json \
  --max-disparity 160 --window 9 --uniqueness 0.15 \
  --max-depth 80 --max-3d-distance 100 \
  --output comparison/dense

# 기존 방식도 같은 이미지·K/R/t·depth/거리 한계로 비교
.venv/bin/python main.py \
  --left sample_undistorted/left.png --right sample_undistorted/right.png \
  --calibration sample_undistorted/calibration.json --phase 6 \
  --max-depth 80 --max-3d-distance 100 --output comparison/sparse
```

80/100은 이번 샘플 비교에 적용한 공통 깊이/거리 한계이며 단위는 board_square입니다.
실촬영에는 장면과 t의 단위에 맞게 바꾸세요. 함수/CLI 기본 한계는 기존과 같이 1000입니다.
이미지 해상도는 양쪽 같아야 합니다. 원본 왜곡 이미지나 raw_camera_model.json은 입력하지 마세요.

## 구현 원리

- **직접 정렬 행렬 계산**: 오른쪽 카메라 중심 `C=-R.T t` 방향을 새 X축으로 하고,
  두 optical axis의 평균 방향과 외적으로 나머지 축을 구성합니다.
  `Q_right=Q_left R.T`, 공통 K_rect를 사용해 대응점을 같은 수평선에 배치합니다.
- **직접 역매핑 좌표 계산**: `K Q.T K_rect^-1 x_rect`로 원본 픽셀을 구합니다.
  OpenCV remap은 계산된 좌표에서 영상만 보간합니다. 입력 밖 영역을 mask로 제외합니다.
- **직접 Census 변환**: 5×5 이웃과 중심 밝기의 순서를 24bit로 인코딩합니다.
- **직접 비용 계산**: 좌 픽셀 x와 우 픽셀 x-d의 Census Hamming 거리와 제한된 밝기차를
  더하고, integral image로 window 평균을 구합니다.
- 최소 비용 시차를 선택하고 인접 세 후보의 parabola로 subpixel 보정합니다.
- 인접 시차 ±1을 제외한 차선 후보와의 구별 정도, local texture, 좌우 독립 매칭 일치도를 검사합니다.
- 주변 8개 중 3개 이상이 비슷한 시차를 갖는지 두 번 검사해 고립된 점을 줄입니다.
- 정렬된 대응점을 원래 pinhole 좌표로 되돌리고 **기존 직접 DLT** 및 depth/거리/재투영 필터를 사용합니다.
- **최종 XYZ는 원래 왼쪽 카메라 좌표계**, RGB는 원래 왼쪽 이미지에서 가져옵니다.
- 점 복제, 빈 영역 메우기, surface/mesh interpolation은 하지 않습니다.

`stereoRectify`, `StereoBM/SGBM`, `triangulatePoints`, `reprojectImageTo3D`는 이 모드에서
사용하지 않습니다. NumPy 기본 연산과 기존 직접 구현 기하 계산을 사용합니다.

## 옵션

| 옵션 | 기본값 | 영향 |
|---|---:|---|
| `--min-disparity` | 1 | 최소 검색 시차(px) |
| `--max-disparity` | 160 | 최대 검색 시차(px), 이미지 폭보다 작아야 함 |
| `--window` | 9 | 매칭 비용 집계 창 크기, 홀수>=3 |
| `--uniqueness` | 0.15 | 클수록 모호한 후보를 더 제거 |
| `--lr-threshold` | 1 | 좌우 매칭 시차 차이 한계(px) |
| `--min-texture` | 3 | 지역 밝기 표준편차 최소값 |
| `--max-cost` | 9 | 최적 window 비용 한계 |
| `--min-depth` | 1e-6 | 양쪽 양의 depth 하한 |
| `--max-depth` | 1000 | 양쪽 depth 상한, t와 같은 단위 |
| `--max-3d-distance` | 1000 | 양쪽 카메라로부터 거리 한계 |

검색 범위 양 끝에서 최소 비용이 나온 픽셀은 잘린 해일 수 있어 제외합니다.
평행 카메라의 대략적인 관계 `Z=f*baseline/disparity`로 필요한 시차 범위를 잡을 수 있습니다.
작은 window는 세부 경계를 보존하지만 잡음에 약하고, 큰 window는 경계를 섞을 수 있습니다.
고해상도에서는 비용 volume 메모리가 커집니다. 약 1억 float32 비용을 넘으면 오류를 냅니다.
단순히 사진만 축소하지 말고 K도 같이 바꾸어야 합니다.

## 출력

- `01_rectified_left.png`, `02_rectified_right.png`: 공통 수평 좌표계 이미지
- `03_rectified_pair.jpg`: 수평선 격자 확인용
- `04_disparity.png`: 최종 통과 영역의 시차 색상 그림; 검정은 무효 영역
- `05_valid_mask.png`: 통과 픽셀 mask
- `06_point_cloud.ply`, `07_point_cloud.json`, `point_cloud.npy`: 기존과 같은 XYZRGB 형식
- `dense_data.npz`: 시차, 마스크, 좌우 일치 오차, 비용, 원래/정렬 좌표 대응점, 정렬 행렬
- `diagnostics.json`: 파라미터·점 수·coverage·제거 수·시간·오차

`dense_data.npz`의 disparity 값은 무효 영역에도 계산값이 있으므로 반드시 final_mask와 함께 사용하세요.
원래 feature index는 존재하지 않으며 rectified_left/right로 각 최종 점을 추적합니다.

## 같은 공개 샘플 비교

OpenCV 4.11.0 공식 sample pair 13, 동일한 보정(K/R/t)과 왜곡 보정 영상으로 비교했습니다.
보정에는 11쌍, 별도 검증에는 13/14번 쌍을 사용한 이전 결과를 재사용했습니다.

| 항목 | 기존 ORB | 새 custom dense |
|---|---:|---:|
| 최종 점 수 | 193 | 44,238 |
| 점 수 배율 | 1 | 229.2 |
| 체커보드 내부 코너 polygon 영역의 점 | 6 | 6,280 |
| 최종 rectified 영상 coverage | 해당 없음 | 14.40% |

같은 depth 80/거리 100 한계로 비교했습니다. 기본 1000 한계에서는 44,773개가 나왔지만
아주 먼 점 535개가 추가되어 비교 그림에 불필요하게 큰 범위를 만들었습니다.
이번 비교의 한계는 양쪽에 동일하게 적용되며 기존 193개는 모두 남았습니다.

독립적인 코너 대응 검사:

- 검출한 체커보드 코너 54개 모두에서 dense 시차가 유효했습니다.
- 알려진 좌우 코너 대응과 비교한 시차 오차: 중앙값 0.114px, 평균 0.139px, 최대 0.786px.
- 1px를 초과한 코너: 0개.
- 코너 이외의 전체 장면에 ground truth가 있는 것은 아닙니다. 이 수치는 전체 정확도를 보장하지 않습니다.

**Dense 재투영 오차를 sparse와 직접 비교하면 안 됩니다.**
수평 정렬된 대응점 `(u,v) ↔ (u-d,v)`는 구성상 에피폴라 제약을 만족하므로, 잘못된 d라도
DLT 재투영 오차가 거의 0일 수 있습니다. 본 비교는 점 수, 좌우 일치도, 코너 시차 오차를 사용합니다.
반복 패턴, 무늬 없는 내부, 반사/가림/조명 차이에는 여전히 구멍이나 오매칭이 남을 수 있습니다.
이는 완전한 표면 복원이나 물체 분할 기능이 아닙니다.

## 검증 및 그림 재생성

```bash
.venv/bin/python -m unittest discover -s tests -v

.venv/bin/python verify_dense_sample.py \
  --comparison-dir comparison \
  --model sample_calibration_result/raw_camera_model.json \
  --left-raw sample_calibration/left13.jpg --right-raw sample_calibration/right13.jpg

# 선택: 그림 생성용 라이브러리 (핵심 복원에는 필요 없음)
.venv/bin/python -m pip install -r requirements-visualization.txt
.venv/bin/python compare_clouds.py \
  --sparse comparison/sparse --dense comparison/dense \
  --left sample_undistorted/left.png --output comparison
```

검증 스크립트는 기본 9×6 공개 샘플용이며 comparison 폴더 아래 sparse/dense 결과를 읽습니다.
기존 26개와 신규 4개를 합친 전체 **30개 테스트**가 통과했습니다.
새 테스트는 integral image 합, 회전 카메라의 수평 정렬과 원래 좌표 복원,
알려진 12px 시차, 가림 영역 제거, 무늬 없는 영상 및 잘못된 입력을 검사합니다.
JSON/PLY/NumPy 좌표·색상 일치와 통과 픽셀당 정확히 한 점인지도 확인했습니다.

비교 그림은 같은 축 범위·시점에서 모든 출력 점을 표시합니다.
별도 `board_detail.png`는 체커보드 내부 polygon의 공통 확대 범위만 보여주는 그림입니다.
확대 범위는 dense XYZ의 0.5–99.5 백분위에 0.5단위를 더한 것이며, sparse 1개/dense 19개가
그림 밖에 있습니다. **저장된 전체 cloud에서 이 점들을 제거한 것은 아닙니다.**
