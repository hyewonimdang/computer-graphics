법선 v2 기본 적용: [SURFACE_V2_GUIDE.md](SURFACE_V2_GUIDE.md). STP1/SRF1 wire layout 변경 없음.

# 표면 데이터 확장 (2026-10-28)

법선·metre splat 반경을 SRF1 보조 스트림/파일로 제공합니다. 기존 STP1 32-byte 레코드는 유지합니다.
새 규약·좌표계·협상·실행 방법은 [SURFACE_GUIDE.md](SURFACE_GUIDE.md)를 참고하세요.

# Stereo Image Reconstruction — Phase 1–6

## KITTI A 파이프라인 및 실제 WebSocket

`kitti_pipeline.py`와 `stream_server.py`로 KITTI Raw/Odometry 로딩, 직접 SGM·DLT,
제공 포즈 적용, world voxel 누적, 신규점 STP1 전송을 수행합니다.
`live`는 계산 직후 전송하고 `replay`는 저장 결과를 재생합니다.
실제 KITTI Raw5쌍으로180,552점/47패킷의 JS 수신 bytes 일치를 검증했습니다.
설치·실행·제약은 [KITTI_GUIDE.md](KITTI_GUIDE.md), B 연결은 [STREAM_PROTOCOL.md](STREAM_PROTOCOL.md)를 참고하세요.

컴퓨터 그래픽스 팀프로젝트용 **sparse stereo 3D reconstruction 코어**입니다.
이미지 2장 → ORB → F/에피폴라 검증 → E/P → 직접 DLT → 좌표·재투영 필터
→ RGB 할당 → N×6 XYZRGB 및 JSON/PLY까지 구현했습니다. WebGL 렌더링은 포함하지 않습니다.

## 촘촘한 포인트 클라우드 모드

**SGM 개선:** `dense_reconstruct.py --method sgm`으로 직접 구현한 8방향 SGM,
경계 가중 비용·적응 P2·신뢰도 필터를 사용할 수 있습니다. 기존 `window` 모드도 유지합니다.
Middlebury 정답 시차 비교는 `benchmark_sgm.py`, 형상 시각화는 `visualize_sgm.py`로 실행합니다.
구현·실행·전달 데이터 규약은 [SGM_GUIDE.md](SGM_GUIDE.md)를 참고하세요.

`dense_reconstruct.py`를 추가했습니다. 직접 구현한 수평 정렬·Census window 매칭으로
픽셀 대응점을 늘리고 기존 DLT·재투영·출력 함수를 재사용합니다.
동일 공개 샘플에서 193개 → 44,238개(약 229배)로 증가했습니다.
실행법과 정확도 해석은 [DENSE_GUIDE.md](DENSE_GUIDE.md)를 참고하세요.

## 실제 카메라 보정 도구

체커보드로 K/D/R/t를 구하는 `calibrate_stereo.py`, 왜곡 보정용
`undistort_stereo.py`, 공식 샘플 다운로드 및 별도 검증 도구를 추가했습니다.
촬영 준비부터 전체 실행 명령은 [CALIBRATION_GUIDE.md](CALIBRATION_GUIDE.md)를 참고하세요.
공개 샘플 13쌍 중 11쌍 학습/2쌍 별도 검증을 수행했고, 기존 복원 파이프라인까지 실행했습니다.

## 바로 실행

프로젝트 폴더에서 가상환경을 생성하여 사용합니다. 검증 환경은 Python 3.8.18,
NumPy 1.24.4, OpenCV 4.11.0입니다. 새 환경에서는 Python 3.11 이상도 사용할 수 있습니다.

```bash
cd "stereo_reconstruction"
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt -r requirements-streaming.txt
.venv/bin/python generate_demo.py --output demo

# 전체 파이프라인, OpenCV 비교와 CSV는 선택
.venv/bin/python main.py \
  --left demo/left.png --right demo/right.png \
  --phase 6 --calibration demo/calibration.json \
  --compare-opencv --csv --output results_phase6

# Phase 5까지만 실행
.venv/bin/python main.py \
  --left demo/left.png --right demo/right.png \
  --phase 5 --calibration demo/calibration.json --output results_phase5

# 전체 테스트
.venv/bin/python -m unittest discover -s tests -v
```

ZIP으로 받은 새 환경은 먼저 설치합니다:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

기존 사용 호환성을 위해 기본값은 `--phase 2`입니다. 전체 실행에는 `--phase 6`을
명시하세요. `--phase 1`은 특징점/매칭까지만, 2는 F/에피폴라 검사까지 실행합니다.
3 이상은 `--calibration`이 필요합니다. `--compare-opencv`는 4 이상에서,
`--csv`는 6에서만 사용할 수 있습니다. 실험마다 다른 `--output` 폴더를 사용하세요.
같은 이름의 파일은 덮어쓰며 이전 실행의 다른 단계 파일은 자동 삭제하지 않습니다.

## 입력 calibration과 좌표 약속

`demo/calibration.json`은 **합성 데모 전용**입니다. 실제 이미지에는 해당 촬영의
K_left/K_right 및 상대 R/t를 입력해야 합니다. 복원 코어 main.py는 calibration 추정이나 렌즈 왜곡 보정을 자동 수행하지 않습니다.
새 준비 도구 calibrate_stereo.py/undistort_stereo.py를 먼저 사용하세요. E 분해 기반 pose 추정은 포함하지 않습니다.

```json
{
  "K_left": [[600,0,450],[0,600,300],[0,0,1]],
  "K_right": [[600,0,450],[0,600,300],[0,0,1]],
  "R": [[1,0,0],[0,1,0],[0,0,1]],
  "t": [-0.4,0,0],
  "units": "synthetic_unit",
  "image_size_left": [900,600],
  "image_size_right": [900,600]
}
```

필수 항목은 `K_left`, `K_right`, `R`, `t`입니다. `units`(문자열),
`image_size_*`(양의 정수 [width,height])는 선택 항목입니다. 이미지 크기를
명시하면 실제 입력과 비교합니다. K는 **보정된 이미지의 현재 해상도**에 맞아야 합니다.

- 세계 좌표계 = 왼쪽 카메라 좌표계: +X 오른쪽, +Y 아래, +Z 전방.
- `X_right = R @ X_left + t`. t는 카메라 중심이 아닙니다.
- 오른쪽 카메라 중심은 `C_right = -R.T @ t`, baseline은 `||t||`입니다.
- XYZ와 깊이·거리 한계는 t와 같은 단위입니다. 알려진 baseline 없이는 절대 길이를 알 수 없습니다.
- NumPy는 점을 행에 저장합니다. 수식의 열벡터 연산은 batch에서 transpose합니다.
- 이미지 좌표 `(u,v)=(열,행)`, 픽셀 접근은 `image[v,u]`입니다.

## Architecture 및 파일 책임

```text
Image files → image_io → features → matched N×2 pixels
  → epipolar + geometry → F, constraints, inlier correspondences
  → camera → E, P_left, P_right
  → triangulation → homogeneous XYZ/W → Euclidean XYZ
  → filtering + point_cloud + reprojection → valid XYZ, reprojection errors
  → point_cloud.assign_colors → N×6 XYZRGB
  → exporter → JSON, PLY, optional CSV
```

| 파일 | 입력 → 출력 / 책임 |
|---|---|
| `main.py` | CLI 입력과 옵션 → 단계 실행, 통계, 결과 파일 |
| `image_io.py` | 한글 경로 이미지 ↔ BGR uint8 이미지 |
| `features.py` | ORB 검출/기술자 → Hamming 매칭 → N×2 대응점 |
| `geometry.py` | `to_homogeneous`, `from_homogeneous`: 임의 차원 동차 변환 |
| `epipolar.py` | 대응점 → RANSAC F, 직접 계산한 제약·직선·픽셀 거리 |
| `camera.py` | K/F/R/t → E 및 P; calibration 유효성 검사 |
| `triangulation.py` | 대응점·P → 직접 DLT, XYZ와 numerical mask |
| `validation.py` | 선택적으로 OpenCV 삼각측량 결과와 비교 |
| `reprojection.py` | P·XYZ → pixel 좌표, 관측점과의 L2 오차 |
| `point_cloud.py` | 양쪽 depth/거리 검증, 이미지 범위 검사, RGB 색상 할당 |
| `filtering.py` | 수치·깊이·거리·재투영 검사 통합, 제거 사유·마스크 유지 |
| `exporter.py` | N×6 XYZRGB → JSON/ASCII PLY/CSV |
| `visualization.py` | 특징점·매칭·직선·재투영 그림 |
| `generate_demo.py` | 다양한 disparity의 텍스처 패치 합성 입력 생성 |
| `tests/` | Phase 1–6 수학·예외·저장 및 연결 검증 |

## 직접 구현 및 OpenCV 경계

OpenCV 사용: 이미지 입출력, ORB 검출·기술자, BFMatcher/Hamming,
허용된 `findFundamentalMat` RANSAC, 시각화용 그리기.

직접 구현: 동차 좌표, F rank-2 정규화, 에피폴라 제약·직선·거리,
E/P 구성, DLT 행렬/SVD 해석, XYZ 정규화, 양쪽 카메라 depth/거리,
재투영, L2 오차, RGB 샘플링 및 내보내기.

`cv2.triangulatePoints`는 `validation.py`에서 비교 옵션으로만 호출합니다.
`recoverPose`, `computeCorrespondEpilines`, `projectPoints`는 핵심 코드에서
사용하지 않습니다. 테스트는 재투영·기본 DLT 경로가 고수준 OpenCV 함수를
호출하지 않는 것도 확인합니다. NumPy의 기본 행렬 연산과 SVD는 사용합니다.

## 핵심 수식 및 함수

1. `to_homogeneous(points)`: `(u,v)→(u,v,1)`, `(X,Y,Z)→(X,Y,Z,1)`.
   `from_homogeneous`는 마지막 성분으로 나눕니다. 상대적 W≈0,
   NaN/Inf, 영벡터는 NaN 행입니다. `(0,D)` 빈 batch도 지원합니다.
2. `epipolar_constraint_error`, `compute_epipolar_errors`:
   `e=x_right.T @ F @ x_left`. F는 배율이 임의이므로 e는 픽셀 거리가 아닙니다.
   `l_right=F x_left`, `l_left=F.T x_right`,
   `distance=|a*u+b*v+c|/sqrt(a²+b²)`. 양쪽 거리와 RANSAC mask로 필터합니다.
3. `compute_essential_matrix`: `E_raw=K_right.T @ F @ K_left`.
   이상적 E는 `[t]_x R`입니다. `enforce_essential_constraints`는 SVD 특이값을
   `(s,s,0)`으로 보정합니다. raw/보정 E/pose의 E를 구분해서 저장합니다.
4. `build_projection_matrix`: `P_left=K_left[I|0]`, `P_right=K_right[R|t]`.
   E와 calibration의 부호·배율 정규화 불일치도도 기록합니다.
5. `triangulate_point_dlt`, `triangulate_points_dlt`:
   A의 네 행은 `u1*P1[2]-P1[0]`, `v1*P1[2]-P1[1]`,
   `u2*P2[2]-P2[0]`, `v2*P2[2]-P2[1]`입니다.
   `A X_h=0`의 최소 제곱 해는 SVD의 `Vt[-1]`입니다. W로 나누어 XYZ를 얻습니다.
   rank 부족/무한원점 등은 NaN 행으로 유지합니다.
6. `validate_3d_points`: 왼쪽 Z와 `(R X+t)`의 오른쪽 Z 모두 양의 깊이 범위인지,
   양쪽 카메라로부터의 유클리드 거리가 한계 이내인지 검사합니다.
7. `project_point(P, point_3d)`, `project_points(P, points_3d)`:
   `x_h=P[X,Y,Z,1].T`, `pixel=x_h[:2]/x_h[2]`. 무효 투영은 NaN입니다.
8. `compute_reprojection_error(observed, projected)`:
   `sqrt((u-u_hat)²+(v-v_hat)²)`, 단일 입력은 float, N×2 입력은 N개 오차입니다.
   무효점의 오차는 inf. 좌우 각각 threshold 이내여야 통과합니다.
   평균 오차 `(left+right)/2`는 진단용이며 한쪽 큰 오차를 가리지 않습니다.
9. `assign_colors(points_3d, image, image_points)`:
   왼쪽 **관측 좌표**에서 최근접 픽셀 `floor(coord+0.5)`을 선택합니다.
   이미지 끝의 소수 좌표는 마지막 픽셀로 한정합니다. 범위 밖 입력은 오류입니다.
   `BGR[::-1]→RGB`, 최종 N×6 배열은 float64지만 RGB 값은 정수 0..255입니다.

## 옵션과 Phase 5 필터 정책

| 옵션 | 기본값 | 의미 |
|---|---:|---|
| `--nfeatures` | 4000 | ORB 특징점 목표 개수 |
| `--ratio` | 0.75 | 최근접/차선 거리 ratio 검사 |
| `--max-distance` | 64 | 특징점 Hamming bit 거리 한계 |
| `--ransac-threshold` | 1.5 | F RANSAC 픽셀 오차 한계 |
| `--epipolar-threshold` | 2.0 | 각 이미지의 점–직선 거리(px) |
| `--min-depth` | 0.000001 | 양쪽 depth가 **초과**해야 할 값 |
| `--max-depth` | 1000 | 양쪽 depth의 최대값(포함) |
| `--max-3d-distance` | 1000 | 양쪽 카메라로부터 최대 거리(포함) |
| `--reprojection-threshold` | 2.0 | 좌우 각각의 최대 재투영 오차(px, 포함) |
| `--seed` | 42 | RANSAC 재현용 seed |

깊이/3D 거리의 단위는 calibration t와 같습니다. 실제 장면에 맞게 설정하세요.
`--max-distance`는 특징점 매칭 거리이며 `--max-3d-distance`와 다릅니다.

제거 사유는 순차적으로 집계합니다: `numerical → depth → distance → image_bounds → reprojection`.
따라서 각 사유의 합이 제거된 전체 점 수와 같습니다. 원소별 mask/오차는 입력 순서로
저장하고, 최종 통과점만 색상을 할당합니다. 모든 점이 제거되면 콘솔에 경고하고,
평균 오차는 JSON null, 최종 cloud는 `(0,6)`, JSON points는 `[]`, PLY vertex는 0입니다.
빈 결과는 복원 성공의 증거가 아니므로 calibration·이미지 순서·필터 값을 확인하세요.

## 출력 파일

| 파일 | 내용 |
|---|---|
| `01_detected_features.jpg` | 양쪽 ORB 특징점 |
| `02_raw_matches.jpg` | raw match 중 상위 최대 100개 |
| `03_filtered_matches.jpg` | 특징점 필터 후 최대 100개; F 검사 전 |
| `04_epipolar_geometry.jpg` | 에피폴라 통과점·직선 최대 20개 |
| `05_reprojection.jpg` | 관측점과 재투영 비교 최대 100개 |
| `06_point_cloud.ply` | ASCII PLY 1.0; double XYZ, uchar RGB |
| `07_point_cloud.json` | schema/단위/좌표계 및 N×6 points |
| `08_point_cloud.csv` | `--csv` 옵션; 헤더 X,Y,Z,R,G,B |
| `point_cloud.npy` | 최종 N×6 float64 NumPy 배열 |
| `point_cloud_source_indices.npy` | 최종 행 → feature-filtered 대응점 원본 index |
| `geometry_data.npz` | F, 대응점, 에피폴라 오차와 마스크 |
| `camera_data.npz` | K/R/t, P_left/right, E_raw/constrained/from_pose |
| `triangulation_data.npz` | 원시 XYZ/동차 좌표, SVD, 수치 mask, 대응점·index |
| `reprojection_data.npz` | 원시 XYZ/대응점 순서의 mask, depth/거리, 좌우 투영·오차 및 최종 좌표 |
| `opencv_comparison.npz` | 비교 옵션 시 OpenCV XYZ와 행별 차이 |
| `diagnostics.json` | 단계, 특징점/매칭 수, 행렬 진단, 제거 수, 최종 평균 오차·점 수 |

재투영 그림: 초록 원=통과 관측점, 빨간 원=제거 관측점, 노란 십자=재투영,
선=두 위치 연결. 이미지 밖이나 유한하지 않은 투영 좌표는 그리지 않습니다.

`reprojection_data.npz`에서 `final_mask`는 triangulation 입력 행 기준입니다.
`accepted_source_indices`, `accepted_points_3d`, `accepted_left/right`는 최종 통과점입니다.
원본 index는 `geometry_data.npz`의 `points_left/right`(feature-filtered 대응점)으로 연결됩니다.
NPZ 진단값에는 NaN/Inf가 있을 수 있지만 최종 cloud 및 JSON/PLY에는 들어가지 않습니다.

## 렌더러 팀원 연결

```python
import numpy as np
cloud = np.load('results_phase6/point_cloud.npy')  # N×6
positions = cloud[:, :3]
colors = cloud[:, 3:] / 255.0
```

JSON은 최상위 object이고 `points`가 N×6 행 배열입니다:

```json
{
  "schema_version": 1,
  "columns": ["X", "Y", "Z", "R", "G", "B"],
  "count": 1,
  "coordinate_system": "left_camera: +X right, +Y down, +Z forward",
  "units": "m",
  "color_encoding": "RGB uint8 (0..255)",
  "points": [[1.23, 0.52, 4.21, 120, 84, 51]]
}
```

```javascript
const data = await fetch('07_point_cloud.json').then(r => r.json());
const positions = new Float32Array(data.points.flatMap(row => row.slice(0, 3)));
const colors = new Float32Array(data.points.flatMap(row => row.slice(3).map(v => v / 255)));
```

출력은 왼쪽 카메라 좌표계입니다. WebGL 카메라 관례가 +Y 위, -Z 전방이면
렌더러에서 적절한 좌표 변환을 적용하세요. 색상에 별도 gamma 변환은 하지 않습니다.
JSON의 RGB는 integer, XYZ는 number입니다. PLY는 17자리로 double 정밀도를 유지합니다.

## 단계별 검증 기록

Phase 1 검증 후 2, Phase 3 검증 후 4, **Phase 5 테스트·통합 실행 통과 후 6**을
구현했습니다. Phase 1–6과 저장 연결을 포함한 전체 **22개 테스트**가 통과했습니다.

- Phase 1: 합성 이동 (-12,0) 매칭, 특징점 없음, 한글 경로·없는 파일.
- Phase 2: 동차 변환/무한원점, 알려진 F, 3px 직선 거리, scale 불변성, outlier RANSAC.
- Phase 3: E 관계, 보정된 특이값, 알려진 projection 좌표, calibration/R/t 검사.
- Phase 4: 회전된 카메라·서로 다른 K로 알려진 3D 100개 복원(atol=1e-10),
  noisy OpenCV 비교, baseline scale, 무한원점/NaN/빈 입력/퇴화 카메라.
- Phase 5: 직접 재투영, 정확한 3–4–5 픽셀 오차, 오른쪽 회전 카메라의 depth,
  단계별 제거 집계, 한쪽만 큰 오차, threshold 경계, 빈/전부 제거 결과.
- Phase 6: BGR→RGB, 최근접·경계 픽셀, JSON/PLY/CSV 역읽기,
  float64 정밀도·RGB 정수값, 잘못된 cloud 거부, 필터 후 색상·index 정렬 유지.

합성 데모 결과: DLT 1,921개 → depth 조건으로 2개 제거 → **최종 1,919개**.
평균 재투영 오차: 왼쪽 `0.00415219px`, 오른쪽 `0.00415212px`.
DLT의 OpenCV 비교 최대 차이 `9.77e-12`(합성 단위).

실행 결과는 `demo_results_phase5/`, `demo_results_phase6/`에 포함됩니다.
`verification_phase56.txt`에는 전체 테스트, Phase 5/6 CLI 실행,
데모 JSON·PLY·NumPy 좌표/색상 및 필터 조건 교차 검증 로그를 보관합니다.
이전 Phase 3/4 로그는 `verification_phase34.txt`에 있습니다.

## 한계

추가된 공개 OpenCV 실촬영 체커보드 샘플로 보정·왜곡 보정·복원 연결을 검증했습니다.
사용자의 실제 촬영 장면과 카메라 품질은 아직 검증하지 않았습니다.
합성 데모는 여러 깊이의 정면 텍스처 패치이며 dense surface reconstruction이 아닙니다.
기본 입력은 왜곡 보정된 pinhole 이미지입니다. 반복 패턴, 움직이는 물체, 겹침 부족,
한 평면만 있는 장면, 순수 회전은 불안정한 F/3D의 원인이 됩니다. 현재 필터가
모든 잘못된 correspondence를 제거한다고 보장하지 않습니다. E pose 불일치 경고,
재투영 오차와 결과 이미지를 함께 확인해야 합니다. SIFT로 교체할 때는 검출기뿐 아니라
matcher도 Hamming에서 L2로 변경해야 합니다.
