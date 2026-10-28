# 체커보드 Stereo Calibration 및 실제 이미지 전처리

기존 Phase 1–6 앞에 **체커보드 보정 → 실제 이미지 왜곡 보정** 단계를 추가했습니다.
복원 코어의 DLT와 재투영 계산은 그대로 직접 구현을 사용합니다. 이 준비 도구에서는
OpenCV의 calibrateCamera/stereoCalibrate/solvePnP 및 렌즈 왜곡 보정 기능을 사용합니다.

## 1. 공개 테스트셋으로 바로 실행

프로젝트 폴더에서 실행하세요. 이미지와 실행 결과가 이미 포함되어 있으므로
다운로드 단계는 생략할 수 있습니다. ZIP에는 가상환경이 없으므로 새 환경에서는
`python3 -m venv .venv` 후 `.venv/bin/python -m pip install -r requirements.txt`로 준비합니다.

```bash
# 선택: 공식 샘플 재다운로드 (네트워크 필요)
.venv/bin/python download_calibration_sample.py --output sample_calibration

# 11쌍으로 보정, 나머지 2쌍은 학습 없이 별도 검증
.venv/bin/python calibrate_stereo.py \
  --pairs sample_calibration/train_pairs.json \
  --validation-pairs sample_calibration/validation_pairs.json \
  --cols 9 --rows 6 --square-size 1 --units board_square \
  --output calibration_run

# 별도 검증 이미지 13번 쌍의 왜곡 보정
.venv/bin/python undistort_stereo.py \
  --left sample_calibration/left13.jpg \
  --right sample_calibration/right13.jpg \
  --model calibration_run/raw_camera_model.json \
  --output real_prepared

# 기존 복원 코어 실행
.venv/bin/python main.py \
  --left real_prepared/left.png --right real_prepared/right.png \
  --calibration real_prepared/calibration.json \
  --phase 6 --compare-opencv --output real_reconstruction

# 별도 검증쌍에서 코너 정답 대응으로 직접 DLT·재투영·칸 길이 검사
.venv/bin/python verify_calibration_sample.py \
  --model calibration_run/raw_camera_model.json \
  --pairs sample_calibration/validation_pairs.json \
  --output calibration_run/held_out_geometry.json
```

공개 샘플의 실제 체커보드 칸 길이는 확인되지 않았으므로 `square-size=1`,
`units=board_square`입니다. **결과를 미터로 해석하지 마세요.**
샘플 보정값은 공개 샘플을 찍은 카메라에만 적용되며 사용자의 카메라에 재사용할 수 없습니다.

## 2. 실제 보정 사진 준비

- 카메라 두 대는 상대 위치를 단단히 고정합니다. 카메라 한 대라면 슬라이더/고정 장치로
  항상 같은 좌우 위치를 재현해야 합니다.
- 보정과 장면 촬영에 같은 렌즈, 초점, 해상도, 크롭/줌 설정을 유지합니다.
- 평평한 판 위의 체커보드를 사용하고 칸 길이를 직접 측정합니다.
- 보드의 위치·기울기·거리를 다양하게 바꿔 20~30쌍 정도 확보하는 것을 권합니다.
- 한 쌍을 촬영하는 동안 보드를 움직이지 않습니다. 두 이미지의 같은 코너가 대응해야 합니다.
- 입력 `--cols`, `--rows`는 사각형 수가 아니라 **내부 교차점 개수**입니다.
  예를 들어 10×7개 사각형 보드의 내부 코너는 9×6입니다.
- 해상도는 모든 좌우 이미지에서 같아야 합니다. 현재 도구는 fisheye 모델을 지원하지 않습니다.

파일명이 양쪽 동일하면 폴더 입력을 사용합니다:

```text
my_calibration/
  left/001.png, 002.png, ...
  right/001.png, 002.png, ...
```

```bash
.venv/bin/python calibrate_stereo.py \
  --left-dir my_calibration/left --right-dir my_calibration/right \
  --cols 9 --rows 6 --square-size 0.025 --units m \
  --output my_camera
```

이 명령은 **칸 크기 25mm를 0.025m로 입력**하는 예시입니다. millimeter를 사용하려면
`--square-size 25 --units mm`로 지정하고, 복원 시 depth/거리 한계도 mm로 설정해야 합니다.
`--units` 문자열만 바꾸어도 좌표가 자동 환산되는 것은 아닙니다.

파일명이 다르면 JSON으로 좌우를 명시합니다:

```json
[
  {"left": "left/frame001.jpg", "right": "right/frame001.jpg"},
  {"left": "left/frame002.jpg", "right": "right/frame002.jpg"}
]
```

각 경로는 JSON 파일이 있는 폴더 기준입니다. 위는 형식 예시이며 기본 실행에는
**검출에 성공한 최소 8쌍**이 필요합니다. `--min-pairs`로 바꿀 수 있지만 3 미만은 허용하지 않습니다.
좌우 폴더 입력은 파일명이 정확히 일치해야 하며 정렬 순서만으로 다른 이름을 짝짓지 않습니다.
이미지 누락·중복 파일, 다른 해상도, 학습/검증 경로 중복은 오류입니다.

별도 검증이 필요하면 보정에 사용하지 않은 쌍을 `--validation-pairs validation_pairs.json`으로
지정하세요. 해당 사진은 학습 폴더/학습 manifest에 포함하지 않아야 합니다.

## 3. 보정 도구가 수행하는 계산

1. 보드의 각 코너를 `(column*square_size, row*square_size, 0)`으로 구성합니다.
2. `findChessboardCorners`와 `cornerSubPix`로 양쪽 픽셀 좌표를 찾습니다.
3. 검출 실패 쌍은 건너뛰고 이유를 기록합니다. 모든 성공 쌍의 코너 그림을 저장합니다.
4. `calibrateCamera`로 각 카메라의 K와 5개 왜곡 계수 D를 추정합니다.
5. `stereoCalibrate(..., CALIB_FIX_INTRINSIC)`로 고정한 K/D 아래 상대 R/t를 추정합니다.
6. training/validation 쌍마다 왼쪽 보드 pose만 `solvePnP`로 추정하고,
   `R_right_board = R @ R_left_board`, `t_right_board = R @ t_left_board + t`로
   오른쪽 코너 위치를 예측합니다. validation에서는 K/D/R/t를 다시 학습하지 않습니다.
7. 진단용 왜곡 투영은 5계수 Brown–Conrady 수식을 NumPy로 직접 계산합니다.

좌표 약속은 기존 코어와 같습니다: **X_right = R X_left + t**.
오른쪽 카메라 중심은 `-R.T@t`이고 baseline은 `||t||`입니다.
K의 초점 거리는 픽셀 단위이며, t와 출력 XYZ의 단위는 square-size와 같습니다.
체커보드 코너 순서는 좌우에서 같은 물리적 코너여야 합니다.
`training_corners/`의 빨간 `0` 표시를 확인하세요. 보드 대칭에 의한 순서 오류를
자동으로 모두 해결하지 않습니다. 심한 pair 오차가 있으면 사진/순서를 확인해야 합니다.

## 4. 생성 파일과 용도

| 파일 | 용도 |
|---|---|
| `raw_camera_model.json` | 원본 이미지용 K_left/right, D_left/right, R, t, 단위·크기·보드 설정 |
| `calibration.json` | **왜곡 보정된 이미지용** K_left/right, R, t, 단위·크기 |
| `calibration_report.json` | 검출 성공·실패, 학습쌍, RMS, pair별 오른쪽 예측 오차, baseline, 경고 |
| `detected_corners.npz` | 학습에 사용한 실제 2D 코너와 보드 좌표 |
| `training_corners/` | 검출 결과 그림. 왼쪽 이미지와 오른쪽 이미지를 나란히 표시 |
| `validation_corners/` | 별도 검증쌍의 검출 그림 |

`raw_camera_model.json`은 **undistort_stereo.py의 --model 입력**입니다.
`calibration.json`은 **main.py의 --calibration 입력**입니다. 두 파일을 바꾸어 쓰면 안 됩니다.
기존 loader도 image_state=raw인 모델을 복원용으로 넣으면 설명과 함께 거부하도록 보완했습니다.
기존 수동 calibration.json이나 demo/calibration.json은 계속 사용할 수 있습니다.

## 5. 실제 장면 이미지 두 장의 왜곡 보정

보정 완료 후 카메라 배치/설정을 바꾸지 말고 장면을 촬영하세요.

```bash
.venv/bin/python undistort_stereo.py \
  --left my_scene/left.jpg --right my_scene/right.jpg \
  --model my_camera/raw_camera_model.json \
  --output my_scene_prepared
```

출력:

- `left.png`, `right.png`: 왜곡 보정된 PNG
- `calibration.json`: 이 출력 이미지에 맞는 복원 입력
- `left_before_after.jpg`, `right_before_after.jpg`: 원본(왼쪽)/보정(오른쪽) 비교
- `valid_left.png`, `valid_right.png`: 원본 이미지에서 샘플링 가능한 영역(흰색)
- `undistortion_report.json`: 원본 경로, 처리 방식, 유효 영역 비율

이 스크립트는 **K와 이미지 크기를 유지**합니다. crop/resize/stereo rectification은 하지 않으므로
R/t도 그대로입니다. 출력 가장자리의 검은 영역은 원본 범위를 벗어난 remap 영역일 수 있습니다.
유효 영역 마스크는 확인용으로 저장하며 현재 복원 CLI가 자동으로 읽지는 않습니다.
이 보정은 역매핑+선형 보간으로 수행합니다. 이미 보정된 이미지에 다시 적용하지 마세요.
도구는 undistorted 모델의 재사용을 거부하지만, raw 모델과 이미 보정된 이미지를 섞는 실수까지
픽셀만으로 자동 판별할 수는 없습니다.
입력과 출력 파일이 겹치면 거부합니다. 해상도가 다르면 자동으로 K를 추측하지 않고 오류를 냅니다.

## 6. 기존 Phase 6 실행

미터 단위, 5cm~10m 범위의 예시입니다:

```bash
.venv/bin/python main.py \
  --left my_scene_prepared/left.png --right my_scene_prepared/right.png \
  --calibration my_scene_prepared/calibration.json \
  --phase 6 --min-depth 0.05 --max-depth 10 --max-3d-distance 10 \
  --reprojection-threshold 2 --compare-opencv --csv \
  --output my_scene_result
```

`05_reprojection.jpg`, `diagnostics.json`, `06_point_cloud.ply`, `07_point_cloud.json`을 확인하세요.
장면에는 서로 다른 깊이의 텍스처가 있는 물체가 좋습니다. 반복 체커 패턴은 ORB 매칭에
모호성이 있으므로 코너 정답 대응의 보정 검증과 ORB 장면 복원은 구분해야 합니다.

## 7. 테스트셋 출처와 검증 결과

- 출처: OpenCV 공식 저장소, 버전 4.11.0, `samples/data/left*.jpg`, `right*.jpg`
- 공식 목록: https://github.com/opencv/opencv/blob/4.11.0/samples/data/stereo_calib.xml
- 공식 예제: https://github.com/opencv/opencv/blob/4.11.0/samples/cpp/stereo_calib.cpp
- 다운로드된 원본 출처 URL/SHA-256은 `sample_calibration/provenance.json`에 기록했습니다.
- 저장소 LICENSE 사본은 `sample_calibration/OPENCV_LICENSE.txt`에 포함했습니다.
- 총 13쌍(640×480, 내부 코너 9×6). 01–09,11,12의 11쌍으로 학습하고 13,14는 별도 검증합니다.
- 실제 칸 크기가 확인되지 않았으므로 **board_square 단위**이며 실제 baseline의 metric 정답 검증은 아닙니다.

검증 환경: Python 3.8.18, NumPy 1.24.4, OpenCV 4.11.0.

| 검사 | 측정 결과 |
|---|---:|
| 학습 검출 성공 | 11/11쌍 |
| 왼쪽 intrinsic RMS | 0.418375px |
| 오른쪽 intrinsic RMS | 0.468344px |
| Stereo RMS | 0.458297px |
| 학습에서 제외한 2쌍의 오른쪽 예측 RMS | 0.422016px |
| baseline | 3.344355 board_square |
| 별도 검증 13번 코너 DLT: 칸 길이 중앙값 | 1.001025칸 |
| 별도 검증 14번 코너 DLT: 칸 길이 중앙값 | 0.999555칸 |
| 코너 DLT 평균 재투영 오차 | 약 0.039~0.054px |
| 실제 remap 후 코너 위치와 예상 보정 위치의 중앙 차이 | 약 0.026px |
| 13번 이미지의 ORB→Phase 6 결과 | 193개 XYZRGB |
| ORB 결과 평균 좌/우 재투영 오차 | 0.32995 / 0.32637px |

별도 코너 검증은 자체 DLT·재투영 함수를 호출합니다. 이는 calibration과
복원 코어의 좌표 약속이 맞는지 확인하는 검사입니다. ORB의 모든 대응점이 올바르거나
다른 실제 카메라에서도 같은 품질이 나온다는 의미는 아닙니다.
공개 샘플 검사 기준(각 쌍): 칸 길이 중앙값 0.9~1.1칸, 변 길이 상대 RMSE<0.15,
평면 거리 RMS<0.1칸, 평균 재투영<1px, 보정 코너 위치 중앙 차이<1px입니다.

```bash
.venv/bin/python -m unittest discover -s tests -v
```

기존 22개와 신규 4개를 합쳐 **26개 테스트**가 통과했습니다. 신규 테스트에는
알려진 K/D/R/t의 합성 observation으로 파라미터 복원, manifest 짝짓기/중복/검증 누출,
무왜곡 영상의 보존, K/R/t 유지, 크기 불일치, 잘못된 모델/입력 겹침,
코너 검출 실패 시 진단 기록을 포함합니다.

재현 가능한 실행 로그는 `verification_calibration.txt`에 포함됩니다.
이미 생성된 예제 결과는 `sample_calibration_result/`, `sample_undistorted/`,
`sample_reconstruction/`에서 확인할 수 있습니다. 보고서의 절대 경로는 검증 당시 경로입니다.

## 8. 문제 해결

- 검출 성공이 적음: 사각형 수와 내부 코너 수 혼동, 흐림, 보드 일부 잘림을 확인합니다.
- Stereo RMS가 큼: 좌우 짝, 코너 순서, 촬영 사이 보드/카메라 이동, 잘못된 칸 크기를 점검합니다.
  칸 크기 오류는 RMS가 낮아도 복원 길이를 틀리게 만들 수 있습니다.
- 극단적인 K/D: 비슷한 정면 보드 사진만 반복하지 말고 기울기와 화면 위치를 다양하게 촬영합니다.
- 깊이가 음수: 좌우 파일 순서와 R/t 좌표 약속을 확인합니다.
- 보정 후 사진 크기가 다름: resize/crop 없는 같은 촬영 설정을 사용하거나 해당 처리에 맞는 K를 별도로 계산해야 합니다.
- 재실행: 실험마다 새 output 폴더를 사용하세요. 기존 같은 이름 파일은 갱신하지만 다른 파일은 삭제하지 않습니다.
