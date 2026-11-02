# Computer Graphics — Stereo Reconstruction & Rendering

스테레오 영상에서 복원한 점군을 웹에서 표면으로 그리고, 붓질과 페인트총으로 장면을 완성하는 컴퓨터 그래픽스 팀 프로젝트입니다.

## 팀원별 코드 안내

| 담당 | 먼저 열 파일 | 이곳에서 수정하는 기능 |
|---|---|---|
| **A · 데이터 파이프라인** | [KITTI 실행](stereo_reconstruction/kitti_pipeline.py) · [SGM](stereo_reconstruction/sgm_stereo.py) · [표면 속성](stereo_reconstruction/surface_attributes.py) · [바이너리 규약](stereo_reconstruction/STREAM_PROTOCOL.md) | 영상에서 시차·XYZ·색·법선·반경 계산, 포즈 적용, 다운샘플링, 점 전송 |
| **B · 렌더링 코어** | [splat 셰이더](renderer/splats.js) · [다중 패스·EDL](renderer/effects.js) · [반경 추정](renderer/spacing.js) | 점을 표면에 맞춰 그리기, 조명·광택, 입자 그림자, 깊이 경계 처리 |
| **C · 연출과 화면** | [입자 발사 순서](renderer/paint.js) · [컨트롤 UI](renderer/index.html) · [재생·이벤트](renderer/viewer.js) | 붓질과 페인트총의 순서·속도, 타임라인·되감기, 시연 버튼 |
| **B·C 공동** | [viewer.js](renderer/viewer.js) · [splats.js](renderer/splats.js) | viewer에서 B는 카메라·렌더 설정, C는 재생 시간과 모드 전환을 담당합니다. splats의 비행 궤적·착지 퍼짐은 C의 연출과 B의 GPU 표현이 만나는 부분입니다. |

A가 저장한 한 프레임을 B가 [디코더](stereo_reconstruction/stream_client)로 읽습니다. C가 점별 착지 시각을 정하면 B의 셰이더가 현재 재생 시각에 맞춰 점의 위치와 재질을 그립니다. [팀원용 렌더러 실행 방법](renderer/README.md)

직접 구현한 부분은 Census/8방향 SGM, 삼각측량·재투영, voxel 선택, 경계 연결 탐색과 Huber IRLS 평면 법선입니다.
NumPy를 행렬 연산에, OpenCV를 이미지 처리·ORB·카메라 보정 등에, websockets를 통신 transport에 사용합니다.
OpenCV SGBM이나 Open3D 법선 추정 함수로 핵심 계산을 대체하지 않습니다.

## 구조

```text
stereo_reconstruction/   # A 소스, 테스트, 실행/바이너리 규약 문서
  stream_client/         # B용 JavaScript 수신·디코더 예제 (HTML 의존성 없음)
  tests/                 # 단위·통합 테스트
renderer/                # B 렌더링과 C 연출·UI
validation/              # 고정 샘플 비교 수치와 테스트 기록
```

## 시작하기

Python 3.11을 기준으로 설명합니다. 기존 검증은 Python 3.8과 3.11에서 수행했습니다.
WebSocket 수신 예제는 Node.js 22 이상을 사용합니다.

```bash
git clone https://github.com/hyewonimdang/computer-graphics.git
cd computer-graphics/stereo_reconstruction
python3 -m venv .venv
source .venv/bin/activate
# Windows PowerShell: .venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt -r requirements-streaming.txt
python -m unittest discover -s tests
```

합성 데모는 다운로드 없이 생성할 수 있습니다. 제공된 demo/calibration.json은 이 데모 전용입니다.

```bash
python generate_demo.py --output demo
python main.py --left demo/left.png --right demo/right.png --calibration demo/calibration.json --phase 6 --output results_demo
```

실제 촬영한 두 장은 먼저 렌즈 왜곡을 보정하고 해당 촬영의 K/R/t를 준비합니다.

```bash
python dense_reconstruct.py --left /path/to/left.png --right /path/to/right.png --calibration /path/to/calibration.json --method sgm --output results_stereo
# 선택: 진단 그림
python -m pip install -r requirements-visualization.txt
python verify_surface.py --run results_stereo
```

물리적 단위가 없는 보정값으로 metre 반경을 추측하지 않습니다. 해당 경우 실측한 `--meters-per-unit` 값이 필요합니다.

## KITTI 샘플 및 B 연결

```bash
python download_kitti_sample.py --output data/kitti --frames 0
python kitti_pipeline.py --root data/kitti --frames 0 --output runs/kitti_frame0
python stream_server.py replay --run runs/kitti_frame0 --port 8767
# 다른 터미널, 같은 폴더에서
node stream_client/receive_stream.mjs ws://127.0.0.1:8767 runs/received
```

KITTI 포즈 적용은 선택적인 기존 시퀀스 경로입니다. 일반 스테레오 한 쌍 복원에는 GPS/IMU가 필요하지 않습니다.
KITTI의 제공 포즈를 사용하며 SLAM을 추정하지 않습니다. 현재 실시간 10Hz 처리를 달성한 구현은 아닙니다.

## 출력 규약

- **STP1:** 32-byte XYZRGB·UV·시차·신뢰도·point_id 레코드.
- **SRF1:** 같은 ID에 연결하는 32-byte 법선·metre 반경·support·유효 flags 레코드.
- KITTI 출력은 이미 world 포즈가 적용되어 있으므로 B에서 포즈를 다시 곱하지 않습니다.
- 반경은 metre 단위의 반지름이며 픽셀 크기/직경이 아닙니다. 무효 법선은 flags로 구분합니다.

[기본 실행](stereo_reconstruction/README.md) · [보정](stereo_reconstruction/CALIBRATION_GUIDE.md) ·
[SGM](stereo_reconstruction/SGM_GUIDE.md) · [KITTI](stereo_reconstruction/KITTI_GUIDE.md) ·
[STP1 통신](stereo_reconstruction/STREAM_PROTOCOL.md) · [SRF1 표면 규약](stereo_reconstruction/SURFACE_GUIDE.md) ·
[법선 v2](stereo_reconstruction/SURFACE_V2_GUIDE.md)

## 검증 요약

같은 KITTI 스테레오 복원 결과의 73,229개 XYZ를 고정한 비교:

| 항목 | 기존 차분 | 개선 법선 v2 |
|---|---:|---:|
| 유효 법선 비율 | 54.46% | 74.41% |
| 공통 이웃 법선 각도 차이 중앙값 | 30.49° | 9.15° |

실제 영상의 각도 차이는 노이즈/연속성의 대리지표이며 정답 법선 정확도는 아닙니다.
정답이 있는 합성 깊이 단차에서도 별도 검증했습니다. 상세 수치는 [comparison.json](validation/comparison.json)에 있습니다.
기존 Python 3.8/3.11 검증은 각각 61개 테스트 통과했습니다. 과거 검증 기록은 validation/에 보존합니다.

원본 영상·대용량 출력·ZIP·가상환경은 Git에 포함하지 않습니다. KITTI 데이터는 다운로드 도구로 별도 준비하며
데이터셋 사용 조건은 [KITTI 공식 사이트](https://www.cvlibs.net/datasets/kitti/raw_data.php)를 확인하세요.
프로젝트 코드의 라이선스는 아직 별도로 지정하지 않았습니다.

B의 스테레오 표면 렌더러 실행 방법은 [렌더러 안내](renderer/README.md)를 참고하세요.
