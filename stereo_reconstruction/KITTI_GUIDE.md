# A 역할: KITTI → 직접 SGM → 포즈 누적 → voxel → WebSocket

이 모듈은 실제 KITTI 스테레오와 제공된 포즈를 이용해 점을 누적하고, B에게 **새 voxel의 대표점만**
기존 STP1 바이너리로 보낸다. HTML/React/Three.js 렌더러는 포함하지 않는다.

## 구현 범위

| 단계 | 구현 |
|---|---|
| KITTI Raw 로딩 | synced+rectified color image_02/03, calibration, OXTS, timestamps 직접 파싱 |
| KITTI Odometry 로딩 | sequences/XX/image_2,image_3, calib.txt, times.txt, poses/XX.txt 지원 |
| 깊이 | 기존 NumPy 직접 구현 Census·경계 가중3×3·8방향 SGM 유지 |
| 정렬·3D | 픽셀 중심 규약을 반영한 축소 K, 직접 rectification 행렬, 기존 직접 DLT |
| 포즈 | Raw GPS/IMU Mercator+Euler, 또는 Odometry 제공 pose 적용 |
| 누적·다운샘플링 | 직접 구현 world-space voxel hash, 대표 관측 선택, append-only ID |
| 전송 | 실제 WebSocket live/replay 서버, STP1 binary, chunk별 ACK/backpressure |
| B 검증 | 기존 JS decoder로 실제 수신, ID·개수·패킷·SHA256 확인 |

OpenCV SGBM/BM, pykitti, Open3D, 외부 SLAM을 호출하지 않는다.
NumPy는 배열/SVD, OpenCV는 이미지 입출력·색변환·resize/remap에 사용한다.
네트워크 RFC6455 transport만 `websockets==13.1`을 사용한다.

## 1. 환경 설치

프로젝트 폴더에서 준비한 Python 환경을 활성화한 뒤:

```bash
python -m pip install -r requirements.txt -r requirements-streaming.txt
```

Python3.8 기존 환경 및 Python3.11 검증 환경을 지원한다. JS 검증 client는 native WebSocket을 제공하는
Node22 이상이 필요하다. B 앱에서 쓰는 `stream_client/read_points.mjs` 자체는 외부 npm 의존성이 없다.
시각화는 Python3.11 환경에서 `requirements-visualization.txt`를 추가 설치한다.

## 2. 공개 KITTI 실제 샘플 준비

```bash
python download_kitti_sample.py --output kitti_data
```

공개 KITTI Raw `2011_09_26_drive_0001_sync`의 frame0,5,10,15,20을 사용한다.
원본 ZIP 약458.6MB 전체 대신 HTTP Range로 해당 stereo10장, OXTS, timestamp, calibration,
frame0 LiDAR만 내려받는다. 검증 실행의 ZIP range 수신량은10,433,174바이트였다.
다운로드 파일은 ZIP CRC와 SHA256으로 확인하며 URL/ETag/hash는 `download_manifest.json`에 남긴다.
공개 객체가 접근 거부되거나 Range를 지원하지 않으면 실패한다. private URL/login을 우회하지 않는다.

전체 Raw 데이터를 가지고 있다면 다음 구조의 root를 사용한다.

```text
kitti_data/
  2011_09_26/
    calib_cam_to_cam.txt
    calib_velo_to_cam.txt
    calib_imu_to_velo.txt
    2011_09_26_drive_0001_sync/
      image_02/data/0000000000.png
      image_03/data/0000000000.png
      image_02/timestamps.txt
      image_03/timestamps.txt
      oxts/data/0000000000.txt
      ...
```

unsynced/unrectified Raw 이미지를 넣지 않는다. frame0 OXTS는 기준 좌표계 구성에 필요하다.

## 3. 파일 출력만 실행

```bash
python kitti_pipeline.py \
  --root kitti_data --format raw --date 2011_09_26 --drive 0001 \
  --frames 0 5 10 15 20 --scale 0.5 --max-disparity 96 \
  --min-depth 2 --max-depth 40 --voxel 0.15 \
  --output kitti_run
```

`--output`은 비어 있는 새 폴더여야 한다. 이전 실행과 섞지 않도록 덮어쓰지 않는다.
기본 map capacity는2,000,000점이며 `--max-points`로 조절한다. 초과하면 조용히 점을 버리지 않고
현재 frame을 map에 반영하기 전에 실패한다. 더 큰 voxel 또는 적은 frame으로 재실행한다.

Odometry color 데이터는 다음과 같이 실행한다.

```bash
python kitti_pipeline.py --format odometry --root KITTI_ODOMETRY \
  --sequence 00 --poses KITTI_ODOMETRY/poses/00.txt \
  --frames 0 5 10 15 20 --output odometry_run
```

`KITTI_ODOMETRY/sequences/00/{image_2,image_3,calib.txt,times.txt}`가 필요하다.
grayscale image_0/1이나 pose 없는 test sequence는 현재 지원 대상이 아니다.
실제 데이터 end-to-end 검증은 Raw로 수행했고, Odometry의 pose 방향·camera0→camera2 lever arm은
알려진 합성 calibration/pose로 단위 검증했다. 실제 Odometry sequence 검증을 했다고 주장하지 않는다.

## 4. 실제 live WebSocket 실행

터미널 A:

```bash
python stream_server.py live \
  --root kitti_data --frames 0 5 10 15 20 \
  --output kitti_live --host 127.0.0.1 --port 8765
```

터미널 B:

```bash
node stream_client/receive_stream.mjs ws://127.0.0.1:8765 received
```

client가 `client_ready`를 보내면 실제 SGM 계산을 시작하며 각 frame 처리 직후 신규점이 전송된다.
기본 서버는 연결을 계속 받는다. `--once`를 붙이면 한 연결의 처리가 끝난 뒤 종료한다.
수신기에는 frame별 point count, 원본uv/d/confidence, global ID, world 좌표가 전달된다.
완료 시 `received/received_points.bin`이 서버의 `map_points.bin`과 완전히 같아야 한다.

이미 계산한 결과를 B에게 빠르게 재생:

```bash
python stream_server.py replay --run kitti_live --host 127.0.0.1 --port 8765
```

replay는 SGM을 재계산하지 않고 같은 신규점 bytes를 보낸다. recorded timestamp대로 지연시키는 재생은 아니다.
C가 재생 속도를 제어하려면 timestamp를 읽어 표시 시점을 조절한다.
기본은 localhost 전용이며 인증/암호화가 없는 개발용 서버다. 별도 요청 없이 외부 주소에 바인딩하지 않는다.

## 포즈와 좌표의 정확한 의미

모든 길이는 **metre**. World는 KITTI **frame0의 원래 rectified 왼쪽 color camera2 좌표계**다.
프레임을10부터 선택해도 기준은0이다. +X 오른쪽, +Y 아래, +Z 앞쪽이다.

Raw에서는 직접 계산한 `Mercator(lat,lon,alt)`와 `Rz(yaw)Ry(pitch)Rx(roll)`로 GPS/IMU pose를 만든다.
IMU→Velodyne→unrectified camera0→rectified camera0→camera2 보정 체인을 적용한다.
camera projection의 마지막 열은 `t=K^-1 P[:,3]` 전체를 사용하므로 작은 Y/Z offset도 보존한다.
좌표는 먼저 frame0 GPS 위치를 빼 큰 좌표의 부동소수점 오차를 줄인다.

```text
T_ENU_from_camera(t) = T_ENU_from_IMU(t) @ inverse(T_camera_from_IMU)
T_world_from_camera(t) = inverse(T_ENU_from_camera(0)) @ T_ENU_from_camera(t)
```

이는 **제공된 포즈의 적용**이다. 카메라 포즈를 영상에서 새로 추정하는 SLAM/visual odometry를 구현한 것은 아니다.
Odometry의 제공 pose는 camera0 기준이므로 camera2 extrinsic을 반영한 뒤 frame0 camera2 기준으로 정규화한다.

축소된 KITTI 영상을 기존 직접 rectification으로 한 번 더 수평 정렬해 작은 수직 baseline 성분을 처리한다.
DLT 좌표는 이 추가 정렬 카메라 기준이며, Q를 원래 camera2로 되돌린 후 world로 옮긴다.

```text
T_world_from_observation_camera = T_world_from_camera2 @ [Q_left.T, 0; 0,1]
p_world = T_world_from_observation_camera @ p_DLT
```

frame metadata의 observation camera는 **축소+추가 정렬한 왼쪽 카메라**다.
uv/시차는 `left_rectified.png`, `right_rectified.png`와 같은621×188 좌표계다.
원본1242×375 픽셀과 혼동하지 않는다. `K_rect`와 baseline도 함께 전달한다.

## 다운샘플링과 신규점 정책

`floor(world_xyz / voxel_size)`를 key로 사용하는 world voxel hash를 직접 구현했다.
wire float32 위치로 먼저 변환한 뒤 key를 계산하므로 B가 voxel을 재구성해도 동일하다.

- 한 frame에서 같은 voxel이면 confidence가 가장 높은 **실제 관측점**을 선택한다.
- 같은 confidence면 원래 입력 순서가 빠른 점을 선택한다.
- 과거 frame에서 이미 채워진 voxel은 추가 전송하지 않는다.
- centroid/색 평균을 만들지 않아 대표점의 uv·시차·신뢰도·색·포즈 대응이 유지된다.
- ID는 stream 안에서0부터 단조 증가한다. 이미 전송한 점은 이동/교체하지 않는다.

이는 append-only 대표점 map이다. 뒤에서 더 좋은 관측이 와도 기존 voxel 대표를 교체하지 않는다.
가중 융합/지도 갱신/동적 물체 제거는 포함하지 않는다. 차량·보행자 이동과 깊이 잡음 때문에
서로 다른 voxel에 흔적이 남을 수 있으며 voxel 중복 제거가 동일 물체 tracking을 뜻하지 않는다.

## 출력 파일

```text
kitti_live/
  config.json, summary.json, server_sessions.json
  map_points.bin          # 최종 누적 STP1 레코드만, header 없음
  map_points_m.ply        # world XYZRGB, metre
  frames/0000000000/
    frame.json            # pose·K·timestamp·증분 개수·처리 시간
    new_points.bin        # 이 frame에서만 추가된 점
    chunk_00000.stp ...   # 실제 전송한 header+payload packet
    observations.npz      # 깊이 유효 관측 전체와 selected_indices
    left_rectified.png, right_rectified.png, valid_mask.png
    lidar_check.json      # scan이 있을 때만, 검증 전용
  ...
```

`observations.npz`의 `selected_indices`는 이 frame의 전체 depth-valid 관측 중 전송한 점의 행이다.
`point_id`는 선택된 점에 대해서만 존재하며 global ID다. 따라서 모든 배열 길이가 같다고 가정하지 않는다.
`camera_xyz_m/pixel_left/disparity_px/confidence/rgb`에는 동일한 selected_indices를 적용해 전송점과 연결한다.
world pose는4×4 row-major 중첩 배열이며 column vector에 곱한다. flat column-major도 frame.json에 제공한다.

전송 레이아웃과 ACK 규약은 [STREAM_PROTOCOL.md](STREAM_PROTOCOL.md)를 따른다.

## 검증 실행

```bash
python -m unittest discover -s tests -v
python verify_kitti_run.py --run kitti_live --received received
python visualize_kitti.py --run kitti_live
```

실제 Raw 검증 frame0/5/10/15/20:

| frame | depth-valid 관측 | frame 내 voxel 수 | 전송 신규점 | 누적점 |
|---:|---:|---:|---:|---:|
| 0 | 72,312 | 39,983 | 39,983 | 39,983 |
| 5 | 74,466 | 39,135 | 33,646 | 73,629 |
| 10 | 74,729 | 41,681 | 35,648 | 109,277 |
| 15 | 75,873 | 44,260 | 37,449 | 146,726 |
| 20 | 73,147 | 41,455 | 33,826 | 180,552 |

전체 depth-valid370,527점 → 최종180,552점, 47개 packet. 수신 payload는5,777,664바이트.
5프레임은 촬영 시간 약2.062초, 카메라 진행 약25.8m를 포함한다.
실제 Python3.11 검증에서 frame 처리시간 합계 약55.9초(SGM 외 DLT·저장 포함),
서버 전체 경과시간 약83.5초(client 대기 등 포함)였다. **10Hz 실시간 처리 달성을 주장하지 않는다.**
빠른 렌더링 통합에는 저장 결과 replay를 사용하고, live 계산 속도 개선은 별도 과제다.

frame0의 유효 stereo/LiDAR 공통14,526pixel에서 시차 차이 중앙값0.303px, 상대 깊이 차이 중앙값2.11%였다.
LiDAR는 직접 투영/z-buffer로 비교한 독립적인 sanity check이며, dense 정답 시차가 아니다.
센서 시간차·가림·동적 물체 영향을 받는다. LiDAR를 SGM 입력·필터·voxel map 생성에 사용하지 않았다.

51개 테스트는 기존40개에 pose/voxel/packet8개, ACK/timeout/reconnect3개를 추가한 것이다.
검증 스크립트는 pose 적용/역변환, RGB/uv/d/confidence 대응, global ID, voxel 유일성,
저장 packet 연결, JS 수신 bytes의 동일성까지 확인한다.

출처:
- [KITTI Raw 데이터와 센서 규약](https://www.cvlibs.net/datasets/kitti/raw_data.php)
- [KITTI Odometry](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)
- [공식 페이지에서 소개한 pykitti의 OXTS 규약](https://github.com/utiasSTARS/pykitti/blob/master/pykitti/utils.py): 규약 확인용, 패키지/코드 호출 없음
- [websockets13.1 transport API](https://websockets.readthedocs.io/en/13.1/reference/asyncio/server.html)
