# v2 기본 계산 업데이트

기본 법선 계산이 깊이 연결 7×7 이웃의 강건한 역깊이 평면 추정으로 개선되었습니다.
직접 구현 방법·같은 샘플 비교·한계는 [SURFACE_V2_GUIDE.md](SURFACE_V2_GUIDE.md)를 참고하세요.
아래 SRF1 바이너리 규약과 단위는 그대로 유지됩니다.

# A → B: 표면 법선과 splat 반경 (SRF1 v1)

2026-10-28. 기존 STP1 XYZRGB·UV·시차·신뢰도·point_id 레코드는 그대로 유지한다.
SRF1은 point_id로 연결하는 추가 표면 데이터다. SGM/DLT/법선/반경은 직접 NumPy 구현이며,
Open3D/PCL의 법선 추정 또는 splat 생성 기능을 사용하지 않는다.
WebSocket transport는 websockets, 이미지 입출력·리사이즈는 OpenCV를 사용한다.
진단 그림은 Matplotlib와 직접 계산한 Lambert 식으로 만들며, B의 GPU 셰이더를 대신 구현한 결과는 아니다.

## v1 차분 계산 (v2의 fallback)

1. voxel 다운샘플링 **전**의 유효 3D 관측과 rectified UV로 organized grid를 만든다.
2. 상하좌우 즉시 인접한 네 픽셀만 확인한다. 영상 밖·결측값 또는
   `abs(z_neighbor-z_center) > 0.02 m + 0.03*min(z_neighbor,z_center)`이면 연결하지 않는다.
3. 좌우 모두 있으면 `du=(P_right-P_left)/2`, 한쪽이면 한 픽셀 단측 차분을 사용한다.
   dv도 같은 방식으로 계산한다. 수평/수직 차분을 모두 구할 수 있어야 한다.
4. `n=normalize(cross(du,dv))`. `dot(n,P)>0`이면 뒤집어서 원래 관측 카메라를 향하게 한다.
   외적 길이가 1e-12 이하이거나 `dot(n,-normalize(P))<0.15`인 grazing surface는 무효로 둔다.
5. 원형 splat의 **반경**은 `r=0.5*sqrt(dot(du,du)+dot(dv,dv))`이다.
   이는 국소 픽셀 footprint의 반대각 길이다. 타원/공분산 추정값이 아니다.
   네 이웃 중 하나라도 없거나 배제되면 가장 가까운 연결 이웃 거리의 절반 이하로 제한한다.
   모든 반경은 최대 0.15 m다. 무효 법선에는 반경 0을 쓴다.
6. voxel 대표점을 선택할 때 법선·반경도 동일한 원본 인덱스로 선택한다.
   정상 벡터는 world 회전 R만 곱한다. 평행이동을 더하지 않는다. 강체변환에서 반경은 유지된다.

법선은 원래 관측 카메라를 향한다. 뷰어 카메라를 움직여도 자동으로 다시 뒤집지 않는다.
경계 제한은 이웃 간 깊이 불연속을 줄이는 보수적 처리이며 완벽한 윤곽 보존을 보장하지 않는다.
잘못된 stereo 매칭·깊이 노이즈도 법선에 영향을 준다. 무효점에 가짜 법선을 넣지 않는다.
voxel 간 빈 공간을 메우기 위해 반경을 키우지 않으므로 축소 후에도 구멍이 남을 수 있다.

## 파일과 좌표계

- `surface.bin`: header 없는 32-byte SRF1 레코드 배열.
- `surface.npz`: 같은 배열을 point_id, normal, radius_m, support, flags, reserved로 분리.
- `surface.json`: 개수, 유효 법선 수, 단위, 좌표계, 기본 파라미터.
- `surface.ply`: metre XYZ + nx/ny/nz + radius_m + RGB + point_id + surface_flags.
- KITTI: run 루트의 `map_points.bin` ↔ `surface.bin`, 각 frame의 `new_points.bin` ↔ `surface.bin`.
  두 파일의 점 수·순서·ID가 같아야 한다. 좌표와 법선은 이미 world_first_left_camera다.
  world 축은 첫 왼쪽 카메라 기준 +X 오른쪽,+Y 아래,+Z 앞. 포즈를 다시 적용하지 않는다.
- 단일 쌍 dense_reconstruct: `point_cloud.npy`/기존 PLY/JSON 행 번호가 point_id다.
  법선은 original_left_camera 좌표, radius는 항상 metre다.
  기존 XYZ는 calibration의 원래 단위를 유지하므로 B는 `surface.ply`의 metre 좌표를 사용하거나
  `diagnostics.json.surface_meters_per_unit`을 원래 XYZ에 곱해야 한다.
- `surface_preview.png`: 색 법선·CPU Lambert·반경 진단. 검증 스크립트가 생성한다.

알 수 없는 보정 단위(예: square)는 실제 길이 없이 metre로 가정하지 않는다.
이 경우 기존 복원 출력은 생성하되 surface 출력을 생략하고 diagnostics.surface_exported=false로 기록한다.
실측 체커보드 한 칸이 24 mm이고 보정의 한 단위가 한 칸이면 `--meters-per-unit 0.024`를 준다.
이 값은 예시이며 실제 보드 치수를 측정해야 한다.

## SRF1 payload: little-endian, stride 32 bytes

| offset | 형식 | 필드 | 의미 |
|---:|---|---|---|
| 0 | uint32 | point_id | 대응 STP1과 같은 ID |
| 4,8,12 | float32 × 3 | nx,ny,nz | 위치와 같은 좌표계의 단위 법선 |
| 16 | float32 | radius_m | 원형 splat 반경, metre (직경 아님) |
| 20 | float32 | support | 연결된 4이웃 수 / 4; 무효=0, 정확도 확률 아님 |
| 24 | uint32 | flags | bit0 유효 법선/반경; bit1 이웃 부족/경계; bit2 반경 제한됨 |
| 28 | uint32 | reserved | 반드시 0 |

유효점: normal 길이 약 1, radius>0. 무효점: bit0=0, normal=(0,0,0), radius=0, support=0.
무효점에도 기존 XYZRGB와 ID는 남는다. 일반 점으로 표시하거나 생략할지는 B가 선택한다.
SGM confidence와 surface support는 다른 값이다. 둘 중 하나를 다른 값으로 덮어쓰지 않는다.

## SRF1 packet header: 32 bytes

| offset | 형식 | 값 |
|---:|---|---|
| 0 | char[4] | SRF1 |
| 4 | uint16 | version=1 |
| 6 | uint16 | header_bytes=32 |
| 8 | uint32 | frame_id |
| 12 | uint32 | chunk_index |
| 16 | uint32 | point_count |
| 20 | uint32 | record_stride=32 |
| 24 | uint32 | bit0=frame의 마지막 chunk; 나머지 0 |
| 28 | uint32 | reserved=0 |

전체 길이 `32 + point_count*32`. surface payload flags와 header flags는 별개다.
0점 frame도 point_count=0,last=1인 header-only STP1, SRF1을 각각 전송한다.
기존 STP1 decoder에 SRF1을 넘기면 안 된다. magic을 구분하고 `read_surface.mjs`를 사용한다.

## WebSocket: 선택적 확장, 기존 클라이언트 호환

1. 서버 stream_begin에 `surface_schema:"SRF1"`을 알린다. 과거 run은 null/미제공.
2. 새 클라이언트는 client_ready에 `surface_schema:"SRF1"`을 명시한다.
   이 필드를 생략하는 기존 클라이언트에는 STP1만 전송한다.
3. `frame_begin → STP1(chunk k) → ack → SRF1(chunk k) → surface_ack` 순서를 반복한다.
4. `surface_ack` 필드: type, stream_id, frame_id, chunk_index. 값이 정확히 일치해야 다음 chunk가 온다.
   ACK timeout/연결 종료/재연결 reset 규칙은 STP1과 같다.
5. 각 SRF1의 frame/chunk/count/last/point_id 배열은 바로 앞 STP1과 일치해야 한다.
   B는 두 데이터를 결합한 뒤 splat 렌더링에 공개한다. STP1 ACK는 수신 임시 버퍼 저장 후 보내도 된다.
6. stream_end의 sha256은 STP1 payload 전체, surface_sha256은 SRF1 payload 전체의 SHA256이다.
   header는 해시에 넣지 않는다. 확장을 요청하지 않은 경우 surface_sha256=null.
7. 새 stream_id와 reset=true면 위치·표면 버퍼를 모두 비우고 재수신한다.

각 점은 기존 32 + 표면 32 = 64 payload bytes. 패킷 header와 JSON은 별도다.
갱신된 receive_stream.mjs는 확장을 자동 협상하고 두 payload를 별도 파일로 저장하며 해시를 검사한다.

## B의 렌더링 연결

`readSurfaceBuffer`/`readSurfacePacket`은 normals(Float32Array), radii(Float32Array),
pointIds(Uint32Array), support(Float32Array), flags(Uint32Array)를 반환한다.

- 위치를 보기용 `(X,-Y,-Z)`로 바꾸면 법선도 `(nx,-ny,-nz)`로 한 번 변환한다.
- 강체 변환에서는 n'=R n. 비균일 model scale을 추가한다면 inverse-transpose normal matrix가 필요하고,
  하나의 원형 radius 값만으로 비균일하게 변형된 타원을 표현할 수 없다는 점에 유의한다.
- 반경은 gl_PointSize의 pixel 값이 아니다. 단순 화면 정렬 점의 근사 직경은 `2*f_pixel*r/z`지만,
  표면 방향 splat은 n에 수직인 tangent/bitangent를 만들어 world 공간 quad를 구성한 뒤 투영하는 것이 적절하다.
- 예: n과 평행하지 않은 축 a를 고르고 t=normalize(cross(a,n)), b=cross(n,t).
  quad vertex = P + r*(sx*t+sy*b), sx,sy∈{-1,+1}. fragment에서 원 밖을 discard하면 원형 표면 splat이다.
- 조명은 같은 좌표계에서 `max(dot(n,light_direction),0)` 등을 직접 GLSL로 구현한다.
- shader·블렌딩·EDL·LOD는 B의 구현 범위이며 A 출력이 이를 자동으로 수행하지 않는다.

## 실행과 검증

프로젝트 폴더에서 Python 의존성을 설치하고 실행한다.

```bash
python -m pip install -r requirements.txt -r requirements-streaming.txt -r requirements-visualization.txt
python -m unittest discover -s tests
python kitti_pipeline.py --root /path/to/kitti_data --frames 0 --output /path/to/new_run
python verify_surface.py --run /path/to/new_run
python stream_server.py replay --run /path/to/new_run --port 8767 --once
# 다른 터미널 (Node 22 이상)
node stream_client/receive_stream.mjs ws://127.0.0.1:8767 /path/to/received
python verify_surface.py --run /path/to/new_run --received /path/to/received
```

일반 보정된 스테레오 한 쌍:

```bash
python dense_reconstruct.py --left left.png --right right.png --calibration calibration.json --method sgm --output new_dense
python verify_surface.py --run new_dense
```

검증은 정면/경사 평면의 해석적 법선, 깊이 단차, 결측·빈 입력, 반경 제한,
회전과 wire layout, 선택적 협상·빈 packet·기존 client 호환을 포함한다.
실제 영상은 법선 정답이 없으므로 단위 길이·방향·점 ID·회전·바이트 일치 및 시각화로 검사한다.
실제 영상에서 법선 정확도를 수치로 입증했다는 뜻은 아니다.
