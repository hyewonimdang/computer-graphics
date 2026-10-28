법선 v2 기본 적용: [SURFACE_V2_GUIDE.md](SURFACE_V2_GUIDE.md). STP1/SRF1 wire layout 변경 없음.

# 표면 데이터 확장 (2026-10-28)

법선·metre splat 반경을 SRF1 보조 스트림/파일로 제공합니다. 기존 STP1 32-byte 레코드는 유지합니다.
새 규약·좌표계·협상·실행 방법은 [SURFACE_GUIDE.md](SURFACE_GUIDE.md)를 참고하세요.

# A→B 실제 스트림 계약: stereo-map-stream/1 + 기존 STP1

기존 handoff의 **32바이트 point 레코드와32바이트 packet header를 그대로 유지**한다.
`stream_client/read_points.mjs`는 이전 B용 decoder와 동일하며, 실제 WebSocket 수신으로 검증했다.
변경된 의미는 JSON 제어 메시지에서 명시한다: **XYZ는 pose가 적용된 world metre**, **ID는 stream-global**.
HTML/Three.js 코드는 이 모듈에 포함하지 않는다.

## 레코드 (little-endian, 32bytes)

| offset | 형식 | 의미 |
|---:|---|---|
| 0,4,8 | float32×3 | world X,Y,Z / metre |
| 12,13,14 | uint8×3 | RGB, sRGB encoded |
| 15 | uint8 | alpha255 |
| 16,18 | uint16×2 | 현재 observation rectified image의 u,v |
| 20 | float32 | 해당 관측의 disparity / pixel |
| 24 | float32 | SGM heuristic confidence0..1, 확률 아님 |
| 28 | uint32 | global point_id |

World는 KITTI frame0 왼쪽 camera2 기준이다. 원래 observation pixel/시차는 해당 frame의 값으로 유지된다.
**수신한 XYZ에 frame pose를 다시 곱하지 않는다.** pose는 관측 추적/picking/검증용이다.
뷰어의 +Y 위/-Z 앞 표현을 원하면 world 전체에 `(X,-Y,-Z)` 회전을 한 번만 적용한다.
카메라 경로·포인트·광선에 같은 world→viewer 변환을 적용한다.

## STP1 packet header (little-endian, 32bytes)

| offset | 형식 | 의미 |
|---:|---|---|
| 0 | ASCII4 | STP1 |
| 4 | uint16 | version1 |
| 6 | uint16 | headerBytes32 |
| 8 | uint32 | KITTI frameId (예:0,5,10,15,20) |
| 12 | uint32 | chunkIndex (frame별0부터) |
| 16 | uint32 | 이 chunk의 pointCount |
| 20 | uint32 | pointStride32 |
| 24 | uint32 | bit0: frame 마지막 chunk, 나머지0 |
| 28 | uint32 | reserved0 |
| 32.. | bytes | point 레코드 배열 |

길이=`32 + pointCount*32`. 기본4096점/chunk. 신규점0인 frame도 count0,last=true인32바이트 packet을 하나 보낸다.
현재 frame의 누적 map 전체를 보내는 것이 아니라 **신규 voxel 대표점만** 보낸다.

## 메시지 순서

1. 서버 → JSON `stream_begin`
   - `schema=stereo-map-stream/1`, `packet_schema=STP1`, UUID `stream_id`.
   - `reset=true`, `coordinate_frame=world_first_left_camera`, `position_unit=metre`, `pose_applied=true`.
   - `point_id_scope=stream_global`, `point_stride=32`, `frame_ids`, `config`.
2. B는 이전 GPU map/ID/chunk 상태를 초기화한 뒤 → JSON:

   ```json
   {"type":"client_ready","stream_id":"서버에서 받은 UUID"}
   ```

3. 서버 → JSON `frame_begin`: frame_id, timestamp_seconds, image_size, K_rect, baseline_m,
   T_world_from_observation_camera, point_count(이번 신규점), total_map_points, chunk_count.
4. 서버 → binary STP1 chunk.
5. B가 decoder로 읽고 GPU에 반영한 다음 → JSON ACK:

   ```json
   {"type":"ack","stream_id":"동일 UUID","frame_id":5,"chunk_index":0}
   ```

6. 서버는 맞는 ACK를 받은 뒤에만 다음 chunk를 보낸다.
7. frame의 마지막 chunk까지 ACK 후 서버 → JSON `frame_end`: added_points, total_map_points.
8. 모든 frame 완료 후 서버 → JSON `stream_end`: complete=true, map_points, frame_count, sha256.

sha256은 수신 순서대로 **각 packet의 header를 제외한 payload만** 연결한 전체 bytes의 SHA256이다.
`map_points.bin`은 바로 이 연결 결과이며 별도 header가 없다.

## 오류·재연결·메모리

- ACK timeout 기본30초. 잘못된 stream_id/frame/chunk, 잘못된 JSON, timeout이면 error 메시지 후 연결 종료.
- 메시지 수신 큐/쓰기 버퍼에 상한을 두고 chunk ACK를 기다리므로 임의로 전체 map을 계속 밀어 넣지 않는다.
- 현재는 활성 client1개만 지원하며 추가 동시 접속은1013으로 거절한다.
- 재연결에는 새 stream_id와 reset=true가 온다. B는 기존 map을 비우고0부터 다시 받는다.
- 같은 live 서버에서 완료된 frame은 파일 cache로 재생하고, 아직 계산하지 않은 frame부터 이어 계산한다.
  부분 chunk 이후 offset부터 재개하는 resume 프로토콜은 구현하지 않았다.
- 프로세스가 중단되어 summary가 없는 run은 replay 불가다. 완성된 run만 replay한다.
- 이전 UUID의 ACK를 새 stream에 보내거나 같은 chunk를 GPU에 두 번 append하면 안 된다.
- 별도 pause/resume 메시지는 없다. C가30초 넘게 ACK를 보류하면 연결이 종료된다.
  긴 일시정지는 재연결/reset 또는 C의 로컬 표시 제어로 처리해야 한다.
- transport는 localhost 개발용, 인증 없음. 실제 외부 서비스 배포는 별도 구성 대상이다.

## B의 최소 수신 예제

`stream_client/receive_stream.mjs`는 Node22+에서 동작하는 실제 수신/검증 예제다.
`fs/crypto` 부분은 검증 파일 저장용이며 브라우저 앱에서는 제거하고 같은 메시지 순서를 사용한다.

```javascript
socket.binaryType = 'arraybuffer';
// binary message:
const p = readPointPacket(event.data);
// p.positions는 이미 world/metre. p.rgb는 Uint8 RGB, p.confidence는 Float32.
// 신규 vertex만 GPU에 append한 뒤 해당 stream_id/frameId/chunkIndex로 ACK.
```

파일의 source pixel/시차는 frame별로 해석해야 한다. global point_id만으로 u/v를 전체 이미지의
선형 index라고 가정하면 안 된다. 누적 map에서 point_id에 대응한 source frame이 필요하면
`frame_begin.first_point_id`와 `point_count` 범위를 기록한다. 신규점0 frame의 first_point_id는null이다.

voxel downsampling은 centroid 대신 실제 관측을 유지한다. 따라서 이 v1 규약 안에서 시차/uv 의미가 유효하다.
향후 LiDAR-only 점이나 평균 융합점까지 보낼 때는 point-kind/attribute-validity를 가진 새 규약이 필요하다.
