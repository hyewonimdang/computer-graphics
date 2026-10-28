법선 v2 기본 적용: [SURFACE_V2_GUIDE.md](SURFACE_V2_GUIDE.md). STP1/SRF1 wire layout 변경 없음.

# 표면 데이터 확장 (2026-10-28)

법선·metre splat 반경을 SRF1 보조 스트림/파일로 제공합니다. 기존 STP1 32-byte 레코드는 유지합니다.
새 규약·좌표계·협상·실행 방법은 [SURFACE_GUIDE.md](SURFACE_GUIDE.md)를 참고하세요.

# A → B/C 전달 자료

**새 A 파이프라인:** KITTI 로딩·제공 pose 누적·voxel·실제 WebSocket이 추가되었습니다.
최신 실행은 [KITTI_GUIDE.md](KITTI_GUIDE.md), B 수신은 [STREAM_PROTOCOL.md](STREAM_PROTOCOL.md)를 따릅니다.
아래 `team_handoff_v1.zip`은 이전 Middlebury 독립 scene 전달본의 설명입니다.

최종 전달 ZIP: `handoff/team_handoff_v1.zip`

ZIP을 풀고 `team_handoff_v1/README_KO.md`부터 읽는다.
데이터는 직접 구현한 SGM의 최종 Middlebury 세 장면 결과이며,
전달 좌표는 **metre**, 왼쪽 카메라 기준 +X 오른쪽 / +Y 아래 / +Z 앞쪽이다.

- `DATA_FORMAT_KO.md`: XYZRGB·시차·신뢰도,32바이트 바이너리, 인덱스 연결, 단위/좌표 변환.
- `INTEGRATION_KO.md`: A/B/C 연결 지점, 포즈/다운샘플링 주의점, 선택적 WebSocket packet 계약.
- `reader/read_points.mjs`: 외부 패키지 없는 JavaScript decoder와 단일 카메라 보기용 좌표 변환.
- `scenes/*/points.bin`: B의 기본 입력. PLY는 같은 점의 외부 뷰어용 대체 파일.
- `scenes/*/analysis.npz`: A의 검증용 배열, 픽셀 대응·전체 map 포함.

세 scene은 독립 장면이며 시간순 프레임이나 월드 누적 맵이 아니다.
KITTI 로딩, LiDAR 처리, 실제 pose, WebSocket 서버는 이 전달 자료에서 구현하지 않았다.
역할표의 SGBM과 달리 현재 추정기는 외부 SGBM 함수를 호출하지 않는 직접 구현 SGM이다.

Python 프로젝트의 `pack_handoff.py`는 기존 SGM 결과로부터 같은 scene 레코드를 재생성한다.
원본은 mm지만 패키지 변환 후에는 metre이므로 렌더러에서 다시1000으로 나누지 않는다.
