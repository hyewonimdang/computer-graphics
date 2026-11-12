# B · 스테레오 표면 렌더러

저장소 루트에서 실행합니다. 먼저 [Python 환경 설정](../README.md#시작하기)을 마치고 KITTI 첫 프레임으로 렌더러 입력을 생성합니다. 이미 A에서 생성한 프레임이 있으면 아래의 FRAME_DIR을 그 경로로 지정하세요.

```sh
(cd stereo_reconstruction && python download_kitti_sample.py --output data/kitti --frames 0 && python kitti_pipeline.py --root data/kitti --frames 0 --output runs/kitti_frame0)
npm ci --prefix renderer
FRAME_DIR=stereo_reconstruction/runs/kitti_frame0/frames/0000000000
mkdir -p renderer/data
cp "$FRAME_DIR/frame.json" "$FRAME_DIR/new_points.bin" "$FRAME_DIR/surface.bin" \
   "$FRAME_DIR/left_rectified.png" "$FRAME_DIR/right_rectified.png" renderer/data/
python3 -m http.server 8735 --bind 127.0.0.1
```

브라우저에서 [렌더러 열기](http://127.0.0.1:8735/renderer/). 입력 파일은 모두 같은 프레임에서 가져옵니다. 새 점만 저장된 후속 프레임은 그 프레임의 증분만 표시합니다.

PowerShell에서 프레임을 복사하고 뷰어를 실행하는 방법입니다.

```powershell
npm.cmd ci --prefix renderer
$frameDir = 'stereo_reconstruction/runs/kitti_frame0/frames/0000000000'
New-Item -ItemType Directory -Force renderer/data | Out-Null
$files = 'frame.json','new_points.bin','surface.bin','left_rectified.png','right_rectified.png'
Copy-Item ($files | ForEach-Object { Join-Path $frameDir $_ }) -Destination renderer/data
python -m http.server 8735 --bind 127.0.0.1
```

A의 공식 디코더로 두 payload를 읽고 개수와 point_id 순서를 대조합니다. 위치와 법선은 (X,−Y,−Z)로 변환하며 반경은 미터 그대로 사용합니다. 완성된 맵의 기본 카메라는 frame.json의 K_rect와 카메라 포즈를 사용합니다.

Three.js는 GPU 버퍼·카메라·드로 호출과 마우스 조작을 담당합니다. splats.js의 GLSL은 각 인스턴스의 접평면과 원형 경계, 확산광과 정반사를 계산합니다. 법선 모드의 RGB는 뷰어 월드 좌표의 단위 법선을 [0,1]로 매핑한 값입니다. 무효 법선은 모든 모드에서 원래 색의 작은 점으로 표시합니다.

### 사진에서 표면으로

`renderer/data/left_rectified.png`와 `right_rectified.png`를 우선 사용합니다. 두 사진의 픽셀을 복원 데이터와 직접 연결할 수 있습니다. 정류 사진 한 쌍이 없으면 같은 프레임의 `left.png`와 `right.png`를 사용합니다. 사진 한 쌍이 없으면 3D 맵으로 바로 열립니다.

위 KITTI 예제에서 프로젝트에 들어온 원본 파일은 다운로드한 시퀀스의 `image_02/data/0000000000.png`와 `image_03/data/0000000000.png`입니다. 원본 모드를 쓰려면 해당 파일을 `renderer/data/left.png`와 `renderer/data/right.png`로 복사하고, 정류 사진은 별도 폴더에 보관합니다.

1. 맵 앞쪽의 사진 두 장은 좌우 촬영 기준점 위에 놓입니다. **왼쪽 사진에서 점 꺼내기**를 누르면 사진이 파랗게 물들고, 방울이 대응하는 복원점에 파란 자국을 남깁니다.
2. **오른쪽 사진에서 점 꺼내기**를 누르면 분홍 방울이 나머지 표면을 채웁니다. 양쪽 물감은 각 지점에 닿은 뒤 서서히 원래 맵 색으로 돌아옵니다.
3. 점이 제자리에 놓이면 조명 Splat으로 이어집니다. 연출 중에도 회전하고 확대할 수 있고, 완성되면 현재 시점을 유지합니다. **처음부터**로 재시작하거나 **3D 맵 바로 보기**로 건너뛸 수 있습니다.

어느 표면을 먼저 그릴지는 왼쪽 사진의 정류 가로 좌표로 정합니다. 두 단계 모두 이미 계산된 스테레오 복원 좌표를 사용합니다.

`stereo-transition.js`는 각 복원점의 좌우 출처를 연출 순서에 맞게 나눕니다. 왼쪽은 `(u,v)`, 오른쪽은 `(u−disparity,v)`에서 출발하며, 두 단계 모두 같은 스테레오 복원 좌표를 사용합니다. 이동하는 점과 제자리에 나타나는 Splat은 같은 착지 시간을 씁니다. 원본 사진 모드에서는 정류 좌표를 정규화해 출발 위치를 근사합니다.

### 표면 효과 비교
완성된 3D 맵에서 조명 splat 모드로 비교합니다. 세 체크박스는 각각 독립적이며 기본값은 꺼짐입니다. 확대는 유효한 원판 반경에만 1~3배를 곱합니다. 블렌딩은 가장 앞의 깊이를 먼저 구한 뒤 그 표면에서 1.5 cm 또는 깊이의 0.3% 중 큰 값 이내인 조각의 선형 RGB를 중심 가중 평균합니다. EDL은 주변 네 방향의 깊이 차이로 뒤쪽 표면을 어둡게 합니다. 배경은 음영 계산에서 제외합니다. 두 후처리는 단일 샘플 렌더 타깃을 사용하므로 기본 화면과 실루엣의 안티앨리어싱이 조금 다릅니다.

주변 간격 기반 반경을 켜면 고정 확대를 일시 대체합니다. 끄면 이전 고정 배율 설정으로 돌아갑니다. 강도 0%는 원래 반경, 100%는 추정 반경이며 사이 값은 선형 보간합니다. 정류 UV 반경 8픽셀에서 깊이와 접평면을 검사하고 법선이 유사한 세 방향 이상의 이웃 간격으로 반경을 추정합니다. 관측된 깊이 단차까지의 거리와 원래 반경의 4배로 추가 확대를 제한합니다. 이웃이 부족하거나 법선이 없으면 원래 표현을 유지합니다. 미관측 영역의 경계까지 보장하지는 않습니다.
