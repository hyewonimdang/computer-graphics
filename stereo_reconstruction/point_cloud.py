"""Phase 5·6: 좌표 검증 및 RGB 색상 할당."""
import numpy as np
from camera import validate_pose


def validate_3d_points(points_3d, R=None, t=None, min_depth=1e-6,
                       max_depth=1000.0, max_distance=1000.0, return_details=False):
    """XYZ → 유효 mask. 세계 좌표는 왼쪽 카메라 좌표계.

    X_right=R X_left+t의 세 번째 성분이 오른쪽 depth이다. 양쪽 모두
    min_depth<Z<=max_depth이고 카메라 중심으로부터 거리<=max_distance여야
    한다. W≈0인 DLT 결과는 앞 단계에서 NaN으로 변환되므로 여기서 제거된다.
    한계값은 t/XYZ와 같은 단위이며 현실 장면에 맞게 설정해야 한다.
    R,t가 모두 생략되면 왼쪽 카메라만 검사한다. 둘 중 하나만 주면 오류.
    """
    xyz = np.asarray(points_3d, float)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError('points_3d는 N×3이어야 합니다.')
    if not np.isfinite([min_depth, max_depth, max_distance]).all() or not (0 <= min_depth < max_depth) or max_distance <= 0:
        raise ValueError('0<=min_depth<max_depth, max_distance>0인 유한한 한계값이 필요합니다.')
    if (R is None) != (t is None):
        raise ValueError('오른쪽 카메라의 R,t는 함께 지정하세요.')
    finite = np.isfinite(xyz).all(axis=1)
    with np.errstate(invalid='ignore', over='ignore'):
        if R is not None:
            R, t = validate_pose(R, t)
            right = xyz @ R.T + t
        else:
            right = xyz
        depth_left, depth_right = xyz[:, 2], right[:, 2]
        distance_left = np.linalg.norm(xyz, axis=1)
        distance_right = np.linalg.norm(right, axis=1)
    depth_valid = (depth_left > min_depth) & (depth_right > min_depth) & (depth_left <= max_depth) & (depth_right <= max_depth)
    distance_valid = (distance_left <= max_distance) & (distance_right <= max_distance)
    valid = finite & depth_valid & distance_valid
    details = dict(finite_3d=finite, depth_valid=depth_valid, distance_valid=distance_valid,
                   depth_left=depth_left, depth_right=depth_right,
                   distance_left=distance_left, distance_right=distance_right)
    return (valid, details) if return_details else valid


def image_point_mask(points, image_shape):
    """유한한 관측점이 이미지 영역 0<=u<width, 0<=v<height 안에 있는가."""
    points = np.asarray(points, float)
    if points.ndim != 2 or points.shape[1] != 2 or len(image_shape) < 2 or min(image_shape[:2]) <= 0:
        raise ValueError('N×2 좌표와 유효한 이미지 shape이 필요합니다.')
    h, w = image_shape[:2]
    return np.isfinite(points).all(axis=1) & (points[:, 0] >= 0) & (points[:, 0] < w) & (points[:, 1] >= 0) & (points[:, 1] < h)


def assign_colors(points_3d, image, image_points):
    """N×3 XYZ + 왼쪽 BGR uint8 이미지 + N×2 관측점 → N×6 XYZRGB.

    최근접 픽셀: floor(coord+0.5), 이미지 끝의 소수 좌표는 마지막 픽셀로
    한정한다. 입력 좌표 자체가 이미지 밖이면 잘못된 색상 연결을 막기 위해
    오류를 낸다. 픽셀 접근은 image[v,u], 채널은 BGR[::-1]로 RGB 변환.
    XYZ와 색상이 같은 float64 배열에 저장되지만 RGB 값은 0..255 정수다.
    이미지에는 gamma 변환을 하지 않는다. 렌더러에서 255로 나눠 사용할 수 있다.
    """
    xyz, points = np.asarray(points_3d, float), np.asarray(image_points, float)
    image = np.asarray(image)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or points.shape != (len(xyz), 2):
        raise ValueError('XYZ N×3과 관측점 N×2의 행 수가 같아야 합니다.')
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8 or min(image.shape[:2]) <= 0:
        raise ValueError('image는 H×W×3 BGR uint8이어야 합니다.')
    if not np.isfinite(xyz).all() or not image_point_mask(points, image.shape).all():
        raise ValueError('색상 할당에는 유한한 XYZ와 이미지 내부 관측점이 필요합니다.')
    pixels = np.floor(points + .5).astype(np.int64)
    pixels[:, 0] = np.minimum(pixels[:, 0], image.shape[1]-1)
    pixels[:, 1] = np.minimum(pixels[:, 1], image.shape[0]-1)
    rgb = image[pixels[:, 1], pixels[:, 0], ::-1]
    return np.column_stack((xyz, rgb))
