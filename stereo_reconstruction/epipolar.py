"""F의 robust 추정만 OpenCV 사용, 제약 검사 및 거리 계산은 직접 구현."""
import cv2
import numpy as np
from geometry import to_homogeneous


def _paired_points(left, right):
    left, right = np.asarray(left, dtype=np.float64), np.asarray(right, dtype=np.float64)
    if left.ndim != 2 or left.shape[1] != 2 or left.shape != right.shape:
        raise ValueError('양쪽 대응점은 동일한 N×2 shape이어야 합니다.')
    if not np.isfinite(left).all() or not np.isfinite(right).all():
        raise ValueError('대응점에 NaN/Inf가 있습니다.')
    return left, right


def _fundamental(F):
    F = np.asarray(F, dtype=np.float64)
    if F.shape != (3, 3) or not np.isfinite(F).all() or np.linalg.norm(F) == 0:
        raise ValueError('F는 유한한 0 아닌 3×3 행렬이어야 합니다.')
    return F


def estimate_fundamental_matrix(points_left, points_right, threshold=1.5, confidence=0.999, seed=42):
    """RANSAC F와 bool inlier mask를 반환. x_right^T F x_left = 0.

    F는 왼쪽 점을 오른쪽 에피폴라 직선으로 대응시키는 rank-2 행렬이다.
    scale이 정해지지 않으므로 Frobenius norm=1로 통일한다. OpenCV
    추정 뒤 SVD의 최소 특이값을 0으로 만들어 rank-2 조건을 보장한다.
    """
    left, right = _paired_points(points_left, points_right)
    if threshold <= 0 or not 0 < confidence < 1:
        raise ValueError('threshold는 양수, confidence는 (0,1)이어야 합니다.')
    if len(left) < 8:
        raise ValueError(f'F 추정에 최소 8개 대응점이 필요합니다: 현재 {len(left)}개')
    for points in (left, right):
        if len(np.unique(points, axis=0)) < 8 or np.linalg.matrix_rank(points - points.mean(axis=0)) < 2:
            raise ValueError('중복/일직선 대응점으로 F를 안정적으로 추정할 수 없습니다.')
    cv2.setRNGSeed(int(seed))
    F, mask = cv2.findFundamentalMat(left, right, cv2.FM_RANSAC, threshold, confidence, 5000)
    if F is None or mask is None or F.shape != (3, 3):
        raise ValueError('F 추정 실패: 겹침, 텍스처, 시차가 충분한 이미지를 사용하세요.')
    F = _fundamental(F)
    u, s, vt = np.linalg.svd(F)
    if s[1] <= s[0] * 1e-12:
        raise ValueError('퇴화한 F 행렬입니다.')
    s[-1] = 0
    F = (u * s) @ vt
    mask = mask.ravel().astype(bool)
    if mask.sum() < 8:
        raise ValueError('RANSAC inlier가 8개 미만입니다.')
    return F / np.linalg.norm(F), mask


def compute_epipolar_errors(F, points_left, points_right):
    """각 행마다 signed scalar x_right^T F x_left를 직접 계산한다.

    batch는 행 벡터로 저장하므로 F x_left 대신 x_left @ F.T를 사용.
    이상적 대응점은 0이며, 이 값은 F의 scale에 의존하여 픽셀 거리가 아니다.
    """
    left, right = _paired_points(points_left, points_right)
    xl, xr = to_homogeneous(left), to_homogeneous(right)
    return np.sum(xr * (xl @ _fundamental(F).T), axis=1)


def epipolar_constraint_error(F, point_left, point_right):
    return float(compute_epipolar_errors(F, np.asarray(point_left).reshape(1, 2),
                                         np.asarray(point_right).reshape(1, 2))[0])


def epipolar_lines(F, points_left, points_right):
    """(왼쪽 직선, 오른쪽 직선). 각 직선 (a,b,c)는 au+bv+c=0.

    l_right=F*x_left, l_left=F.T*x_right. OpenCV 직선 함수 미사용.
    """
    left, right = _paired_points(points_left, points_right)
    F = _fundamental(F)
    return to_homogeneous(right) @ F, to_homogeneous(left) @ F.T


def point_to_line_distances(points, lines):
    """수직 거리 |a*u+b*v+c| / sqrt(a²+b²), 단위는 pixel.

    각 line을 먼저 정규화해 F의 임의 배율에 따른 수치 불안정을 줄인다.
    a=b=0인 퇴화 직선은 거리를 inf로 반환하여 필터에서 제외한다.
    """
    points, lines = np.asarray(points, float), np.asarray(lines, float)
    if points.ndim != 2 or points.shape[1] != 2 or lines.shape != (len(points), 3):
        raise ValueError('points=N×2, lines=N×3이어야 합니다.')
    scale = np.max(np.abs(lines), axis=1, keepdims=True)
    normalized = np.zeros_like(lines)
    np.divide(lines, scale, out=normalized, where=np.isfinite(scale) & (scale > 0))
    denominator = np.hypot(normalized[:, 0], normalized[:, 1])
    numerator = np.abs(np.sum(to_homogeneous(points) * normalized, axis=1))
    distances = np.full(len(points), np.inf)
    np.divide(numerator, denominator, out=distances, where=(denominator > 1e-15) & np.isfinite(numerator))
    return distances


def compute_epipolar_distances(F, points_left, points_right):
    left_lines, right_lines = epipolar_lines(F, points_left, points_right)
    return (point_to_line_distances(points_left, left_lines),
            point_to_line_distances(points_right, right_lines))
