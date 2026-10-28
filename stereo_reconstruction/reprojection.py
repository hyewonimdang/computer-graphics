"""Phase 5: P X의 동차 정규화 및 픽셀 오차를 NumPy로 직접 계산."""
import numpy as np
from geometry import to_homogeneous, from_homogeneous


def project_points(P, points_3d):
    """3×4 P와 N×3 XYZ → N×2 pixel 좌표.

    각 점 X_h=[X,Y,Z,1]에 대해 x_h=P X_h이다. batch는 행벡터이므로
    X_h @ P.T로 계산한 뒤 마지막 성분으로 나눈다. 투영 분모≈0 또는
    NaN/Inf는 NaN 행을 반환한다. 음의 depth도 수학적 투영은 가능하므로
    cheirality는 별도의 3D 검증에서 처리한다. cv2.projectPoints 미사용.
    """
    P, xyz = np.asarray(P, float), np.asarray(points_3d, float)
    if P.shape != (3, 4) or not np.isfinite(P).all() or np.linalg.matrix_rank(P) != 3:
        raise ValueError('P는 유한한 rank-3 3×4 행렬이어야 합니다.')
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError('points_3d는 N×3이어야 합니다.')
    with np.errstate(invalid='ignore', over='ignore'):
        homogeneous = to_homogeneous(xyz) @ P.T
    return from_homogeneous(homogeneous)


def project_point(P, point_3d):
    """단일 (3,) XYZ → (2,) pixel. 무효점은 NaN 반환."""
    point = np.asarray(point_3d, float)
    if point.shape != (3,):
        raise ValueError('point_3d는 (3,)이어야 합니다.')
    return project_points(P, point[None])[0]


def compute_reprojection_error(observed, projected):
    """N×2 두 배열의 행별 L2 거리(px). 단일 (2,) 입력은 float 반환.

    sqrt((u-u_hat)^2+(v-v_hat)^2)를 직접 계산한다. 유한하지 않은
    관측/투영 좌표의 오차는 inf로 처리하여 threshold 필터에서 제외한다.
    """
    observed, projected = np.asarray(observed, float), np.asarray(projected, float)
    if observed.shape != projected.shape or observed.ndim not in (1, 2) or observed.shape[-1] != 2:
        raise ValueError('observed/projected는 동일한 (2,) 또는 N×2 shape이어야 합니다.')
    with np.errstate(invalid='ignore', over='ignore'):
        diff = observed - projected
        errors = np.hypot(diff[..., 0], diff[..., 1])
    errors = np.where(np.isfinite(errors), errors, np.inf)
    return float(errors) if observed.ndim == 1 else errors
