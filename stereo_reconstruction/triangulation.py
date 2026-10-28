"""Phase 4: DLT 행렬 구성과 SVD를 직접 구현. OpenCV 의존성 없음."""
import numpy as np
from geometry import from_homogeneous


def _projection_pair(P1, P2):
    matrices, centers = [], []
    for P in (P1, P2):
        P = np.asarray(P, dtype=np.float64)
        if P.shape != (3, 4) or not np.isfinite(P).all() or np.linalg.matrix_rank(P) != 3:
            raise ValueError('P는 유한한 rank-3 3×4 투영행렬이어야 합니다.')
        matrices.append(P)
        center = np.linalg.svd(P)[2][-1]
        centers.append(center / np.linalg.norm(center))
    if min(np.linalg.norm(centers[0]-centers[1]), np.linalg.norm(centers[0]+centers[1])) <= 1e-12:
        raise ValueError('카메라 중심이 동일하여 triangulation baseline이 없습니다.')
    return matrices


def triangulate_points_dlt(points1, points2, P1, P2, return_details=False, eps=1e-12):
    """N×2 대응점 → N×3 XYZ. 선택적으로 (XYZ, details) 반환.

    x × (P X_h)=0에서 독립인 두 식을 카메라마다 택한다:
      u*P[2]-P[0], v*P[2]-P[1]. 네 행으로 A(4×4)를 구성한다.
    ||X_h||=1 제약 아래 ||A X_h||를 최소화하는 해는 가장 작은
    특이값에 대응하는 우측 특이벡터 Vt[-1]이다. 잡음이 있으면 정확한
    nullspace 대신 최소 제곱 해가 된다. 마지막 W로 나눠 XYZ를 얻는다.

    SVD 벡터의 부호는 임의지만 XYZ는 동일하다. 최소 3개의 독립 제약이
    없는 점, NaN/Inf 입력, W≈0은 XYZ=NaN으로 표시한다. 행 순서는
    유지한다. numerical_valid는 양의 깊이/재투영 품질을 보장하지 않는다.
    그런 물리적·품질 필터는 Phase 5에서 추가한다.
    """
    if not np.isfinite(eps) or eps <= 0:
        raise ValueError('eps는 유한한 양수여야 합니다.')
    P1, P2 = _projection_pair(P1, P2)
    p1, p2 = np.asarray(points1, float), np.asarray(points2, float)
    if p1.ndim != 2 or p1.shape[1] != 2 or p1.shape != p2.shape:
        raise ValueError('두 대응점 배열은 동일한 N×2 shape이어야 합니다.')
    n = len(p1)
    homogeneous = np.full((n, 4), np.nan)
    singular_values = np.full((n, 4), np.nan)
    rank_ok = np.zeros(n, bool)
    finite_input = np.isfinite(p1).all(axis=1) & np.isfinite(p2).all(axis=1)
    for i in np.flatnonzero(finite_input):
        u1, v1 = p1[i]
        u2, v2 = p2[i]
        with np.errstate(over='ignore', invalid='ignore'):
            A = np.stack([u1 * P1[2] - P1[0], v1 * P1[2] - P1[1],
                          u2 * P2[2] - P2[0], v2 * P2[2] - P2[1]])
        if not np.isfinite(A).all():
            continue
        try:
            _, s, vt = np.linalg.svd(A)
        except np.linalg.LinAlgError:
            continue
        singular_values[i] = s
        homogeneous[i] = vt[-1]
        # rank<3이면 독립적인 null 방향이 둘 이상이라 단일 3D점을 정할 수 없다.
        rank_ok[i] = s[2] > eps * s[0]
    xyz = from_homogeneous(homogeneous, eps=eps)
    valid = rank_ok & np.isfinite(xyz).all(axis=1)
    xyz[~valid] = np.nan
    details = dict(points_homogeneous=homogeneous, singular_values=singular_values,
                   finite_input=finite_input, numerical_valid=valid)
    return (xyz, details) if return_details else xyz


def triangulate_point_dlt(point1, point2, P1, P2):
    """단일 대응점의 직접 DLT 결과 (3,) 반환. 무효점은 NaN 3개."""
    p1, p2 = np.asarray(point1, float), np.asarray(point2, float)
    if p1.shape != (2,) or p2.shape != (2,):
        raise ValueError('단일 점은 (2,) shape이어야 합니다.')
    return triangulate_points_dlt(p1[None], p2[None], P1, P2)[0]
