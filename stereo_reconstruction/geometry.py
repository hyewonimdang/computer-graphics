"""OpenCV 없이 구현한 임의 차원 동차 좌표 연산."""
import numpy as np


def to_homogeneous(points):
    """(..., D) → (..., D+1): 마지막 성분에 1을 추가한다.

    동차 좌표는 0 아닌 배율의 곱을 같은 점으로 표현한다. 따라서
    (u,v) ↔ (u,v,1), (X,Y,Z) ↔ (X,Y,Z,1) 모두 같은 규칙을 쓴다.
    단일 점 (D,) 및 빈 batch (0,D)도 처리한다.
    """
    points = np.asarray(points, dtype=np.float64)
    if points.ndim < 1 or points.shape[-1] < 1:
        raise ValueError('좌표의 마지막 차원은 1 이상이어야 합니다.')
    return np.concatenate([points, np.ones(points.shape[:-1] + (1,))], axis=-1)


def from_homogeneous(points, eps=1e-12):
    """(..., D+1) → (..., D): 마지막 성분 w로 앞의 성분을 나눈다.

    먼저 각 벡터의 최대 절댓값으로 정규화하여 배율에 불변인 w 검사를
    수행한다. |w|≈0은 무한원점으로 간주하고 해당 행 전체를 NaN으로
    반환한다. NaN/Inf 입력 역시 NaN 행이 되어 후속 필터에서 제거된다.
    """
    points = np.asarray(points, dtype=np.float64)
    if points.ndim < 1 or points.shape[-1] < 2 or eps <= 0:
        raise ValueError('동차 좌표 차원은 2 이상, eps는 양수여야 합니다.')
    scale = np.max(np.abs(points), axis=-1, keepdims=True)
    normalized = np.zeros_like(points)
    valid = np.isfinite(points).all(axis=-1, keepdims=True) & (scale > 0)
    np.divide(points, scale, out=normalized, where=valid)
    w = normalized[..., -1:]
    result = np.full(points.shape[:-1] + (points.shape[-1] - 1,), np.nan)
    np.divide(normalized[..., :-1], w, out=result, where=valid & (np.abs(w) > eps))
    return result
