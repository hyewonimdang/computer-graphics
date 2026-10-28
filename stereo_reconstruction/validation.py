"""검증 전용 OpenCV 비교. 핵심 복원 경로는 이 모듈에 의존하지 않는다."""
import numpy as np
from geometry import from_homogeneous


def compare_with_opencv(points1, points2, P1, P2, custom_xyz):
    """같은 pixel 좌표와 P로 cv2 결과를 비교; 양쪽 유한한 행만 통계에 사용.

    OpenCV와 일치한다는 사실만으로 물리적 정확성이 보장되지는 않는다.
    알려진 3D 정답과의 정확도 테스트를 별도로 수행해야 한다.
    """
    import cv2  # 검증 옵션에서만 고수준 삼각측량을 사용한다.
    p1, p2 = np.asarray(points1, float), np.asarray(points2, float)
    custom = np.asarray(custom_xyz, float)
    if p1.ndim != 2 or p1.shape[1] != 2 or p1.shape != p2.shape or custom.shape != (len(p1), 3):
        raise ValueError('대응점 N×2, custom_xyz N×3 shape이 일치해야 합니다.')
    xyz = np.full_like(custom, np.nan)
    finite = np.isfinite(p1).all(axis=1) & np.isfinite(p2).all(axis=1)
    if finite.any():
        h = cv2.triangulatePoints(np.asarray(P1, float), np.asarray(P2, float),
                                 p1[finite].T.copy(), p2[finite].T.copy()).T
        xyz[finite] = from_homogeneous(h)
    comparable = np.isfinite(xyz).all(axis=1) & np.isfinite(custom).all(axis=1)
    differences = np.full(len(p1), np.nan)
    differences[comparable] = np.linalg.norm(custom[comparable] - xyz[comparable], axis=1)
    return dict(opencv_xyz=xyz, comparable_mask=comparable, differences=differences,
                compared_count=int(comparable.sum()),
                mean_difference=float(differences[comparable].mean()) if comparable.any() else None,
                max_difference=float(differences[comparable].max()) if comparable.any() else None)
