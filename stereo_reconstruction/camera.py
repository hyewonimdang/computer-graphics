"""Phase 3: NumPy로 직접 구성하는 calibrated stereo geometry.

열벡터 기준 X_right=R@X_left+t. 세계 좌표계는 왼쪽 카메라 좌표계다.
"""
import json
from pathlib import Path
import numpy as np


def _matrix(value, shape, name):
    value = np.asarray(value, dtype=np.float64)
    if value.shape != shape or not np.isfinite(value).all():
        raise ValueError(f'{name}은 유한한 {shape} 배열이어야 합니다.')
    return value


def validate_intrinsics(K):
    K = _matrix(K, (3, 3), 'K')
    if not np.allclose(K[2], [0, 0, 1], atol=1e-10, rtol=0) or abs(K[1, 0]) > 1e-10:
        raise ValueError('K는 마지막 행 [0,0,1]인 상삼각 pinhole intrinsic이어야 합니다.')
    if K[0, 0] <= 0 or K[1, 1] <= 0 or np.linalg.cond(K) > 1e12:
        raise ValueError('초점 거리는 양수이며 K는 안정적으로 역변환 가능해야 합니다.')
    return K


def validate_pose(R, t):
    R = _matrix(R, (3, 3), 'R')
    t = np.asarray(t, dtype=np.float64)
    if t.shape == (3, 1):
        t = t[:, 0]
    t = _matrix(t, (3,), 't')
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-7, rtol=0) or not np.isclose(np.linalg.det(R), 1, atol=1e-7):
        raise ValueError('R은 R.T@R=I, det(R)=+1인 회전행렬이어야 합니다.')
    return R, t


def skew_symmetric(t):
    """[t]_x @ v = t × v인 외적 행렬을 직접 구성한다."""
    x, y, z = _matrix(t, (3,), 't')
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=np.float64)


def compute_essential_matrix(F, K_left, K_right):
    """E_raw=K_right.T@F@K_left. 정규화 카메라 좌표의 에피폴라 제약.

    x_pixel=K*x_normalized를 x_right.T F x_left=0에 대입하면 이 식이
    나온다. 이상적인 calibrated geometry에서는 E=[t]_x R이다.
    F에서 유도한 E는 임의의 scale을 가지므로 metric t를 복원하지 않는다.
    입력 F의 잡음을 숨기지 않기 위해 여기서는 특이값을 보정하지 않는다.
    """
    F = _matrix(F, (3, 3), 'F')
    if np.linalg.norm(F) == 0:
        raise ValueError('F는 0 행렬일 수 없습니다.')
    return validate_intrinsics(K_right).T @ F @ validate_intrinsics(K_left)


def enforce_essential_constraints(E):
    """E의 특이값을 (s,s,0)으로 투영. s=(s1+s2)/2.

    잡음으로 rank-2/동일 특이값 조건이 깨졌을 때 사용하는 별도 결과다.
    원래 E_raw를 대체하지 않으며 pose를 추정하는 함수도 아니다.
    """
    E = _matrix(E, (3, 3), 'E')
    u, singular, vt = np.linalg.svd(E)
    if singular[1] <= singular[0] * 1e-12 or singular[0] == 0:
        raise ValueError('Essential matrix가 퇴화했습니다.')
    s = (singular[0] + singular[1]) / 2
    return u @ np.diag([s, s, 0]) @ vt


def build_projection_matrix(K, R, t):
    """P=K[R|t] (3×4). x_h=P@[X,Y,Z,1], pixel=x_h[:2]/x_h[2].

    t는 오른쪽 카메라 중심이 아니다. 중심은 C=-R.T@t이다.
    왼쪽 카메라는 R=I,t=0을 넣어 P_left=K_left[I|0]을 만든다.
    """
    K = validate_intrinsics(K)
    R, t = validate_pose(R, t)
    return K @ np.column_stack((R, t))


def load_calibration(path, left_shape=None, right_shape=None):
    """필수 K_left,K_right,R,t; image_size_*는 선택 [width,height].

    distortion 보정은 수행하지 않으므로 보정된 pinhole 이미지와 그 K를
    입력한다. 크기가 명시되면 이미지와 정확히 일치하는지 확인한다.
    """
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('calibration JSON은 object여야 합니다.')
    if data.get('image_state') == 'raw':
        raise ValueError('raw_camera_model.json은 복원 입력이 아닙니다. undistort_stereo.py로 보정한 이미지와 calibration.json을 사용하세요.')
    missing = {'K_left', 'K_right', 'R', 't'} - data.keys()
    if missing:
        raise ValueError(f'calibration 필수 항목 누락: {sorted(missing)}')
    kl, kr = validate_intrinsics(data['K_left']), validate_intrinsics(data['K_right'])
    R, t = validate_pose(data['R'], data['t'])
    if np.linalg.norm(t) <= 1e-12:
        raise ValueError('stereo baseline은 0보다 커야 합니다.')
    for key, shape in [('image_size_left', left_shape), ('image_size_right', right_shape)]:
        if key in data:
            size = np.asarray(data[key])
            if size.shape != (2,) or not np.issubdtype(size.dtype, np.integer) or (size <= 0).any():
                raise ValueError(f'{key}는 양의 정수 [width,height]여야 합니다.')
            if shape is not None and tuple(size) != (shape[1], shape[0]):
                raise ValueError(f'{key}가 입력 이미지 해상도와 다릅니다. 이미지 크기에 맞는 K를 사용하세요.')
    return kl, kr, R, t, data.get('units', 'unspecified')
