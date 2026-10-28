"""Phase 5: 제거 사유와 입력 순서를 보존하는 stereo 품질 필터."""
import numpy as np
from point_cloud import validate_3d_points, image_point_mask
from reprojection import project_points, compute_reprojection_error


def filter_reconstructed_points(xyz, points_left, points_right, P_left, P_right, R, t,
                                numerical_valid=None, left_shape=None, right_shape=None,
                                min_depth=1e-6, max_depth=1000., max_distance=1000.,
                                reprojection_threshold=2.):
    """각 단계 누적 mask와 원소별 진단 반환. 배열 순서/길이는 변경하지 않는다.

    양쪽 카메라 각각 error<=threshold를 요구한다. 평균만 사용하면 한쪽의
    큰 오차가 가려질 수 있으므로 평균은 진단값으로만 저장한다.
    제거 수는 순차 배타적이며 각 수의 합=전체 제거 수가 된다.
    """
    xyz = np.asarray(xyz, float)
    left, right = np.asarray(points_left, float), np.asarray(points_right, float)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or left.shape != (len(xyz), 2) or right.shape != left.shape:
        raise ValueError('XYZ N×3 및 대응점 N×2의 행 수가 같아야 합니다.')
    if not np.isfinite(reprojection_threshold) or reprojection_threshold <= 0:
        raise ValueError('reprojection_threshold는 유한한 양수여야 합니다.')
    geometry_valid, details = validate_3d_points(xyz, R, t, min_depth, max_depth, max_distance, True)
    numerical = details['finite_3d'].copy()
    if numerical_valid is not None:
        supplied = np.asarray(numerical_valid)
        if supplied.shape != (len(xyz),) or supplied.dtype != np.bool_:
            raise ValueError('numerical_valid는 N개 bool이어야 합니다.')
        numerical &= supplied
    in_image = np.isfinite(left).all(axis=1) & np.isfinite(right).all(axis=1)
    if left_shape is not None:
        in_image &= image_point_mask(left, left_shape)
    if right_shape is not None:
        in_image &= image_point_mask(right, right_shape)
    projected_left = project_points(P_left, xyz)
    projected_right = project_points(P_right, xyz)
    error_left = compute_reprojection_error(left, projected_left)
    error_right = compute_reprojection_error(right, projected_right)
    reprojection_valid = (error_left <= reprojection_threshold) & (error_right <= reprojection_threshold)
    after_depth = numerical & details['depth_valid']
    after_distance = after_depth & details['distance_valid']
    after_image = after_distance & in_image
    final = after_image & reprojection_valid
    counts = dict(numerical=int((~numerical).sum()),
                  depth=int((numerical & ~after_depth).sum()),
                  distance=int((after_depth & ~after_distance).sum()),
                  image_bounds=int((after_distance & ~after_image).sum()),
                  reprojection=int((after_image & ~final).sum()))
    return dict(details, numerical_valid=numerical, geometry_valid=geometry_valid,
                image_valid=in_image, reprojection_valid=reprojection_valid, final_mask=final,
                projected_left=projected_left, projected_right=projected_right,
                error_left=error_left, error_right=error_right,
                error_mean=error_left / 2 + error_right / 2, removal_counts=counts)
