"""Phase 1–6: calibrated stereo geometry와 직접 구현 DLT CLI."""
import argparse
import json
from pathlib import Path
import numpy as np
from camera import (load_calibration, compute_essential_matrix, enforce_essential_constraints,
                    build_projection_matrix, skew_symmetric)
from image_io import load_stereo_images, save_image
from features import detect_features, match_features, matched_points
from visualization import draw_features, draw_matches
from visualization import draw_epipolar_geometry
from epipolar import estimate_fundamental_matrix, compute_epipolar_errors, compute_epipolar_distances
from triangulation import triangulate_points_dlt
from filtering import filter_reconstructed_points
from point_cloud import assign_colors
from exporter import export_json, export_ply, export_csv
from visualization import draw_reprojection


def run_phase1(left_path, right_path, output, nfeatures=4000, ratio=0.75, max_distance=64):
    left, right = load_stereo_images(left_path, right_path)
    kl, dl = detect_features(left, nfeatures)
    kr, dr = detect_features(right, nfeatures)
    raw, filtered = match_features(dl, dr, ratio, max_distance)
    output = Path(output)
    save_image(output / '01_detected_features.jpg', draw_features(left, right, kl, kr))
    save_image(output / '02_raw_matches.jpg', draw_matches(left, right, kl, kr, raw))
    save_image(output / '03_filtered_matches.jpg', draw_matches(left, right, kl, kr, filtered))
    pl, pr = matched_points(kl, kr, filtered)
    print(f'Left features: {len(kl)}\nRight features: {len(kr)}')
    print(f'Raw matches: {len(raw)}\nFiltered matches: {len(filtered)}')
    return left, right, pl, pr, dict(left_features=len(kl), right_features=len(kr),
                                    raw_matches=len(raw), filtered_matches=len(filtered))


def run_phase2(left, right, pl, pr, output, stats, ransac_threshold=1.5, epipolar_threshold=2.0, seed=42):
    if epipolar_threshold <= 0:
        raise ValueError('epipolar_threshold는 양수여야 합니다.')
    F, ransac = estimate_fundamental_matrix(pl, pr, ransac_threshold, seed=seed)
    errors = compute_epipolar_errors(F, pl, pr)
    dl, dr = compute_epipolar_distances(F, pl, pr)
    # 양쪽 카메라 모두 threshold 이내여야 후속 triangulation에 전달한다.
    accepted = ransac & (dl <= epipolar_threshold) & (dr <= epipolar_threshold)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    np.savez(output / 'geometry_data.npz', F=F, points_left=pl, points_right=pr,
             ransac_mask=ransac, epipolar_mask=accepted, algebraic_errors=errors,
             distances_left=dl, distances_right=dr,
             accepted_left=pl[accepted], accepted_right=pr[accepted])
    save_image(output / '04_epipolar_geometry.jpg',
               draw_epipolar_geometry(left, right, F, pl[accepted], pr[accepted]))
    stats = dict(stats, phase=2, ransac_inliers=int(ransac.sum()), epipolar_inliers=int(accepted.sum()),
                 ransac_threshold_px=ransac_threshold, epipolar_threshold_px=epipolar_threshold, seed=seed,
                 mean_absolute_epipolar_error=float(np.abs(errors[accepted]).mean()) if accepted.any() else None,
                 mean_left_line_distance_px=float(dl[accepted].mean()) if accepted.any() else None,
                 mean_right_line_distance_px=float(dr[accepted].mean()) if accepted.any() else None,
                 F=F.tolist())
    (output / 'diagnostics.json').write_text(json.dumps(stats, indent=2, allow_nan=False), encoding='utf-8')
    print('Fundamental matrix:\n', F)
    print(f'RANSAC inliers: {ransac.sum()}\nEpipolar inliers: {accepted.sum()}')
    print(f'Mean absolute epipolar error: {stats["mean_absolute_epipolar_error"]}')
    print(f'Mean left/right line distance: {stats["mean_left_line_distance_px"]} / '
          f'{stats["mean_right_line_distance_px"]} px')
    if accepted.sum() < 8:
        raise ValueError('에피폴라 검사를 통과한 점이 8개 미만입니다. 진단 결과를 확인하세요.')
    return F, accepted


def run_phase3(F, calibration, output):
    kl, kr, R, t, units = calibration
    E_raw = compute_essential_matrix(F, kl, kr)
    E = enforce_essential_constraints(E_raw)
    E_pose = skew_symmetric(t) @ R
    P1 = build_projection_matrix(kl, np.eye(3), np.zeros(3))
    P2 = build_projection_matrix(kr, R, t)
    # 임의의 부호·배율을 제거한 E와 calibration pose의 일치도 진단.
    a, b = E / np.linalg.norm(E), E_pose / np.linalg.norm(E_pose)
    mismatch = float(min(np.linalg.norm(a-b), np.linalg.norm(a+b)))
    np.savez(Path(output) / 'camera_data.npz', K_left=kl, K_right=kr, R=R, t=t,
             E_raw=E_raw, E_constrained=E, E_from_pose=E_pose, P_left=P1, P_right=P2)
    path = Path(output) / 'diagnostics.json'
    stats = json.loads(path.read_text())
    stats.update(phase=3, units=units, baseline=float(np.linalg.norm(t)),
                 essential_pose_mismatch=mismatch,
                 E_raw_singular_values=np.linalg.svd(E_raw, compute_uv=False).tolist())
    path.write_text(json.dumps(stats, indent=2, allow_nan=False), encoding='utf-8')
    print('Essential matrix (raw):\n', E_raw)
    print('Projection matrix left:\n', P1)
    print('Projection matrix right:\n', P2)
    print(f'Calibration baseline: {np.linalg.norm(t)} {units}; E pose mismatch: {mismatch}')
    if mismatch > 0.1:
        print('주의: 추정 E와 보정 pose가 다릅니다. 이미지 순서/K/R/t 및 대응점을 확인하세요.')
    return P1, P2


def run_phase4(pl, pr, accepted, P1, P2, output, compare_opencv=False):
    points1, points2 = pl[accepted], pr[accepted]
    xyz, details = triangulate_points_dlt(points1, points2, P1, P2, return_details=True)
    valid = details['numerical_valid']
    output = Path(output)
    # 입력 feature-filtered 대응점까지 역추적 가능한 index를 보존한다.
    np.savez(output / 'triangulation_data.npz', points_3d=xyz,
             points_left=points1, points_right=points2, source_indices=np.flatnonzero(accepted),
             **details)
    path = output / 'diagnostics.json'
    stats = json.loads(path.read_text())
    stats.update(phase=4, triangulated_points=len(xyz), numerically_valid_points=int(valid.sum()),
                 numerically_invalid_points=int((~valid).sum()), opencv_comparison=None)
    if compare_opencv:
        from validation import compare_with_opencv
        comparison = compare_with_opencv(points1, points2, P1, P2, xyz)
        np.savez(output / 'opencv_comparison.npz', **{k: comparison[k] for k in
                 ['opencv_xyz', 'comparable_mask', 'differences']})
        stats['opencv_comparison'] = {k: comparison[k] for k in
                                      ['compared_count', 'mean_difference', 'max_difference']}
        print('OpenCV comparison:', stats['opencv_comparison'])
    path.write_text(json.dumps(stats, indent=2, allow_nan=False), encoding='utf-8')
    print(f'Triangulated points: {len(xyz)}\nNumerically valid: {valid.sum()}\n'
          f'Numerically invalid (NaN rows): {(~valid).sum()}')
    print('Phase 4 결과: 양의 깊이/재투영 오차 필터는 아직 적용하지 않았습니다.')
    if not valid.any():
        raise ValueError('수치적으로 유효한 DLT 결과가 없습니다. baseline/시차를 확인하세요.')
    return xyz


def run_phase5(left, right, P1, P2, calibration, output, min_depth=1e-6,
               max_depth=1000., max_distance=1000., reprojection_threshold=2.):
    output = Path(output)
    _, _, R, t, _ = calibration
    with np.load(output / 'triangulation_data.npz') as data:
        xyz, pl, pr = data['points_3d'], data['points_left'], data['points_right']
        indices, numerical = data['source_indices'], data['numerical_valid']
    result = filter_reconstructed_points(xyz, pl, pr, P1, P2, R, t, numerical,
               left.shape, right.shape, min_depth, max_depth, max_distance, reprojection_threshold)
    mask = result['final_mask']
    np.savez(output / 'reprojection_data.npz',
             **{key: value for key, value in result.items() if key != 'removal_counts'},
             source_indices=indices, points_3d=xyz, points_left=pl, points_right=pr,
             accepted_source_indices=indices[mask], accepted_points_3d=xyz[mask],
             accepted_left=pl[mask], accepted_right=pr[mask])
    save_image(output / '05_reprojection.jpg', draw_reprojection(left, right, pl, pr,
               result['projected_left'], result['projected_right'], mask))
    path = output / 'diagnostics.json'
    stats = json.loads(path.read_text())
    stats.update(phase=5, final_valid_3d_points=int(mask.sum()),
                 invalid_points_removed=int((~mask).sum()), removal_counts=result['removal_counts'],
                 filter_limits=dict(min_depth=min_depth, max_depth=max_depth,
                                    max_distance=max_distance, reprojection_threshold_px=reprojection_threshold),
                 mean_left_reprojection_error_px=float(result['error_left'][mask].mean()) if mask.any() else None,
                 mean_right_reprojection_error_px=float(result['error_right'][mask].mean()) if mask.any() else None)
    path.write_text(json.dumps(stats, indent=2, allow_nan=False), encoding='utf-8')
    print(f'Invalid points removed: {(~mask).sum()} | Reasons: {result["removal_counts"]}')
    print(f'Mean left reprojection error: {stats["mean_left_reprojection_error_px"]} px')
    print(f'Mean right reprojection error: {stats["mean_right_reprojection_error_px"]} px')
    print(f'Final valid 3D points: {mask.sum()}')
    if not mask.any():
        print('주의: 모든 점이 제거되었습니다. 보정값/이미지 순서/필터 한계를 확인하세요.')
    return xyz[mask], pl[mask], indices[mask]


def run_phase6(xyz, left_image, observed_left, source_indices, units, output, csv=False):
    cloud = assign_colors(xyz, left_image, observed_left)
    output = Path(output)
    export_ply(cloud, output / '06_point_cloud.ply')
    export_json(cloud, output / '07_point_cloud.json', units=units)
    np.save(output / 'point_cloud.npy', cloud)
    np.save(output / 'point_cloud_source_indices.npy', source_indices)
    if csv:
        export_csv(cloud, output / '08_point_cloud.csv')
    path = output / 'diagnostics.json'
    stats = json.loads(path.read_text())
    stats.update(phase=6, exported_points=len(cloud), point_cloud_shape=list(cloud.shape),
                 color_sampling='left observed pixel; nearest floor(u+0.5), floor(v+0.5); BGR to RGB')
    path.write_text(json.dumps(stats, indent=2, allow_nan=False), encoding='utf-8')
    print(f'Exported XYZRGB: {cloud.shape}; JSON / PLY / NumPy' + (' / CSV' if csv else ''))
    return cloud


def main():
    parser = argparse.ArgumentParser(description='Stereo reconstruction: Phase 1–6')
    parser.add_argument('--left', required=True)
    parser.add_argument('--right', required=True)
    parser.add_argument('--output', default='results')
    parser.add_argument('--phase', type=int, choices=[1, 2, 3, 4, 5, 6], default=2)
    parser.add_argument('--calibration', help='Phase 3–6 필수: K_left/K_right/R/t JSON')
    parser.add_argument('--compare-opencv', action='store_true', help='Phase 4 결과를 OpenCV와 검증용 비교')
    parser.add_argument('--csv', action='store_true', help='Phase 6: CSV도 저장')
    parser.add_argument('--nfeatures', type=int, default=4000)
    parser.add_argument('--ratio', type=float, default=0.75)
    parser.add_argument('--max-distance', type=float, default=64)
    parser.add_argument('--ransac-threshold', type=float, default=1.5)
    parser.add_argument('--epipolar-threshold', type=float, default=2.0)
    parser.add_argument('--min-depth', type=float, default=1e-6)
    parser.add_argument('--max-depth', type=float, default=1000.)
    parser.add_argument('--max-3d-distance', type=float, default=1000.)
    parser.add_argument('--reprojection-threshold', type=float, default=2.)
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()
    if args.phase >= 3 and not args.calibration:
        parser.error('Phase 3–6는 --calibration 파일이 필요합니다.')
    if args.compare_opencv and args.phase < 4:
        parser.error('--compare-opencv는 --phase 4 이상에서만 사용합니다.')
    if args.csv and args.phase != 6:
        parser.error('--csv는 --phase 6에서만 사용합니다.')
    try:
        if args.phase >= 3:
            load_calibration(args.calibration)
        left, right, pl, pr, stats = run_phase1(args.left, args.right, args.output,
                                               args.nfeatures, args.ratio, args.max_distance)
        if args.phase >= 2:
            F, accepted = run_phase2(left, right, pl, pr, args.output, stats,
                       args.ransac_threshold, args.epipolar_threshold, args.seed)
        if args.phase >= 3:
            calibration = load_calibration(args.calibration, left.shape, right.shape)
            P1, P2 = run_phase3(F, calibration, args.output)
        if args.phase >= 4:
            run_phase4(pl, pr, accepted, P1, P2, args.output, args.compare_opencv)
        if args.phase >= 5:
            xyz, observed_left, source_indices = run_phase5(left, right, P1, P2, calibration, args.output,
                       args.min_depth, args.max_depth, args.max_3d_distance, args.reprojection_threshold)
        if args.phase == 6:
            run_phase6(xyz, left, observed_left, source_indices, calibration[4], args.output, args.csv)
    except (ValueError, OSError) as error:
        parser.exit(2, f'오류: {error}\n')


if __name__ == '__main__':
    main()
