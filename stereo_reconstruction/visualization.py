import cv2
import numpy as np


def side_by_side(left, right):
    canvas = np.zeros((max(left.shape[0], right.shape[0]), left.shape[1] + right.shape[1], 3), np.uint8)
    canvas[:left.shape[0], :left.shape[1]] = left
    canvas[:right.shape[0], left.shape[1]:] = right
    return canvas


def draw_features(left, right, keypoints_left, keypoints_right):
    return side_by_side(cv2.drawKeypoints(left, keypoints_left, None, color=(0, 255, 0)),
                        cv2.drawKeypoints(right, keypoints_right, None, color=(0, 255, 0)))


def draw_matches(left, right, keypoints_left, keypoints_right, matches, limit=100):
    return cv2.drawMatches(left, keypoints_left, right, keypoints_right, matches[:limit], None,
                           flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)


def line_segment_in_image(line, width, height):
    """au+bv+c=0과 이미지 네 경계의 교점을 계산. 수직선도 처리한다."""
    a, b, c = np.asarray(line, float)
    norm = np.hypot(a, b)
    if not np.isfinite([a, b, c]).all() or norm == 0:
        return None
    a, b, c = a / norm, b / norm, c / norm
    candidates = []
    if abs(b) > 1e-12:
        candidates.extend([(0, -c / b), (width - 1, -(a * (width - 1) + c) / b)])
    if abs(a) > 1e-12:
        candidates.extend([(-c / a, 0), (-(b * (height - 1) + c) / a, height - 1)])
    points = []
    for x, y in candidates:
        if -1e-7 <= x <= width - 1 + 1e-7 and -1e-7 <= y <= height - 1 + 1e-7:
            point = (int(round(x)), int(round(y)))
            if point not in points:
                points.append(point)
    return (points[0], points[1]) if len(points) >= 2 else None


def draw_epipolar_geometry(left, right, F, points_left, points_right, limit=20):
    from epipolar import epipolar_lines
    pl, pr = points_left[:limit], points_right[:limit]
    ll, lr = epipolar_lines(F, pl, pr)
    left, right = left.copy(), right.copy()
    rng = np.random.default_rng(42)
    for p, q, l, r in zip(pl, pr, ll, lr):
        color = tuple(int(c) for c in rng.integers(70, 256, 3))
        for image, point, line in ((left, p, l), (right, q, r)):
            segment = line_segment_in_image(line, image.shape[1], image.shape[0])
            if segment:
                cv2.line(image, *segment, color, 1, cv2.LINE_AA)
            cv2.circle(image, tuple(np.rint(point).astype(int)), 4, color, -1)
    return side_by_side(left, right)


def draw_reprojection(left, right, observed_left, observed_right, projected_left,
                      projected_right, valid_mask, limit=100):
    """초록 원=통과 관측점, 빨강 원=제거 관측점, 노랑 십자=재투영.

    유한하고 이미지 안에 있는 좌표만 그린다. 긴 outlier 좌표를 정수로
    변환하지 않아 overflow를 방지한다. 통과/제거 점을 최대 절반씩 표시.
    """
    from point_cloud import image_point_mask
    valid_mask = np.asarray(valid_mask, bool)
    rejected = np.flatnonzero(~valid_mask)[:limit // 2]
    accepted = np.flatnonzero(valid_mask)[:limit - len(rejected)]
    indices = np.concatenate((rejected, accepted))
    canvases = []
    for image, observed, projected in ((left, observed_left, projected_left),
                                        (right, observed_right, projected_right)):
        canvas = image.copy()
        observed_valid = image_point_mask(observed, image.shape)
        projected_valid = image_point_mask(projected, image.shape)
        for i in indices:
            color = (0, 220, 0) if valid_mask[i] else (0, 0, 255)
            if observed_valid[i]:
                a = tuple(np.rint(observed[i]).astype(int))
                cv2.circle(canvas, a, 4, color, 1, cv2.LINE_AA)
            if projected_valid[i]:
                b = tuple(np.rint(projected[i]).astype(int))
                cv2.drawMarker(canvas, b, (0, 255, 255), cv2.MARKER_CROSS, 7, 1)
                if observed_valid[i]:
                    cv2.line(canvas, a, b, color, 1, cv2.LINE_AA)
        cv2.putText(canvas, 'Green: kept | Red: rejected | Yellow +: projected',
                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, .45, (255,255,255), 1, cv2.LINE_AA)
        canvases.append(canvas)
    return side_by_side(*canvases)
