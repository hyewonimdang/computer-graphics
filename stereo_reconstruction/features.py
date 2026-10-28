"""검출기는 독립적으로 교체 가능; 현재 매칭 거리는 ORB 전용 Hamming이다."""
import cv2
import numpy as np


def detect_features(image, nfeatures=4000):
    if nfeatures < 8:
        raise ValueError("nfeatures는 8 이상이어야 합니다.")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return cv2.ORB_create(nfeatures=nfeatures).detectAndCompute(gray, None)


def match_features(desc_left, desc_right, ratio=0.75, max_distance=64):
    """(raw, filtered) 반환. k=2 ratio 검사 후 역방향 최근접 일치도 요구.

    ORB 기술자는 이진 벡터이므로 Hamming distance(다른 bit 수)를 쓴다.
    d1 < ratio*d2: 최선 후보가 차선 후보보다 충분히 구별될 때만 채택.
    양방향 최근접 검사는 서로 다른 왼쪽 점의 동일 오른쪽 점 중복을 막는다.
    raw는 각 왼쪽 기술자의 최근접 후보이며 필터링 이전 시각화에 사용한다.
    """
    if not 0 < ratio < 1 or max_distance <= 0:
        raise ValueError("ratio는 (0,1), max_distance는 양수여야 합니다.")
    if desc_left is None or desc_right is None or min(len(desc_left), len(desc_right)) == 0:
        return [], []
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    pairs = matcher.knnMatch(desc_left, desc_right, k=min(2, len(desc_right)))
    raw = [pair[0] for pair in pairs if pair]
    reverse = {m.queryIdx: m.trainIdx for m in matcher.match(desc_right, desc_left)}
    filtered = [pair[0] for pair in pairs if len(pair) == 2
                and pair[0].distance < ratio * pair[1].distance
                and pair[0].distance <= max_distance
                and reverse.get(pair[0].trainIdx) == pair[0].queryIdx]
    return sorted(raw, key=lambda m: m.distance), sorted(filtered, key=lambda m: m.distance)


def matched_points(keypoints_left, keypoints_right, matches):
    left = np.array([keypoints_left[m.queryIdx].pt for m in matches], dtype=np.float64).reshape(-1, 2)
    right = np.array([keypoints_right[m.trainIdx].pt for m in matches], dtype=np.float64).reshape(-1, 2)
    return left, right
