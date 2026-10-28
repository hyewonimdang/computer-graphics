"""NumPy로 직접 구현한 8방향 SGM, 경계 적응 P2, 신뢰도 필터.

Hirschmüller의 경로 점화식을 사용하되 비용은 Census24 + clipped AD다.
OpenCV는 BGR→gray 변환만 한다. 외부 stereo/최적화/보간 함수를 쓰지 않는다.
출력 confidence는 확률이 아닌 0..1 휴리스틱 점수. 구멍을 채우지 않는다.
"""
import cv2
import numpy as np
from dense_stereo import census5, box_sum

DIRECTIONS = ((0, 1), (0, -1), (1, 0), (-1, 0),
              (1, 1), (1, -1), (-1, 1), (-1, -1))
INVALID = 1000.0


def path_step(cost, previous, p1, p2):
    """D×N 독립 경로의 한 step. min을 빼 긴 경로에서도 수치 범위 유지."""
    minimum = previous.min(axis=0)
    candidate = np.minimum(previous, minimum[None] + p2)
    candidate[1:] = np.minimum(candidate[1:], previous[:-1] + p1)
    candidate[:-1] = np.minimum(candidate[:-1], previous[1:] + p1)
    return cost + candidate - minimum[None]


def aggregate_sgm(cost, guide, p1=2., p2=24., edge_scale=10., directions=DIRECTIONS):
    """8방향 합의 평균. 한 방향의 모든 scanline을 NumPy로 동시에 계산.

    P2(p,q)=max(P1+epsilon,P2/(1+|I(p)-I(q)|/edge_scale)).
    영상 경계에서 더 싼 깊이 점프를 허용한다. edge_scale=None이면 고정 P2.
    전체 path volume 대신 직전 D×W 또는 D×H만 유지한다.
    """
    if cost.ndim != 3 or guide.shape != cost.shape[1:] or not np.isfinite(cost).all():
        raise ValueError('유한한 D×H×W 비용과 H×W guide가 필요합니다.')
    if not np.isfinite([p1, p2]).all() or not 0 < p1 < p2:
        raise ValueError('0 < P1 < P2 이어야 합니다.')
    if edge_scale is not None and (not np.isfinite(edge_scale) or edge_scale <= 0):
        raise ValueError('edge_scale은 양수 또는 None이어야 합니다.')
    if not directions or any(d not in DIRECTIONS for d in directions):
        raise ValueError('지원하지 않는 경로 방향입니다.')
    _, h, w = cost.shape
    total = np.zeros_like(cost, dtype=np.float32)
    guide = np.asarray(guide, np.float32)
    for dy, dx in directions:
        if dy == 0:
            previous = None
            for x in (range(w) if dx > 0 else range(w-1, -1, -1)):
                current = cost[:, :, x]
                if previous is None:
                    value = current.copy()
                else:
                    gradient = np.abs(guide[:, x] - guide[:, x-dx])
                    penalty = p2 if edge_scale is None else np.maximum(p1+.001, p2/(1+gradient/edge_scale))
                    value = path_step(current, previous, p1, penalty)
                total[:, :, x] += value
                previous = value
        else:
            previous = None
            for y in (range(h) if dy > 0 else range(h-1, -1, -1)):
                current = cost[:, y, :]
                value = current.copy()
                if previous is not None:
                    # 행 끝을 wrap하지 않는다. 새 diagonal의 첫 pixel은 C 자체다.
                    target = slice(max(dx, 0), min(w, w+dx))
                    source = slice(max(-dx, 0), min(w, w-dx))
                    gradient = np.abs(guide[y, target]-guide[y-dy, source])
                    penalty = p2 if edge_scale is None else np.maximum(p1+.001, p2/(1+gradient/edge_scale))
                    value[:, target] = path_step(current[:, target], previous[:, source], p1, penalty)
                total[:, y, :] += value
                previous = value
    return total / len(directions)


def guided_cost(raw, left, right, valid, sigma=12.):
    """3×3 양측 영상 경계 가중 평균. 양쪽 중 큰 밝기차를 weight에 사용.

    w(p,q)=exp(-max(|L(p)-L(q)|,|R(p-d)-R(q-d)|)/sigma).
    중심 자체는 weight1. 잘못된 영역은 support로 쓰지 않는다.
    시차를 평균하는 사후 필터가 아니라 후보 비용의 작은 지지 영역이다.
    """
    h,w=raw.shape
    numerator=np.where(valid,raw,0).astype(np.float32)
    denominator=valid.astype(np.float32)
    pr,pv=np.pad(raw,1),np.pad(valid,1)
    pl,pp=np.pad(left.astype(np.float32),1),np.pad(right.astype(np.float32),1)
    for dy,dx in DIRECTIONS:
        ss=(slice(1+dy,1+dy+h),slice(1+dx,1+dx+w))
        delta=np.maximum(np.abs(pl[ss]-left),np.abs(pp[ss]-right))
        weight=np.exp(-delta/sigma)*pv[ss]
        numerator+=weight*pr[ss]; denominator+=weight
    return np.where(valid,numerator/np.maximum(denominator,1e-6),INVALID)


def census_cost_volume(a, b, vl, vr, min_disparity, max_disparity):
    """5×5 Census + 밝기차 + 경계 보존3×3 비용 집계."""
    h, w = a.shape
    ca, cb = census5(a), census5(b)
    lut = np.array([bin(i).count('1') for i in range(256)], np.uint8)
    vl = box_sum(vl, 2) == 25
    vr = box_sum(vr, 2) == 25
    volume = np.full((max_disparity-min_disparity+1, h, w), INVALID, np.float32)
    for index, d in enumerate(range(min_disparity, max_disparity+1)):
        end = w-d
        xor = ca[:, d:] ^ cb[:, :end]
        ham = lut[xor & 255] + lut[(xor >> 8) & 255] + lut[(xor >> 16) & 255]
        ad = np.minimum(np.abs(a[:, d:].astype(np.float32)-b[:, :end]), 30)/15
        volume[index, :, d:] = guided_cost(ham+ad,a[:,d:],b[:,:end],vl[:,d:]&vr[:,:end])
    return volume


def select_disparity(aggregated, raw, minimum):
    index = aggregated.argmin(axis=0)
    best = np.take_along_axis(aggregated, index[None], axis=0)[0]
    second = np.full(best.shape, np.inf, np.float32)
    for i in range(len(aggregated)):
        second = np.minimum(second, np.where(np.abs(index-i) > 1, aggregated[i], np.inf))
    margin = np.clip((second-best)/np.maximum(second, 1.), 0, 1)
    lo = np.take_along_axis(aggregated, np.maximum(index-1, 0)[None], axis=0)[0]
    hi = np.take_along_axis(aggregated, np.minimum(index+1, len(aggregated)-1)[None], axis=0)[0]
    denom = lo+hi-2*best
    offset = np.zeros(best.shape, np.float32)
    np.divide(.5*(lo-hi), denom, out=offset, where=denom > 1e-6)
    disp = minimum+index.astype(np.float32)+np.clip(offset, -.5, .5)
    photo = np.take_along_axis(raw, index[None], axis=0)[0]
    # 경계 minimum은 search range가 잘렸을 수 있으므로 거절한다.
    eligible = (index > 0) & (index < len(aggregated)-1) & (photo < INVALID)
    return disp, margin, photo, eligible


def confidence_filter(dl, dr, ml, mr, photo, gl, gr, guide,
                      lr_threshold=1., uniqueness=.04, max_cost=16.,
                      min_confidence=.35, support=2):
    """독립 오른쪽 SGM winner와 확인; 임의 보간 없이 작은 고립점을 제거.

    confidence = margin/(margin+.08) * exp(-LR) * exp(-photo/24).
    낮은 texture 자체를 거절하지 않아 경로 정보가 지지하는 영역을 유지한다.
    """
    h, w = dl.shape
    yy, xx = np.indices((h, w))
    rx = np.rint(xx-dl).astype(int)
    inside = (rx >= 0) & (rx < w)
    rr = np.clip(rx, 0, w-1)
    lr = np.abs(dl-dr[yy, rr])
    margin = np.minimum(ml, mr[yy, rr])
    confidence = margin/(margin+.08)*np.exp(-lr)*np.exp(-photo/24.)
    valid = gl & inside & gr[yy, rr] & (lr <= lr_threshold)
    valid &= (margin >= uniqueness) & (photo <= max_cost) & (confidence >= min_confidence)
    before_support = int(valid.sum())
    # 색 경계를 넘어선 이웃은 지지 판단에 쓰지 않는다. 평균으로 깊이를 섞지 않음.
    for _ in range(2):
        counts = np.zeros((h, w), np.uint8)
        pd, pv, pg = np.pad(dl, 1), np.pad(valid, 1), np.pad(guide.astype(float), 1)
        for dy, dx in DIRECTIONS:
            ss = (slice(1+dy, 1+dy+h), slice(1+dx, 1+dx+w))
            counts += pv[ss] & (np.abs(pd[ss]-dl) < 2.) & (np.abs(pg[ss]-guide) < 30.)
        valid &= counts >= support
    confidence[~valid] = 0
    return valid, confidence.astype(np.float32), lr, before_support


def sgm_correspondences(left, right, valid_left=None, valid_right=None,
                        min_disparity=1, max_disparity=160, p1=2., p2=24.,
                        edge_scale=10., uniqueness=.04, lr_threshold=1.,
                        max_cost=16., min_confidence=.35, support=2):
    if left.shape != right.shape or left.ndim != 3 or left.shape[2] != 3 or left.dtype != np.uint8 or right.dtype != np.uint8:
        raise ValueError('같은 크기의 uint8 BGR stereo 이미지가 필요합니다.')
    h, w = left.shape[:2]
    if not isinstance(min_disparity, (int, np.integer)) or not isinstance(max_disparity, (int, np.integer)) or not 0 <= min_disparity < max_disparity < w or max_disparity-min_disparity < 3:
        raise ValueError('시차 범위는 0<=min<max<width인 정수이며 최소 4개 후보가 필요합니다.')
    if min(h, w) < 5 or (max_disparity-min_disparity+1)*h*w > 60_000_000:
        raise ValueError('영상이 너무 작거나 비용 volume이 너무 큽니다 (최대 60M entries).')
    if not np.isfinite([uniqueness, lr_threshold, max_cost, min_confidence]).all() or not 0 <= uniqueness < 1 or lr_threshold < 0 or max_cost <= 0 or not 0 <= min_confidence <= 1 or not isinstance(support, int) or not 0 <= support <= 8:
        raise ValueError('매칭 threshold 범위를 확인하세요.')
    vl = np.ones((h, w), bool) if valid_left is None else np.asarray(valid_left, bool)
    vr = np.ones((h, w), bool) if valid_right is None else np.asarray(valid_right, bool)
    if vl.shape != (h, w) or vr.shape != (h, w):
        raise ValueError('valid mask shape 불일치')
    a, b = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY), cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    raw = census_cost_volume(a, b, vl, vr, min_disparity, max_disparity)
    agg = aggregate_sgm(raw, a, p1, p2, edge_scale)
    dl, ml, photo, gl = select_disparity(agg, raw, min_disparity)
    del agg
    # 오른쪽 reference 좌표로 raw 비용을 옮긴 뒤 독립적으로 경로 누적한다.
    for index, d in enumerate(range(min_disparity, max_disparity+1)):
        raw[index, :, :w-d] = raw[index, :, d:].copy()
        if d:
            raw[index, :, w-d:] = INVALID
    agg = aggregate_sgm(raw, b, p1, p2, edge_scale)
    dr, mr, _, gr = select_disparity(agg, raw, min_disparity)
    del agg, raw
    valid, confidence, lr, count = confidence_filter(dl, dr, ml, mr, photo, gl, gr, a,
        lr_threshold, uniqueness, max_cost, min_confidence, support)
    return dict(disparity=dl, valid=valid, confidence=confidence, best_cost=photo,
                left_right_error=lr, before_support=count, uniqueness_margin=ml,
                right_disparity=dr)
