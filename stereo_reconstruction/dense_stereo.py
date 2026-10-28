"""직접 구현한 calibrated rectification + Census window stereo matching.

OpenCV는 remap/색 변환/이미지 저장에만 사용한다. stereoRectify,
StereoBM/SGBM, reprojectImageTo3D, triangulatePoints는 호출하지 않는다.
"""
import numpy as np
import cv2
from camera import validate_intrinsics,validate_pose


def rectification_geometry(K_left,K_right,R,t,size):
    """왼쪽 기준 카메라 중심 C=-R.T t를 공통 X축으로 정한다.

    Z축의 초기 방향은 두 optical axis의 평균. Y=normalize(Z_hint×X),
    Z=X×Y로 우수직교 좌표계를 구성한다. Q_left는 world→rectified 회전,
    Q_right=Q_left R.T이다. 공통 새 K에서 epipolar line이 수평이 된다.
    """
    kl,kr=validate_intrinsics(K_left),validate_intrinsics(K_right)
    R,t=validate_pose(R,t)
    center=-R.T@t; baseline=np.linalg.norm(center)
    if baseline<1e-12: raise ValueError('0 baseline은 정렬할 수 없습니다.')
    x=center/baseline; hint=np.array([0.,0,1])+R.T@np.array([0.,0,1])
    y=np.cross(hint,x)
    if np.linalg.norm(y)<1e-8: raise ValueError('전방 이동에 가까운 rig는 수평 dense stereo 모드에 적합하지 않습니다.')
    y/=np.linalg.norm(y); z=np.cross(x,y)
    ql=np.stack([x,y,z]); qr=ql@R.T
    w,h=size; f=float((kl[0,0]+kl[1,1]+kr[0,0]+kr[1,1])/4)
    kn=np.array([[f,0,(w-1)/2],[0,f,(h-1)/2],[0,0,1.]])
    return kn,ql,qr,baseline


def rectify_image(image,K,Q,K_rect,size):
    """출력 ray를 원본 pinhole image로 역매핑: K Q.T K_rect^-1 x.

    입력은 이미 렌즈 왜곡이 제거된 이미지여야 한다. crop/resize 없음.
    유효 mask로 원본 밖 및 뒤를 바라보는 ray를 배제한다.
    """
    w,h=size
    u,v=np.meshgrid(np.arange(w),np.arange(h))
    rays=np.stack([u,v,np.ones_like(u)],axis=-1)@(np.asarray(K)@Q.T@np.linalg.inv(K_rect)).T
    with np.errstate(divide='ignore',invalid='ignore'):
        mx=rays[:,:,0]/rays[:,:,2]; my=rays[:,:,1]/rays[:,:,2]
    valid=(rays[:,:,2]>1e-12)&np.isfinite(mx)&np.isfinite(my)&(mx>=0)&(my>=0)&(mx<=image.shape[1]-1)&(my<=image.shape[0]-1)
    mx=np.where(valid,mx,-1).astype(np.float32); my=np.where(valid,my,-1).astype(np.float32)
    return cv2.remap(image,mx,my,cv2.INTER_LINEAR),valid,mx,my


def box_sum(image,radius):
    """직접 integral image로 (2r+1)² box sum. 경계 밖은 0 padding."""
    padded=np.pad(np.asarray(image,dtype=np.float64),radius)
    integral=np.pad(padded,((1,0),(1,0))).cumsum(0).cumsum(1)
    k=2*radius+1
    return integral[k:,k:]-integral[:-k,k:]-integral[k:,:-k]+integral[:-k,:-k]


def census5(gray):
    """5×5 이웃과 중심의 밝기 순서를 24bit 서명으로 직접 인코딩."""
    gray=np.asarray(gray)
    h,w=gray.shape; padded=np.pad(gray,2,mode='edge'); bits=np.zeros((h,w),np.uint32)
    for dy in range(-2,3):
        for dx in range(-2,3):
            if dx==dy==0: continue
            bits=(bits<<1)|(padded[2+dy:2+dy+h,2+dx:2+dx+w]<gray)
    return bits


def _winner(cost,min_disparity,uniqueness,max_cost):
    best_index=np.argmin(cost,axis=0)
    best=np.take_along_axis(cost,best_index[None],axis=0)[0]
    second=np.full(best.shape,np.inf,np.float32)
    # 인접 disparity 후보는 같은 minimum basin이므로 uniqueness에서 제외.
    for index in range(len(cost)):
        np.minimum(second,np.where(np.abs(best_index-index)>1,cost[index],np.inf),out=second)
    # 검색 범위 양 끝의 minimum은 잘린 해일 수 있으므로 보수적으로 제거.
    valid=(best_index>0)&(best_index<len(cost)-1)&(best<max_cost)&((second-best)>uniqueness*np.maximum(best,1.0))
    lo=np.take_along_axis(cost,np.maximum(best_index-1,0)[None],axis=0)[0]
    hi=np.take_along_axis(cost,np.minimum(best_index+1,len(cost)-1)[None],axis=0)[0]
    denom=lo+hi-2*best
    offset=np.zeros(best.shape,np.float32)
    np.divide(.5*(lo-hi),denom,out=offset,where=denom>1e-6)
    disparity=best_index.astype(np.float32)+min_disparity+np.clip(offset,-.5,.5)
    return disparity,valid,best


def dense_correspondences(left,right,valid_left=None,valid_right=None,min_disparity=1,
                          max_disparity=160,window=9,uniqueness=.15,lr_threshold=1.,
                          min_texture=3.,max_cost=9.,support=3):
    """Census Hamming + 밝기차의 window 평균 비용으로 양방향 시차를 구한다.

    좌 픽셀(u,v)의 우 후보는 (u-d,v). 정답 후보 1개 선택 후 3점 parabola로
    subpixel 보정한다. 좌우 독립 winner가 같은 점을 가리켜야 하며 texture,
    uniqueness, 주변 8개 중 support개 이상의 시차 일관성을 요구한다.
    구멍 메우기/point interpolation은 하지 않는다.
    """
    if left.shape!=right.shape or left.ndim!=3 or left.shape[2]!=3:
        raise ValueError('같은 크기의 BGR stereo 이미지가 필요합니다.')
    h,w=left.shape[:2]
    if not (0<=min_disparity<max_disparity<w) or max_disparity-min_disparity<3:
        raise ValueError('시차 범위는 0<=min<max<width이며 최소 4개 후보가 필요합니다.')
    if not np.isfinite([uniqueness,lr_threshold,min_texture,max_cost]).all():
        raise ValueError('매칭 threshold는 모두 유한해야 합니다.')
    if window<3 or window%2==0 or window>min(h,w) or not 0<=uniqueness<1 or lr_threshold<0 or min_texture<0 or max_cost<=0 or not 0<=support<=8:
        raise ValueError('window는 홀수>=3, uniqueness=[0,1), 나머지 threshold와 support 범위를 확인하세요.')
    # 비용 volume 폭주 방지: float32 한 volume에 최대 400MB.
    if (max_disparity-min_disparity+1)*h*w>100_000_000:
        raise ValueError('비용 volume이 너무 큽니다. 이미지와 K를 함께 축소하거나 시차 범위를 줄이세요.')
    a=cv2.cvtColor(left,cv2.COLOR_BGR2GRAY); b=cv2.cvtColor(right,cv2.COLOR_BGR2GRAY)
    vl=np.ones((h,w),bool) if valid_left is None else np.asarray(valid_left,bool)
    vr=np.ones((h,w),bool) if valid_right is None else np.asarray(valid_right,bool)
    if vl.shape!=(h,w) or vr.shape!=(h,w): raise ValueError('valid mask shape 불일치')
    # Census가 필요한 2pixel border 및 aggregation window 전체가 유효해야 한다.
    vl=box_sum(vl,2)>=25; vr=box_sum(vr,2)>=25
    ca,cb=census5(a),census5(b)
    lut=np.array([bin(i).count('1') for i in range(256)],np.uint8)
    costs=np.full((max_disparity-min_disparity+1,h,w),1000.,np.float32)
    radius=window//2; area=window*window
    for index,d in enumerate(range(min_disparity,max_disparity+1)):
        end=w-d
        xor=ca[:,d:]^cb[:,:end]
        ham=lut[xor&255]+lut[(xor>>8)&255]+lut[(xor>>16)&255]
        raw=np.zeros((h,w),np.float32)
        raw[:,d:]=ham+np.minimum(np.abs(a[:,d:].astype(float)-b[:,:end]),30)/15
        pair_valid=np.zeros((h,w),bool); pair_valid[:,d:]=vl[:,d:]&vr[:,:end]
        supported=box_sum(pair_valid,radius)>=area
        costs[index]=np.where(supported,box_sum(raw,radius)/area,1000)
    dl,good_l,best_l=_winner(costs,min_disparity,uniqueness,max_cost)
    # 같은 photometric cost를 우 영상 좌표로 옮기고, 우측 winner는 독립적으로 구한다.
    for index,d in enumerate(range(min_disparity,max_disparity+1)):
        costs[index,:,:w-d]=costs[index,:,d:].copy()
        if d: costs[index,:,w-d:]=1000
    dr,good_r,_=_winner(costs,min_disparity,uniqueness,max_cost)
    del costs
    mean=box_sum(a,radius)/area
    texture=np.sqrt(np.maximum(0,box_sum(a.astype(float)**2,radius)/area-mean**2))
    yy,xx=np.indices((h,w)); rx=np.rint(xx-dl).astype(int)
    in_bounds=(rx>=0)&(rx<w); clipped=np.clip(rx,0,w-1)
    consistency=np.abs(dl-dr[yy,clipped])
    valid=good_l&(texture>=min_texture)&in_bounds&good_r[yy,clipped]&(consistency<=lr_threshold)
    before_support=int(valid.sum())
    for _ in range(2):
        counts=np.zeros((h,w),np.uint8)
        pd=np.pad(dl,1,constant_values=-1000); pv=np.pad(valid,1)
        for dy,dx in ((-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)):
            counts+=pv[1+dy:1+dy+h,1+dx:1+dx+w]&(np.abs(pd[1+dy:1+dy+h,1+dx:1+dx+w]-dl)<2)
        valid&=counts>=support
    return dict(disparity=dl,valid=valid,best_cost=best_l,left_right_error=consistency,
                before_support=before_support,texture=texture)
