"""STP1 v1 레코드/packet 및 append-only 대표 관측 voxel map. 직접 NumPy 구현."""
import struct
import numpy as np

RECORD=np.dtype(dict(names=['xyz','rgba','uv','disparity','confidence','point_id'],
    formats=[('<f4',3),('u1',4),('<u2',2),'<f4','<f4','<u4'],offsets=[0,12,16,20,24,28],itemsize=32))
HEADER=struct.Struct('<4sHHIIIIII')


def packetize(records,frame_id,chunk_points=4096):
    if records.dtype!=RECORD or not 0<=frame_id<2**32 or not 1<=chunk_points<=65536: raise ValueError('잘못된 packet 설정')
    # 빈 frame도 last=true인0점 packet 하나를 보낸다.
    for chunk,start in enumerate(range(0,max(1,len(records)),chunk_points)):
        subset=records[start:start+chunk_points]; last=start+chunk_points>=len(records)
        yield HEADER.pack(b'STP1',1,32,frame_id,chunk,len(subset),32,int(last),0)+subset.tobytes()


def unpack_packet(data):
    if len(data)<32: raise ValueError('truncated header')
    magic,version,header,frame,chunk,count,stride,flags,reserved=HEADER.unpack_from(data)
    if (magic,version,header,stride,reserved)!=(b'STP1',1,32,32,0) or flags&~1 or len(data)!=32+32*count:
        raise ValueError('잘못된 STP1 packet')
    return frame,chunk,bool(flags&1),np.frombuffer(data,dtype=RECORD,offset=32,count=count)


class VoxelMap:
    """한 voxel당 관측점1개, 이미 전송한 점은 변경하지 않는 append-only map.

    frame 내부에서는 confidence 최고점을 선택 (동률이면 입력 순서).
    과거에 채워진 voxel은 중복으로 버린다. centroid/평균을 만들지 않으므로
    선택된 원본 uv/disparity/confidence와 좌표의 기하학적 관계를 유지한다.
    """
    def __init__(self,voxel_size=.15,max_points=2_000_000):
        if not np.isfinite(voxel_size) or voxel_size<=0 or not isinstance(max_points,int) or not 0<max_points<2**32:
            raise ValueError('양의 voxel_size와 uint32 범위 map capacity 필요')
        self.voxel_size=float(voxel_size); self.max_points=max_points; self.cells=set(); self.count=0
    def add(self,xyz,rgb,uv,disparity,confidence):
        xyz=np.asarray(xyz,float); rgb=np.asarray(rgb); uv=np.asarray(uv)
        disparity=np.asarray(disparity); confidence=np.asarray(confidence)
        n=len(xyz)
        if xyz.shape!=(n,3) or rgb.shape!=(n,3) or uv.shape!=(n,2) or disparity.shape!=(n,) or confidence.shape!=(n,): raise ValueError('point attribute shape 불일치')
        if not np.isfinite(xyz).all() or not np.isfinite(disparity).all() or not np.isfinite(confidence).all() or np.any((confidence<0)|(confidence>1)):
            raise ValueError('유한한 XYZ/disparity 및0..1 confidence 필요')
        if not np.isfinite(rgb).all() or np.any((rgb<0)|(rgb>255)|(rgb!=np.floor(rgb))) or not np.isfinite(uv).all() or np.any((uv<0)|(uv>65535)|(uv!=np.floor(uv))): raise ValueError('RGB/UV 범위 오류')
        # 실제 wire float32 위치를 기준으로 voxelize하여 수신자가 동일 cell을 재현한다.
        positions=xyz.astype('<f4')
        cell_float=np.floor(positions.astype(float)/self.voxel_size)
        if not np.isfinite(cell_float).all() or np.any(np.abs(cell_float)>2**62): raise ValueError('voxel index overflow')
        cells=cell_float.astype(np.int64)
        ordered=np.argsort(-confidence,kind='stable')
        _,first=np.unique(cells[ordered],axis=0,return_index=True)
        candidates=np.sort(ordered[first]) # 원본 pixel 순서로 보냄
        selected=[]; new_keys=[]
        for i in candidates:
            key=tuple(cells[i])
            if key not in self.cells: selected.append(i); new_keys.append(key)
        if self.count+len(selected)>self.max_points:
            raise MemoryError('voxel map capacity 초과: 큰 voxel 또는 적은 frame으로 재실행하세요. 일부만 전송하지 않습니다.')
        chosen=np.asarray(selected,dtype=np.int64)
        out=np.zeros(len(chosen),dtype=RECORD)
        out['xyz']=positions[chosen]; out['rgba'][:,:3]=rgb[chosen]; out['rgba'][:,3]=255
        out['uv']=uv[chosen]; out['disparity']=disparity[chosen]; out['confidence']=confidence[chosen]
        out['point_id']=np.arange(self.count,self.count+len(chosen),dtype='<u4')
        self.cells.update(new_keys); self.count+=len(chosen)
        return out,chosen,dict(input_points=n,within_frame_voxels=len(candidates),new_points=len(chosen),
            duplicate_existing_voxels=len(candidates)-len(chosen),map_points=self.count)
