"""KITTI Raw synced/rectified 및 Odometry color stereo 로더. pykitti 미사용.

모든 SE(3)는 column-vector 규약 T_destination_from_source, 위치 단위 metre.
Raw pose는 OXTS GPS/IMU를 사용하며 시각적으로 새로 추정하는 odometry가 아니다.
"""
from pathlib import Path
import numpy as np
import cv2


def read_calib(path):
    result={}
    for line in Path(path).read_text().splitlines():
        if ':' not in line: continue
        key,value=line.split(':',1)
        try: array=np.array([float(x) for x in value.split()])
        except ValueError: continue # date field
        if not np.isfinite(array).all(): raise ValueError('nonfinite calibration')
        result[key]=array
    return result


def transform(R=None,t=None):
    T=np.eye(4)
    if R is not None: T[:3,:3]=np.asarray(R).reshape(3,3)
    if t is not None: T[:3,3]=np.asarray(t).reshape(3)
    return validate_pose(T)


def validate_pose(T):
    T=np.asarray(T,dtype=float)
    if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1],atol=1e-8):
        raise ValueError('유한한4x4 rigid transform 필요')
    R=T[:3,:3]
    if not np.allclose(R.T@R,np.eye(3),atol=5e-5) or not np.isclose(np.linalg.det(R),1,atol=5e-5):
        raise ValueError('pose rotation이 올바르지 않습니다.')
    return T


def apply_pose(T,xyz):
    T=validate_pose(T); xyz=np.asarray(xyz,float)
    if xyz.ndim!=2 or xyz.shape[1]!=3 or not np.isfinite(xyz).all(): raise ValueError('유한한 Nx3 필요')
    return xyz@T[:3,:3].T+T[:3,3]


def projection_camera(P):
    P=np.asarray(P,float).reshape(3,4)
    K=P[:,:3]
    if not np.allclose(K[2],[0,0,1]) or K[0,0]<=0 or K[1,1]<=0: raise ValueError('rectified projection 필요')
    return K,transform(t=np.linalg.solve(K,P[:,3]))


def oxts_pose(packet,scale):
    """위경도 Mercator와 Rz(yaw)Ry(pitch)Rx(roll)를 직접 구성."""
    lat,lon,alt,roll,pitch,yaw=np.asarray(packet,float)[:6]
    if not np.isfinite([lat,lon,alt,roll,pitch,yaw,scale]).all() or not -89<lat<89 or not -180<=lon<=180 or not 0<scale<=1:
        raise ValueError('OXTS 범위 오류')
    cr,sr=np.cos(roll),np.sin(roll); cp,sp=np.cos(pitch),np.sin(pitch); cy,sy=np.cos(yaw),np.sin(yaw)
    Rx=np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    Ry=np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rz=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    radius=6378137.
    t=[scale*radius*np.deg2rad(lon),scale*radius*np.log(np.tan(np.pi/4+np.deg2rad(lat)/2)),alt]
    return transform(Rz@Ry@Rx,t)


class KittiRaw:
    kind='raw'; pose_source='OXTS GPS/IMU Mercator + roll/pitch/yaw; not visual odometry'
    def __init__(self,root,date='2011_09_26',drive='0001'):
        root=Path(root); self.calib_root=root/date
        self.path=self.calib_root/f'{date}_drive_{drive}_sync'
        self.name=self.path.name
        cam=read_calib(self.calib_root/'calib_cam_to_cam.txt')
        self.K_left,C2=projection_camera(cam['P_rect_02'])
        self.K_right,C3=projection_camera(cam['P_rect_03'])
        self.T_right_left=C3@np.linalg.inv(C2)
        rect=transform(cam['R_rect_00'])
        v=read_calib(self.calib_root/'calib_velo_to_cam.txt')
        i=read_calib(self.calib_root/'calib_imu_to_velo.txt')
        self.T_camera_velo=C2@rect@transform(v['R'],v['T'])
        self.T_camera_imu=self.T_camera_velo@transform(i['R'],i['T'])
        packet0=self.oxts(0); self.scale=float(np.cos(np.deg2rad(packet0[0])))
        self.world_from_imu0=oxts_pose(packet0,self.scale)
        self.origin_mercator=self.world_from_imu0[:3,3].copy()
        # 큰 GPS 좌표의 소수점 손실을 줄이기 위해 translation origin을 먼저 제거.
        self.world_from_imu0[:3,3]=0
        self.reference_camera=self.world_from_imu0@np.linalg.inv(self.T_camera_imu)
        self.times=[s.strip() for s in (self.path/'image_02/timestamps.txt').read_text().splitlines()]
        self.right_times=[s.strip() for s in (self.path/'image_03/timestamps.txt').read_text().splitlines()]
        self.size=tuple(cam['S_rect_02'].astype(int))
    def oxts(self,index):
        p=np.loadtxt(self.path/f'oxts/data/{index:010d}.txt').reshape(-1)
        if len(p)<30 or not np.isfinite(p).all(): raise ValueError('불완전 OXTS packet')
        return p
    def pose(self,index):
        p=oxts_pose(self.oxts(index),self.scale); p[:3,3]-=self.origin_mercator
        return validate_pose(np.linalg.inv(self.reference_camera)@p@np.linalg.inv(self.T_camera_imu))
    def images(self,index):
        return self._read(self.path/f'image_02/data/{index:010d}.png',self.path/f'image_03/data/{index:010d}.png')
    def _read(self,left,right):
        a,b=cv2.imread(str(left)),cv2.imread(str(right))
        if a is None or b is None: raise ValueError(f'KITTI 이미지 누락: {left}, {right}')
        if a.shape!=b.shape or (a.shape[1],a.shape[0])!=self.size: raise ValueError('KITTI 이미지/보정 크기 불일치')
        return a,b
    def timestamp(self,index):
        return float((np.datetime64(self.times[index])-np.datetime64(self.times[0]))/np.timedelta64(1,'s'))
    def validate_frame(self,index):
        packet=self.oxts(index); self.pose(index)
        if abs(float((np.datetime64(self.times[index])-np.datetime64(self.right_times[index]))/np.timedelta64(1,'s')))>.02:
            raise ValueError('좌우 timestamp 차이가20ms를 초과합니다.')
        return dict(position_accuracy_m=float(packet[23]),navstat=int(packet[25]),satellites=int(packet[26]))
    def lidar(self,index):
        p=self.path/f'velodyne_points/data/{index:010d}.bin'
        if not p.exists(): return None
        data=np.fromfile(p,dtype='<f4')
        if len(data)%4: raise ValueError('잘못된 velodyne scan')
        return data.reshape(-1,4)


class KittiOdometry:
    kind='odometry'; pose_source='provided KITTI odometry camera-0 poses; not visual odometry estimated here'
    def __init__(self,root,sequence='00',poses=None):
        root=Path(root); self.path=root/'sequences'/sequence; self.name='odometry_'+sequence
        c=read_calib(self.path/'calib.txt')
        self.K_left,self.C2=projection_camera(c['P2']); self.K_right,C3=projection_camera(c['P3'])
        self.T_right_left=C3@np.linalg.inv(self.C2)
        values=np.loadtxt(poses or root/'poses'/f'{sequence}.txt',ndmin=2)
        if values.shape[1]!=12: raise ValueError('pose 파일의 한 행은3x4 row-major12개 값이어야 합니다.')
        self.poses=np.repeat(np.eye(4)[None],len(values),axis=0); self.poses[:,:3,:]=values.reshape(-1,3,4)
        for p in self.poses: validate_pose(p)
        self.times=np.loadtxt(self.path/'times.txt',ndmin=1)
        if len(self.times)!=len(self.poses): raise ValueError('pose/timestamp 개수 불일치')
        self.reference=self.poses[0]@np.linalg.inv(self.C2)
        self.T_camera_velo=None
    def images(self,index):
        left=cv2.imread(str(self.path/f'image_2/{index:06d}.png')); right=cv2.imread(str(self.path/f'image_3/{index:06d}.png'))
        if left is None or right is None or left.shape!=right.shape: raise ValueError('color stereo image_2/image_3 누락/크기 불일치')
        return left,right
    def pose(self,index):
        return validate_pose(np.linalg.inv(self.reference)@self.poses[index]@np.linalg.inv(self.C2))
    def timestamp(self,index): return float(self.times[index]-self.times[0])
    def validate_frame(self,index): self.pose(index); return {}
    def lidar(self,index): return None
