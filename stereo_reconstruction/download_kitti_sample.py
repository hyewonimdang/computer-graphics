"""공개 KITTI Raw ZIP에서 필요한 동기화 frame만 HTTP Range로 받는다.

로그인/비공개 URL을 우회하지 않는다. 공개 S3 객체가 거부되면 명확히 실패한다.
약438MiB 전체 ZIP 대신 지정 color stereo/OXTS/calibration과 선택적 LiDAR만 저장.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import urllib.request
import zipfile

BASE='https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/'


class RemoteZipFile(io.RawIOBase):
    def __init__(self,url):
        self.url=url; self.position=0; self.transferred=0
        with urllib.request.urlopen(urllib.request.Request(url,method='HEAD'),timeout=60) as r:
            self.size=int(r.headers['Content-Length']); self.etag=r.headers.get('ETag')
        self.cache={}
    def seekable(self): return True
    def readable(self): return True
    def tell(self): return self.position
    def seek(self,offset,whence=0):
        self.position=offset if whence==0 else self.position+offset if whence==1 else self.size+offset
        if self.position<0: raise ValueError('negative seek')
        return self.position
    def read(self,size=-1):
        end=self.size if size<0 else min(self.size,self.position+size)
        start=self.position
        if end<=start: return b''
        key=(start,end)
        if key not in self.cache:
            headers={'Range':f'bytes={start}-{end-1}'}
            if self.etag: headers['If-Match']=self.etag
            with urllib.request.urlopen(urllib.request.Request(self.url,headers=headers),timeout=60) as r:
                if r.status!=206: raise RuntimeError('서버가 Range를 지원하지 않습니다. 전체 ZIP을 로컬로 받아 사용하세요.')
                if r.headers.get('Content-Range')!=f'bytes {start}-{end-1}/{self.size}': raise RuntimeError('잘못된 Range 응답')
                data=r.read(end-start+1)
            if len(data)!=end-start: raise RuntimeError('불완전 Range 응답')
            self.transferred+=len(data)
            # 작은 header/CD만 cache하여 전체 영상들을 메모리에 쌓지 않는다.
            if len(data)<250000: self.cache[key]=data
        else: data=self.cache[key]
        self.position=end
        return data


def download(root,frames=(0,5,10,15,20),date='2011_09_26',drive='0001',lidar=True):
    root=Path(root); root.mkdir(parents=True,exist_ok=True)
    sequence=f'{date}_drive_{drive}_sync'
    url=BASE+f'{date}_drive_{drive}/{sequence}.zip'
    remote=RemoteZipFile(url)
    prefix=f'{date}/{sequence}/'
    names=[prefix+f'{sensor}/timestamps.txt' for sensor in ('image_02','image_03','oxts')]
    for i in sorted(set(frames)|{0}):
        names.append(prefix+f'oxts/data/{i:010d}.txt')
    for i in frames:
        names.extend([prefix+f'image_{camera:02d}/data/{i:010d}.png' for camera in (2,3)])
    if lidar: names.append(prefix+f'velodyne_points/data/{frames[0]:010d}.bin')
    hashes={}
    with zipfile.ZipFile(remote) as z:
        for name in names:
            data=z.read(name) # zipfile가 CRC 검사
            dest=root/name; dest.parent.mkdir(parents=True,exist_ok=True); dest.write_bytes(data)
            hashes[name]=hashlib.sha256(data).hexdigest()
            print('Downloaded',name,flush=True)
    calib_url=BASE+date+'_calib.zip'
    with urllib.request.urlopen(calib_url,timeout=60) as r: data=r.read()
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        for name in ('calib_cam_to_cam.txt','calib_velo_to_cam.txt','calib_imu_to_velo.txt'):
            key=f'{date}/{name}'; raw=z.read(key); (root/key).write_bytes(raw)
            hashes[key]=hashlib.sha256(raw).hexdigest()
    report=dict(dataset='KITTI Raw synced+rectified',date=date,drive=drive,frames=list(frames),
        url=url,etag=remote.etag,archive_bytes=remote.size,downloaded_range_bytes=remote.transferred,
        calibration_url=calib_url,sha256=hashes,
        source='https://www.cvlibs.net/datasets/kitti/raw_data.php')
    (root/'download_manifest.json').write_text(json.dumps(report,indent=2))
    print('Selected data bytes:',remote.transferred,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--output',required=True)
    p.add_argument('--frames',nargs='+',type=int,default=[0,5,10,15,20])
    a=p.parse_args()
    if any(i<0 for i in a.frames) or len(set(a.frames))!=len(a.frames): p.error('중복 없는 비음수 frame id 필요')
    download(a.output,tuple(sorted(a.frames)))
