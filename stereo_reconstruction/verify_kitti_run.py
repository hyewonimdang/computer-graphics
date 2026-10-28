"""실제 실행의 pose/voxel/attribute/packet와 JS 수신 bytes까지 교차 검증."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import cv2
from point_stream import RECORD,unpack_packet
from kitti_io import apply_pose


def verify(run,received=None):
    root=Path(run); summary=json.loads((root/'summary.json').read_text())
    cloud=np.frombuffer((root/'map_points.bin').read_bytes(),dtype=RECORD)
    assert len(cloud)==summary['map_points']
    np.testing.assert_array_equal(cloud['point_id'],np.arange(len(cloud)))
    cells=np.floor(cloud['xyz'].astype(float)/summary['config']['voxel_size_m']).astype(np.int64)
    assert len(np.unique(cells,axis=0))==len(cloud)
    at=0; packet_count=0; checks=[]; timestamps=[]
    for meta in summary['frames']:
        folder=root/f'frames/{meta["frame_id"]:010d}'
        points=np.frombuffer((folder/'new_points.bin').read_bytes(),dtype=RECORD)
        n=len(points); assert n==meta['point_count']==meta['new_points']
        np.testing.assert_array_equal(points,cloud[at:at+n])
        with np.load(folder/'observations.npz',allow_pickle=False) as a:
            idx=a['selected_indices']; assert len(idx)==n
            world=apply_pose(a['T_world_from_observation_camera'],a['camera_xyz_m'][idx])
            np.testing.assert_array_equal(world.astype('<f4'),points['xyz'])
            # 원래 관측으로 역변환: pose를 한 번만 적용했는지 확인.
            local=apply_pose(np.linalg.inv(a['T_world_from_observation_camera']),points['xyz'])
            np.testing.assert_allclose(local,a['camera_xyz_m'][idx],atol=1e-5,rtol=1e-6)
            for field,key in [('uv','pixel_left'),('disparity','disparity_px'),('confidence','confidence')]:
                np.testing.assert_array_equal(points[field],a[key][idx])
            np.testing.assert_array_equal(points['point_id'],a['point_id'])
            np.testing.assert_array_equal(points['rgba'][:,:3],a['rgb'][idx])
            rgb=cv2.imdecode(np.frombuffer((folder/'left_rectified.png').read_bytes(),dtype=np.uint8),cv2.IMREAD_COLOR)[:,:,::-1]
            np.testing.assert_array_equal(points['rgba'][:,:3],rgb[points['uv'][:,1],points['uv'][:,0]])
            uv=a['pixel_left']; mask=a['valid_mask']
            np.testing.assert_array_equal(np.column_stack(np.nonzero(mask)[::-1]),uv)
            np.testing.assert_array_equal(a['disparity_px'],a['disparity_map_px'][uv[:,1],uv[:,0]])
        rebuilt=[]; files=sorted(folder.glob('chunk_*.stp'))
        assert len(files)==meta['chunk_count']
        for i,path in enumerate(files):
            frame,chunk,last,part=unpack_packet(path.read_bytes())
            assert frame==meta['frame_id'] and chunk==i and last==(i==len(files)-1)
            rebuilt.append(part.tobytes())
        assert b''.join(rebuilt)==points.tobytes(); packet_count+=len(files)
        at+=n; timestamps.append(meta['timestamp_seconds'])
        checks.append(dict(frame_id=meta['frame_id'],new_points=n,pose_attributes_packets='PASS'))
    assert at==len(cloud) and np.all(np.diff(timestamps)>0)
    sha=hashlib.sha256((root/'map_points.bin').read_bytes()).hexdigest()
    assert sha==summary['binary_sha256']
    if received:
        received=Path(received)
        assert (root/'map_points.bin').read_bytes()==(received/'received_points.bin').read_bytes()
        client=json.loads((received/'validation.json').read_text())
        assert client['status']=='PASS' and client['payload_sha256']==sha and client['chunks']==packet_count
    report=dict(status='PASS',map_points=len(cloud),unique_voxels=len(cloud),packets=packet_count,
        sha256=sha,javascript_received_identical=bool(received),frames=checks)
    (root/'verification.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2)); return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--run',required=True); p.add_argument('--received')
    a=p.parse_args(); verify(a.run,a.received)
