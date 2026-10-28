"""SRF1 파일/packet/회전/ID/실제 JS 수신 검증 및 그래픽스용 진단 그림."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
import cv2
from surface_attributes import SURFACE_RECORD,SURFACE_HEADER,validate_records,estimate_surface,make_records
from point_stream import RECORD


def verify(root,received=None):
    root=Path(root); surf=np.frombuffer((root/'surface.bin').read_bytes(),dtype=SURFACE_RECORD);validate_records(surf)
    meta=json.loads((root/'surface.json').read_text());assert len(surf)==meta['point_count']
    method='robust' if meta.get('estimator')=='connected_huber_inverse_depth_plane_v2' else 'legacy'
    with np.load(root/'surface.npz') as a:
        for key in surf.dtype.names:np.testing.assert_array_equal(a[key],surf[key])
    frames=[]
    if (root/'config.json').exists():
        summary=json.loads((root/'summary.json').read_text());points=np.frombuffer((root/'map_points.bin').read_bytes(),dtype=RECORD)
        np.testing.assert_array_equal(surf['point_id'],points['point_id']);parts=[]
        for frame in summary['frames']:
            folder=root/f'frames/{frame["frame_id"]:010d}'
            local=np.frombuffer((folder/'surface.bin').read_bytes(),dtype=SURFACE_RECORD);parts.append(local.tobytes())
            with np.load(folder/'observations.npz') as a:
                ix=a['selected_indices']; xyz=a['camera_xyz_m'];uv=a['pixel_left']; R=a['T_world_from_observation_camera'][:3,:3]
                attrs=estimate_surface(xyz,uv,frame['image_size'],method=method); expected=make_records({k:v[ix] for k,v in attrs.items()},local['point_id'],R)
                np.testing.assert_array_equal(expected,local)
                valid=(local['flags']&1)!=0;camnorm=local['normal']@R
                assert np.all(np.sum(camnorm[valid]*xyz[ix][valid],axis=1)<0)
            packets=sorted(folder.glob('surface_*.srf'));payload=[]
            for i,p in enumerate(packets):
                data=p.read_bytes(); hdr=SURFACE_HEADER.unpack_from(data)
                assert hdr[:5]==(b'SRF1',1,32,frame['frame_id'],i) and hdr[6:]==(32,int(i==len(packets)-1),0)
                assert len(data)==32+32*hdr[5];payload.append(data[32:])
            assert b''.join(payload)==local.tobytes()
            frames.append(dict(frame=frame['frame_id'],surface_records=len(local),valid=int(valid.sum()),packets=len(packets)))
        assert b''.join(parts)==surf.tobytes()
        assert hashlib.sha256(surf.tobytes()).hexdigest()==summary['surface_sha256']
        first=summary['frames'][0];folder=root/f'frames/{first["frame_id"]:010d}'
        firstsurf=np.frombuffer((folder/'surface.bin').read_bytes(),dtype=SURFACE_RECORD)
        with np.load(folder/'observations.npz') as a: uv=a['pixel_left'][a['selected_indices']]; norm=firstsurf['normal']@a['T_world_from_observation_camera'][:3,:3]
        image=cv2.imdecode(np.frombuffer((folder/'left_rectified.png').read_bytes(),dtype=np.uint8),1)[:,:,::-1]
        vis=firstsurf
    else:
        cloud=np.load(root/'point_cloud.npy');d=json.loads((root/'diagnostics.json').read_text());scale=d['surface_meters_per_unit']
        np.testing.assert_array_equal(surf['point_id'],np.arange(len(cloud)))
        with np.load(root/'dense_data.npz') as a:
            uv=a['rectified_left'].astype(int);q=a['Q_left'];xyz=cloud[:,:3]@q.T*scale
            expected=make_records(estimate_surface(xyz,uv,d['image_size'],method=method),surf['point_id'],q.T)
            np.testing.assert_array_equal(expected,surf);norm=surf['normal']@q.T
        valid=(surf['flags']&1)!=0;assert np.all(np.sum(surf['normal'][valid]*cloud[:,:3][valid],axis=1)<0)
        image=cv2.imdecode(np.frombuffer((root/'01_rectified_left.png').read_bytes(),dtype=np.uint8),1)[:,:,::-1];vis=surf
    if received:
        assert (Path(received)/'received_surface.bin').read_bytes()==surf.tobytes()
        assert (Path(received)/'received_points.bin').read_bytes()==(root/'map_points.bin').read_bytes()
    valid=(surf['flags']&1)!=0
    report=dict(status='PASS',points=len(surf),valid_normals=int(valid.sum()),invalid_normals=int((~valid).sum()),
        valid_percent=float(valid.mean()*100) if len(surf) else 0,
        max_unit_length_error=float(np.max(np.abs(np.linalg.norm(surf['normal'][valid],axis=1)-1))) if valid.any() else 0,
        radius_m_quantiles=np.quantile(surf['radius_m'][valid],[0,.5,.95,1]).tolist() if valid.any() else [],
        surface_sha256=hashlib.sha256(surf.tobytes()).hexdigest(),javascript_bytes_identical=bool(received),frames=frames,
        evidence='Synthetic plane normal/radius correctness in unit tests; real sample validates geometry/IDs/units/serialization. No real ground-truth normals.')
    (root/'surface_verification.json').write_text(json.dumps(report,indent=2)+'\n')
    plot(image,uv,norm,vis,root/'surface_preview.png')
    print(json.dumps(report,indent=2));return report


def plot(image,uv,norm,surf,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    h,w=image.shape[:2]; valid=(surf['flags']&1)!=0;u,v=uv.T
    normal_image=np.zeros((h,w,3));normal_image[v[valid],u[valid]]=(norm[valid]+1)/2
    light=np.array([-.3,-.5,-1.]);light/=np.linalg.norm(light)
    shade=np.zeros((h,w,3));intensity=.15+.85*np.maximum(0,norm@light)
    shade[v[valid],u[valid]]=image[v[valid],u[valid]]/255*intensity[valid,None]
    radius=np.full((h,w),np.nan);radius[v[valid],u[valid]]=surf['radius_m'][valid]*100
    fig,ax=plt.subplots(2,2,figsize=(15,7),constrained_layout=True)
    ax[0,0].imshow(image);ax[0,0].set_title('Actual stereo sample: rectified left')
    ax[0,1].imshow(normal_image);ax[0,1].set_title('Custom normals: (n + 1) / 2; invalid/missing = black')
    ax[1,0].imshow(shade);ax[1,0].set_title('CPU Lambert diagnostic (not a B shader implementation)')
    im=ax[1,1].imshow(radius,cmap='viridis',vmin=0,vmax=15);ax[1,1].set_title('Circular splat radius (cm); missing = white');fig.colorbar(im,ax=ax[1,1],shrink=.7)
    for a in ax.flat:a.axis('off')
    fig.suptitle(f'Direct finite differences / cross product | {valid.sum():,} valid of {len(surf):,} points',fontsize=16)
    fig.savefig(path,dpi=150);plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',required=True);p.add_argument('--received');a=p.parse_args();verify(a.run,a.received)
