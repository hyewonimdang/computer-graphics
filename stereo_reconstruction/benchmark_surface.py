"""고정된 실제 stereo XYZ와 정답 normal 합성 장면에서 v1/v2 비교."""
import argparse,json,time,hashlib
from pathlib import Path
import numpy as np
import cv2
from surface_attributes import estimate_surface,make_records,export_surface,SURFACE_RECORD


def angular(a,b):
    return np.degrees(np.arccos(np.clip(np.sum(a.astype(float)*b,axis=1),-1,1)))


def synthetic_case():
    rng=np.random.default_rng(42);h,w=64,80;y,x=np.mgrid[:h,:w]
    rays=np.stack([(x-w/2)/180,(y-h/2)/180,np.ones_like(x)],axis=-1)
    truth=np.empty((h,w,3));truth[:,:w//2]=[.25,.12,-1];truth[:,w//2:]=[-.35,-.15,-1]
    truth/=np.linalg.norm(truth,axis=2)[...,None]
    distance=np.where(x<w//2,-5.,-8.)
    depth=distance/np.sum(rays*truth,axis=2)
    depth*=1+rng.normal(0,.004,depth.shape)
    xyz=rays*depth[...,None]
    keep=rng.random((h,w))>.12
    uv=np.column_stack([x[keep],y[keep]]);boundary=np.abs(uv[:,0]-(w/2-.5))<=3
    return xyz[keep],uv,(w,h),truth[keep],boundary


def run(source,output):
    source=Path(source);out=Path(output);out.mkdir(parents=True,exist_ok=True)
    cloud=np.load(source/'point_cloud.npy');diag=json.loads((source/'diagnostics.json').read_text())
    with np.load(source/'dense_data.npz') as d:uv=d['rectified_left'].astype(int);Q=d['Q_left'];xyz=cloud[:,:3]@Q.T*diag['surface_meters_per_unit']
    size=diag['image_size'];results={};times={}
    for method in ['legacy','robust']:
        start=time.perf_counter();results[method]=estimate_surface(xyz,uv,size,method=method);times[method]=time.perf_counter()-start
    old=np.frombuffer((source/'surface.bin').read_bytes(),dtype=SURFACE_RECORD)
    np.testing.assert_array_equal(old,make_records(results['legacy'],np.arange(len(xyz)),Q.T))
    # Fair noise proxy: same depth-continuous edges, both normals valid under both methods.
    grid=np.full((size[1],size[0]),-1,int);grid[uv[:,1],uv[:,0]]=np.arange(len(uv));pairs=[]
    common=results['legacy']['valid']&results['robust']['valid']
    for dx,dy in [(1,0),(0,1)]:
        u,v=uv[:,0]+dx,uv[:,1]+dy;inside=(u<size[0])&(v<size[1]);j=grid[np.minimum(v,size[1]-1),np.minimum(u,size[0]-1)]
        valid=inside&(j>=0)&common&common[np.maximum(j,0)]
        valid &= np.abs(xyz[:,2]-xyz[np.maximum(j,0),2])<=.02+.03*np.minimum(xyz[:,2],xyz[np.maximum(j,0),2])
        pairs.append(np.column_stack([np.flatnonzero(valid),j[valid]]))
    pairs=np.concatenate(pairs);report=dict(points=len(xyz),common_continuous_edges=len(pairs),real={},synthetic={},
        note='Real adjacent-normal angle is a noise/continuity proxy, not ground-truth accuracy; real geometry/point IDs are unchanged.')
    for method,a in results.items():
        angles=angular(a['normal'][pairs[:,0]],a['normal'][pairs[:,1]])
        report['real'][method]=dict(valid=int(a['valid'].sum()),valid_percent=float(a['valid'].mean()*100),seconds=times[method],
            common_edge_angle_median_deg=float(np.median(angles)),common_edge_angle_p90_deg=float(np.quantile(angles,.9)))
    synxyz,synuv,synsize,truth,boundary=synthetic_case()
    for method in ['legacy','robust']:
        a=estimate_surface(synxyz,synuv,synsize,method=method);valid=a['valid'];angles=angular(a['normal'][valid],truth[valid]);edge=valid&boundary
        report['synthetic'][method]=dict(valid=int(valid.sum()),total=len(valid),valid_percent=float(valid.mean()*100),
            angular_median_deg=float(np.median(angles)),angular_p90_deg=float(np.quantile(angles,.9)),
            boundary_valid=int(edge.sum()),boundary_total=int(boundary.sum()),boundary_angular_median_deg=float(np.median(angular(a['normal'][edge],truth[edge]))))
    improved=out/'improved';improved.mkdir(exist_ok=True)
    # Copy source data byte-for-byte; only surface attributes are regenerated.
    for name in ['point_cloud.npy','dense_data.npz','diagnostics.json','01_rectified_left.png','02_rectified_right.png','point_attributes.npz']:
        (improved/name).write_bytes((source/name).read_bytes())
    records=make_records(results['robust'],np.arange(len(xyz)),Q.T)
    export_surface(improved,records,cloud[:,:3]*diag['surface_meters_per_unit'],cloud[:,3:6],'original_left_camera')
    np.savez_compressed(improved/'fit_diagnostics.npz',plane_fitted=results['robust']['plane_fitted'],
        neighborhood_count=results['robust']['neighborhood_count'],relative_plane_residual=results['robust']['relative_plane_residual'])
    report['unchanged_point_cloud_sha256']=hashlib.sha256((source/'point_cloud.npy').read_bytes()).hexdigest()
    assert (improved/'point_cloud.npy').read_bytes()==(source/'point_cloud.npy').read_bytes()
    (out/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    image=cv2.imdecode(np.frombuffer((source/'01_rectified_left.png').read_bytes(),np.uint8),1)[:,:,::-1]
    preview(image,uv,results,out/'comparison.png')
    print(json.dumps(report,indent=2));return report


def preview(image,uv,results,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(2,3,figsize=(18,7),constrained_layout=True)
    h,w=image.shape[:2];u,v=uv.T;light=np.array([-.3,-.5,-1]);light/=np.linalg.norm(light)
    for row,(method,a) in enumerate(results.items()):
        valid=a['valid'];normal=a['normal'];norm=np.zeros((h,w,3));shade=norm.copy();radius=np.full((h,w),np.nan)
        norm[v[valid],u[valid]]=(normal[valid]+1)/2
        illumination=.15+.85*np.maximum(0,normal@light)
        shade[v[valid],u[valid]]=image[v[valid],u[valid]]/255*illumination[valid,None]
        radius[v[valid],u[valid]]=a['radius_m'][valid]*100
        axs[row,0].imshow(norm);axs[row,1].imshow(shade);im=axs[row,2].imshow(radius,cmap='viridis',vmin=0,vmax=15)
        axs[row,0].set_title(f'{method}: normals | {valid.sum():,} / {len(valid):,}')
        axs[row,1].set_title('Same CPU Lambert light (diagnostic)');axs[row,2].set_title('Splat radius: 0–15 cm')
        for ax in axs[row]:ax.axis('off')
    fig.colorbar(im,ax=axs[:,2],shrink=.65)
    fig.suptitle('Same stereo XYZ, same depth-edge threshold | 4-neighbour derivatives vs connected robust plane',fontsize=15)
    fig.savefig(path,dpi=150);plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--output',required=True);a=p.parse_args();run(a.source,a.output)
