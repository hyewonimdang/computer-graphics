"""직접 구현한 organized point-cloud 법선/원형 splat 반경. NumPy만 사용."""
from pathlib import Path
import json
import struct
import numpy as np

SURFACE_RECORD = np.dtype(dict(names=['point_id','normal','radius_m','support','flags','reserved'],
    formats=['<u4',('<f4',3),'<f4','<f4','<u4','<u4'], offsets=[0,4,16,20,24,28],itemsize=32))
SURFACE_HEADER = struct.Struct('<4sHHIIIIII')
DEFAULTS = dict(relative_depth_jump=.03, absolute_depth_jump_m=.02, max_radius_m=.15, min_view_cos=.15)


def estimate_surface_legacy(xyz_m, uv, image_size, **options):
    """입력: rectified observation camera (+X right,+Y down,+Z forward), metre.

    즉시 인접한 4픽셀만 사용. 유효/깊이 연속인 이웃에서 중앙 또는 단측 차분;
    cross(du,dv)를 카메라 방향으로 뒤집음. 반경은 국소 픽셀 footprint의 반대각.
    경계에서는 가장 가까운 연결 이웃 거리의 절반으로 제한. invalid는 0벡터/0반경.
    """
    cfg=dict(DEFAULTS); cfg.update(options)
    if set(cfg)!=set(DEFAULTS) or not np.isfinite(list(cfg.values())).all() or any(v<=0 for v in cfg.values()) or cfg['min_view_cos']>=1:
        raise ValueError('Invalid surface parameters')
    xyz=np.asarray(xyz_m,float); uv=np.asarray(uv); w,h=map(int,image_size); n=len(xyz)
    if xyz.shape!=(n,3) or uv.shape!=(n,2) or w<1 or h<1: raise ValueError('Invalid shape/image size')
    if not np.isfinite(xyz).all() or np.any(xyz[:,2]<=0) or not np.isfinite(uv).all() or np.any(uv!=np.floor(uv)):
        raise ValueError('Finite positive camera depth and integer UV required')
    if np.any(uv<0) or np.any(uv[:,0]>=w) or np.any(uv[:,1]>=h) or len(np.unique(uv,axis=0))!=n: raise ValueError('UV bounds/duplicates')
    uv=uv.astype(int); grid=np.full((h,w),-1,int); grid[uv[:,1],uv[:,0]]=np.arange(n)
    vectors=[]; good=[]; distances=[]
    for dx,dy in [(-1,0),(1,0),(0,-1),(0,1)]:
        u,v=uv[:,0]+dx,uv[:,1]+dy; inside=(u>=0)&(u<w)&(v>=0)&(v<h)
        idx=grid[np.clip(v,0,h-1),np.clip(u,0,w-1)]
        neighbor=xyz[np.maximum(idx,0)] if n else xyz.copy()
        delta=neighbor-xyz
        ok=inside&(idx>=0)&(np.abs(delta[:,2])<=cfg['absolute_depth_jump_m']+cfg['relative_depth_jump']*np.minimum(xyz[:,2],neighbor[:,2]))
        vectors.append(delta);good.append(ok);distances.append(np.where(ok,np.linalg.norm(delta,axis=1),np.inf))
    left,right,up,down=good
    du=(vectors[1]*right[:,None]-vectors[0]*left[:,None])/np.maximum(1,left.astype(int)+right)[:,None]
    dv=(vectors[3]*down[:,None]-vectors[2]*up[:,None])/np.maximum(1,up.astype(int)+down)[:,None]
    normal=np.cross(du,dv); length=np.linalg.norm(normal,axis=1)
    normal/=np.maximum(length,1e-30)[:,None]
    facing=np.sum(normal*xyz,axis=1)>0; normal[facing]*=-1
    cosine=-np.sum(normal*xyz,axis=1)/np.maximum(np.linalg.norm(xyz,axis=1),1e-30)
    valid=(left|right)&(up|down)&(length>1e-12)&(cosine>=cfg['min_view_cos'])
    edge=~(left&right&up&down)
    raw=.5*np.sqrt(np.sum(du*du+dv*dv,axis=1))
    limit=np.where(edge,.5*np.min(np.asarray(distances),axis=0),np.inf)
    radius=np.minimum(np.minimum(raw,limit),cfg['max_radius_m'])
    valid &= np.isfinite(radius)&(radius>0)
    normal[~valid]=0; radius[~valid]=0
    flags=valid.astype(np.uint32)|(edge.astype(np.uint32)<<1)|((valid&(radius<raw)).astype(np.uint32)<<2)
    support=sum(g.astype(float) for g in good)/4; support[~valid]=0
    return dict(normal=normal.astype('<f4'),radius_m=radius.astype('<f4'),support=support.astype('<f4'),flags=flags,valid=valid)


def make_records(attributes,point_ids,rotation=None):
    ids=np.asarray(point_ids)
    if ids.ndim!=1 or np.any(ids<0) or np.any(ids>=2**32) or np.any(ids!=np.floor(ids)) or len(np.unique(ids))!=len(ids): raise ValueError('Invalid point IDs')
    out=np.zeros(len(ids),dtype=SURFACE_RECORD);out['point_id']=ids
    normals=np.asarray(attributes['normal'])
    if rotation is not None:
        rotation=np.asarray(rotation,float)
        if rotation.shape!=(3,3) or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-7) or not np.isclose(np.linalg.det(rotation),1): raise ValueError('Rigid rotation required')
        normals=normals@rotation.T
    for k in ['radius_m','support','flags']: out[k]=attributes[k]
    out['normal']=normals
    validate_records(out)
    return out


def validate_records(records):
    if records.dtype!=SURFACE_RECORD: raise ValueError('Wrong surface dtype')
    valid=(records['flags']&1)!=0
    if np.any(records['flags']&~np.uint32(7)) or np.any(records['reserved']): raise ValueError('Unsupported surface flags')
    if not all(np.isfinite(records[k]).all() for k in ['normal','radius_m','support']): raise ValueError('Nonfinite surface')
    if np.any((records['support']<0)|(records['support']>1)) or np.any(records['radius_m'][valid]<=0): raise ValueError('Invalid radius/support')
    if not np.allclose(np.linalg.norm(records['normal'][valid],axis=1),1,atol=1e-5): raise ValueError('Nonunit normal')
    if np.any(records['normal'][~valid]) or np.any(records['radius_m'][~valid]) or np.any(records['support'][~valid]): raise ValueError('Invalid surface must be zero')


def packetize_surface(records,frame_id,chunk_points=4096):
    validate_records(records)
    if not 0<=frame_id<2**32 or not 1<=chunk_points<=65536: raise ValueError('Invalid packet config')
    for chunk,start in enumerate(range(0,max(1,len(records)),chunk_points)):
        part=records[start:start+chunk_points]
        yield SURFACE_HEADER.pack(b'SRF1',1,32,frame_id,chunk,len(part),32,int(start+chunk_points>=len(records)),0)+part.tobytes()


def export_surface(folder,records,xyz_m,rgb,coordinate_frame):
    folder=Path(folder);validate_records(records)
    if np.asarray(xyz_m).shape!=(len(records),3) or np.asarray(rgb).shape!=(len(records),3): raise ValueError('Surface cloud row mismatch')
    (folder/'surface.bin').write_bytes(records.tobytes())
    np.savez_compressed(folder/'surface.npz',**{key:records[key] for key in records.dtype.names})
    metadata=dict(schema='SRF1',record_stride=32,point_count=len(records),valid_normals=int(np.sum(records['flags']&1)),
        coordinate_frame=coordinate_frame,position_unit='metre',radius_unit='metre',normal_unit='unit vector',
        estimator='connected_huber_inverse_depth_plane_v2',robust_parameters=ROBUST_DEFAULTS,parameters=DEFAULTS,orientation='toward source observation camera',join='point_id; same order as corresponding point records',
        invalid='flags bit0=0: normal/radius/support=0; render as ordinary point or omit',
        caveat='Local connected robust plane with derivative fallback, not ground-truth normals. Circular footprint of observed pixels, not hole filling; voxel sparsity is not inflated.')
    (folder/'surface.json').write_text(json.dumps(metadata,indent=2)+'\n')
    header='ply\nformat ascii 1.0\nelement vertex %d\n'%len(records)
    for name in ['x','y','z','nx','ny','nz','radius_m']: header+='property float '+name+'\n'
    for name in ['red','green','blue']: header+='property uchar '+name+'\n'
    header+='property uint point_id\nproperty uint surface_flags\nend_header\n'
    with (folder/'surface.ply').open('w') as f:
        f.write(header)
        values=np.column_stack([xyz_m,records['normal'],records['radius_m'],rgb,records['point_id'],records['flags']])
        np.savetxt(f,values,fmt=['%.9g']*7+['%d']*5)
    return metadata


ROBUST_DEFAULTS=dict(window_radius=3,min_neighbors=8,irls_iterations=3,max_relative_residual=.025)


def estimate_surface(xyz_m,uv,image_size,method='robust',**options):
    """경계 연결 7x7 이웃 + spatial-weighted Huber IRLS inverse-depth plane.

    inverse depth is affine in pinhole pixels on a plane. No normal-estimation library.
    The legacy derivative remains available for a fixed-input baseline/fallback.
    """
    if method=='legacy': return estimate_surface_legacy(xyz_m,uv,image_size,**options)
    if method!='robust': raise ValueError('Unknown surface method')
    cfg=dict(DEFAULTS);cfg.update(options)
    base=estimate_surface_legacy(xyz_m,uv,image_size,**options) # validates input, edge flags/support
    xyz=np.asarray(xyz_m,float);uv=np.asarray(uv,int);n=len(xyz);w,h=map(int,image_size)
    base.update(plane_fitted=np.zeros(n,bool),neighborhood_count=np.zeros(n,np.uint16),relative_plane_residual=np.zeros(n,np.float32))
    if n<8: return base
    radius=ROBUST_DEFAULTS['window_radius'];side=2*radius+1
    dy,dx=np.mgrid[-radius:radius+1,-radius:radius+1];offset=np.column_stack([dx.ravel(),dy.ravel()])
    design=np.column_stack([offset/radius,np.ones(side*side)])
    # Derive the pinhole ray map from existing XYZ/UV; does not change coordinates.
    pixel=np.column_stack([uv,np.ones(n)])
    mapping=np.linalg.lstsq(pixel,xyz[:,:2]/xyz[:,2:3],rcond=None)[0]
    M=np.vstack([mapping.T,[0,0,1]])
    if np.linalg.cond(M)>1e10: return base
    invM=np.linalg.inv(M)
    grid=np.full((h,w),-1,int);grid[uv[:,1],uv[:,0]]=np.arange(n)
    accepted=np.zeros(n,bool);counts=np.zeros(n,np.uint16);residual=np.zeros(n,np.float32)
    for start in range(0,n,2048):
        end=min(n,start+2048);centers=xyz[start:end];cu=uv[start:end];m=len(centers)
        samples=cu[:,None,:]+offset[None,:,:]
        inside=(samples[:,:,0]>=0)&(samples[:,:,0]<w)&(samples[:,:,1]>=0)&(samples[:,:,1]<h)
        idx=grid[np.clip(samples[:,:,1],0,h-1),np.clip(samples[:,:,0],0,w-1)]
        present=(inside&(idx>=0)).reshape(m,side,side)
        z=xyz[np.maximum(idx,0),2].reshape(m,side,side)
        horizontal=present[:,:,:-1]&present[:,:,1:]&(np.abs(z[:,:,:-1]-z[:,:,1:])<=cfg['absolute_depth_jump_m']+cfg['relative_depth_jump']*np.minimum(z[:,:,:-1],z[:,:,1:]))
        vertical=present[:,:-1,:]&present[:,1:,:]&(np.abs(z[:,:-1,:]-z[:,1:,:])<=cfg['absolute_depth_jump_m']+cfg['relative_depth_jump']*np.minimum(z[:,:-1,:],z[:,1:,:]))
        connected=np.zeros_like(present);connected[:,radius,radius]=True
        for _ in range(side*side):
            expanded=connected.copy()
            expanded[:,:,1:] |= connected[:,:,:-1]&horizontal
            expanded[:,:,:-1] |= connected[:,:,1:]&horizontal
            expanded[:,1:,:] |= connected[:,:-1,:]&vertical
            expanded[:,:-1,:] |= connected[:,1:,:]&vertical
            if np.array_equal(expanded,connected):break
            connected=expanded
        mask=connected.reshape(m,-1);count=mask.sum(1);counts[start:end]=count
        spatial=np.exp(-np.sum(offset**2,axis=1)/(2*2.**2))
        weights=mask*spatial
        target=centers[:,2,None]/z.reshape(m,-1)-1
        beta=np.zeros((m,3));stable=np.zeros(m,bool)
        for iteration in range(ROBUST_DEFAULTS['irls_iterations']):
            H=np.einsum('nk,ki,kj->nij',weights,design,design)
            eigen=np.linalg.eigvalsh(H)
            stable=(count>=ROBUST_DEFAULTS['min_neighbors'])&(eigen[:,0]>eigen[:,-1]*.01)
            rhs=np.einsum('nk,ki,nk->ni',weights,design,target)
            H[~stable]=np.eye(3);rhs[~stable]=0
            beta=np.linalg.solve(H,rhs[...,None])[...,0]
            errors=target-beta@design.T
            # Robust scale only on graph-connected observations, not excluded depth layers.
            masked=np.where(mask,np.abs(errors),np.nan)
            scale=np.maximum(.001,1.4826*np.nanmedian(masked,axis=1))
            weights=mask*spatial*np.minimum(1,1.345*scale[:,None]/np.maximum(np.abs(errors),1e-12))
        rms=np.sqrt(np.sum(weights*errors**2,axis=1)/np.maximum(weights.sum(1),1e-12));residual[start:end]=rms
        # p^T [u,v,1] = inverse depth. n = -M^{-T}p, oriented toward camera.
        p=np.column_stack([beta[:,:2]/radius,1+beta[:,2]-np.sum(beta[:,:2]/radius*cu,axis=1)])/centers[:,2,None]
        normals=-(p@invM);normals/=np.maximum(np.linalg.norm(normals,axis=1),1e-30)[:,None]
        dot=np.sum(normals*centers,axis=1);normals[dot>0]*=-1
        cosine=-np.sum(normals*centers,axis=1)/np.linalg.norm(centers,axis=1)
        good=stable&(rms<=ROBUST_DEFAULTS['max_relative_residual'])&(cosine>=cfg['min_view_cos'])
        # Plane tangents at the measured center: no position displacement or hole filling.
        r=centers/centers[:,2,None]; rayu=M[:,0];rayv=M[:,1]
        denom=np.sum(normals*r,axis=1)
        du=centers[:,2,None]*(rayu-r*(normals@rayu/np.where(np.abs(denom)>1e-12,denom,1))[:,None])
        dv=centers[:,2,None]*(rayv-r*(normals@rayv/np.where(np.abs(denom)>1e-12,denom,1))[:,None])
        raw=.5*np.sqrt(np.sum(du*du+dv*dv,axis=1))
        edge=(base['flags'][start:end]&2)!=0
        limit=np.where(edge,.5*np.minimum(np.linalg.norm(du,axis=1),np.linalg.norm(dv,axis=1)),np.inf)
        sizes=np.minimum(np.minimum(raw,limit),cfg['max_radius_m'])
        good &= np.isfinite(sizes)&(sizes>0)
        where=np.flatnonzero(good)+start;accepted[where]=True
        base['normal'][where]=normals[good];base['radius_m'][where]=sizes[good]
        base['valid'][where]=True
        base['flags'][where]=1|(edge[good].astype(np.uint32)<<1)|((sizes[good]<raw[good]).astype(np.uint32)<<2)
        # Keep SRF1 support semantics: immediate depth-connected neighbour fraction.
        base['support'][where]=(horizontal[:,radius,radius-1].astype(int)+horizontal[:,radius,radius]+vertical[:,radius-1,radius]+vertical[:,radius,radius])[good]/4
    base['plane_fitted']=accepted;base['neighborhood_count']=counts;base['relative_plane_residual']=residual
    return base
