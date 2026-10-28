"""두 출력 cloud의 동일 축/시점 비교 그림. Matplotlib는 시각화에만 사용."""
import argparse,os,json
from pathlib import Path
import tempfile
os.environ.setdefault('MPLCONFIGDIR',os.path.join(tempfile.gettempdir(),'stereo-matplotlib-cache'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from image_io import load_image


def compare(sparse_dir,dense_dir,left_path,output):
    sparse_dir,dense_dir,output=Path(sparse_dir),Path(dense_dir),Path(output)
    output.mkdir(parents=True,exist_ok=True)
    clouds=[np.load(folder/'point_cloud.npy') for folder in (sparse_dir,dense_dir)]
    if any(len(c)==0 for c in clouds): raise ValueError('두 cloud 모두 점이 있어야 비교할 수 있습니다.')
    with np.load(sparse_dir/'reprojection_data.npz') as a: sp=a['accepted_left']
    with np.load(dense_dir/'dense_data.npz') as a: dp=a['points_left']
    source=load_image(left_path)[:,:,::-1]
    xyz=np.concatenate([c[:,:3] for c in clouds]); lo=xyz.min(0); hi=xyz.max(0)
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.titleweight':'bold',
                         'text.color':'#172033','axes.labelcolor':'#526078'})
    fig=plt.figure(figsize=(15,11.5),facecolor='#f8fafc')
    gs=fig.add_gridspec(2,2,left=.055,right=.95,top=.82,bottom=.08,wspace=.14,hspace=.23)
    fig.text(.055,.953,'DENSE STEREO · SAME REAL IMAGE PAIR',fontsize=12,color='#2563eb',weight='bold')
    fig.text(.055,.910,f'{len(clouds[0]):,} → {len(clouds[1]):,} points  |  {len(clouds[1])/len(clouds[0]):.1f}× more',fontsize=25,weight='bold')
    fig.text(.055,.873,'Same calibration, image pair, depth limits, axes and viewpoint. No point duplication or surface filling.',fontsize=11,color='#526078')
    for col,(cloud,pixels,name) in enumerate(zip(clouds,[sp,dp],['ORB features','Custom dense Census matching'])):
        ax=fig.add_subplot(gs[0,col]); ax.imshow(source)
        ax.scatter(pixels[:,0],pixels[:,1],s=10 if col==0 else .12,c='#00d5ff',alpha=.9 if col==0 else .65,linewidths=0)
        ax.set_title(f'{name} · {len(cloud):,} points on the left image',loc='left',pad=12); ax.axis('off')
        ax=fig.add_subplot(gs[1,col],projection='3d'); ax.set_facecolor('#f8fafc')
        ax.scatter(cloud[:,0],cloud[:,2],-cloud[:,1],c=cloud[:,3:]/255.,
                   s=12 if col==0 else .6,edgecolors='#64748b' if col==0 else 'none',linewidths=.3,depthshade=False)
        ax.set_title(f'{name} · 3D sampled RGB',loc='left',pad=8)
        ax.set_xlim(lo[0]-1,hi[0]+1); ax.set_ylim(lo[2]-1,hi[2]+1); ax.set_zlim(-hi[1]-1,-lo[1]+1)
        ax.set_box_aspect((hi[0]-lo[0]+2,hi[2]-lo[2]+2,hi[1]-lo[1]+2))
        ax.view_init(elev=18,azim=-73)
        ax.set_xlabel('X · right'); ax.set_ylabel('Z · depth'); ax.set_zlabel('−Y · up')
        ax.tick_params(labelsize=8)
        for axis in (ax.xaxis,ax.yaxis,ax.zaxis):
            axis.pane.fill=False; axis._axinfo['grid']['color']=(.78,.82,.88,.5)
    fig.text(.055,.027,'All exported points shown. Units: board squares. Textureless/occluded pixels remain holes; dense reprojection error is not an accuracy score.',fontsize=9,color='#526078')
    fig.savefig(output/'sparse_vs_dense.png',dpi=180,facecolor=fig.get_facecolor()); plt.close(fig)
    # Dense 정면/사선 시점. 점을 재샘플링하지 않는다.
    cloud=clouds[1]
    fig=plt.figure(figsize=(13,6),facecolor='#f8fafc')
    for i,(elev,azim) in enumerate([(0,-90),(18,-65)]):
        ax=fig.add_subplot(1,2,i+1,projection='3d'); ax.set_facecolor('#f8fafc')
        ax.scatter(cloud[:,0],cloud[:,2],-cloud[:,1],c=cloud[:,3:]/255.,s=.9,depthshade=False)
        ax.set_box_aspect((hi[0]-lo[0],hi[2]-lo[2],hi[1]-lo[1])); ax.view_init(elev=elev,azim=azim)
        if i==0: ax.set_yticks([])
        ax.set_xlabel('X'); ax.set_ylabel('Z (depth)'); ax.set_zlabel('−Y (up)')
        ax.set_title('Front view' if i==0 else 'Perspective view')
    fig.suptitle(f'Dense point cloud · {len(cloud):,} real matched pixels · no mesh',fontsize=16,weight='bold')
    fig.savefig(output/'dense_views.png',dpi=180,facecolor=fig.get_facecolor()); plt.close(fig)
    print('Saved comparison figures:',output)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sparse',required=True); p.add_argument('--dense',required=True)
    p.add_argument('--left',required=True); p.add_argument('--output',required=True)
    a=p.parse_args(); compare(a.sparse,a.dense,a.left,a.output)
