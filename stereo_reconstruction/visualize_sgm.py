"""정답/기존/SGM 시차, 오차, 동일 시점의 실제 XYZRGB 비교. HTML 없음."""
import argparse
import json
import os
from pathlib import Path
import tempfile
os.environ.setdefault('MPLCONFIGDIR',os.path.join(tempfile.gettempdir(),'stereo-matplotlib-cache'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import cv2
from benchmark_sgm import read_pfm,read_calibration


def visualize(data,results,scene):
    src=Path(data)/scene; dest=Path(results)/scene
    left=cv2.imread(str(src/'im0.png'))[:,:,::-1]
    right=cv2.imread(str(src/'im1.png'))[:,:,::-1]
    gt=read_pfm(src/'disp0GT.pfm'); calib=read_calibration(src/'calib.txt')
    report=json.loads((dest/'metrics.json').read_text())
    attrs=[np.load(dest/m/'point_attributes.npz') for m in ('window','sgm')]
    clouds=[np.load(dest/m/'point_cloud.npy') for m in ('window','sgm')]
    plt.rcParams.update({'font.size':10,'figure.facecolor':'#f6f8fc','axes.facecolor':'#132033','axes.titleweight':'bold'})
    fig,axes=plt.subplots(1,2,figsize=(13,5))
    for ax,img,title in zip(axes,[left,right],['Left input','Right input']):
        ax.imshow(img); ax.set_title(title); ax.axis('off')
    fig.suptitle(f'Middlebury {scene} · original trainingQ pair',fontsize=17)
    fig.tight_layout(); fig.savefig(dest/'input_pair.png',dpi=150); plt.close(fig)
    fig,axes=plt.subplots(2,3,figsize=(16,9))
    axes[0,0].imshow(left); axes[0,0].set_title('Left image')
    cmap=plt.get_cmap('turbo').copy(); cmap.set_bad('#132033')
    error_cmap=plt.get_cmap('magma').copy(); error_cmap.set_bad('#132033')
    axes[1,0].imshow(np.ma.masked_invalid(gt),vmin=0,vmax=report['search_range'][1],cmap=cmap)
    axes[1,0].set_title('Ground-truth disparity · reference only')
    for col,(a,method) in enumerate(zip(attrs,('window','sgm')),1):
        valid=a['valid_mask']; d=a['disparity_map']
        m=report['methods'][method]; e=m['evaluation']['all_gt']
        axes[0,col].imshow(np.ma.masked_where(~valid,d),vmin=0,vmax=report['search_range'][1],cmap=cmap)
        axes[0,col].set_title(f'{method.upper()} · {m["points"]:,} points\nGT coverage {e["coverage_percent"]:.1f}%')
        error=np.ma.masked_where(~valid|~np.isfinite(gt),np.abs(d-gt))
        im=axes[1,col].imshow(error,vmin=0,vmax=3,cmap=error_cmap)
        axes[1,col].set_title(f'Accepted-point error · MAE {e["mae_px"]:.3f} px\nBad >2px: {e["bad2_percent"]:.2f}%')
    for ax in axes.flat: ax.axis('off')
    fig.suptitle(f'{scene} · fixed inputs and disparity range · no GT-assisted reconstruction',fontsize=18)
    fig.subplots_adjust(left=.025,right=.94,bottom=.09,top=.88,wspace=.08,hspace=.23)
    cax=fig.add_axes([.955,.14,.012,.29]); fig.colorbar(im,cax=cax,label='Absolute disparity error (px)')
    fig.text(.025,.025,'Dark areas are rejected / unknown pixels. Error colors share 0–3 px scale; GT values never guide predictions.',fontsize=11)
    fig.savefig(dest/'disparity_comparison.png',dpi=170); plt.close(fig)
    # 보정값을 그대로 사용한 GT cloud는 시각적 reference로만 표시한다.
    y,x=np.nonzero(np.isfinite(gt)&(gt>0)); d=gt[y,x]
    K=calib['cam0']; offset=calib['cam1'][0,2]-K[0,2]
    z=K[0,0]*calib['baseline']/(d+offset)
    truth=np.column_stack(((x-K[0,2])*z/K[0,0],(y-K[1,2])*z/K[1,1],z,left[y,x]))
    # 표시만 공통 범위로 crop; 출력 점을 제거하거나 GT로 필터링하지 않음.
    pooled=np.concatenate([c[:,:3] for c in clouds])
    lo,hi=np.percentile(pooled,[.5,99.5],axis=0)
    pad=(hi-lo)*.03; lo-=pad; hi+=pad
    fig=plt.figure(figsize=(16,10),facecolor='#f6f8fc')
    for row,(elev,azim) in enumerate([(0,-90),(18,-65)]):
        for col,(c,title) in enumerate(zip([truth]+clouds,['GT reference','Window','Custom SGM'])):
            inside=((c[:,:3]>=lo)&(c[:,:3]<=hi)).all(1)
            shown=c[inside]
            ax=fig.add_subplot(2,3,row*3+col+1,projection='3d'); ax.set_facecolor('#f6f8fc')
            ax.scatter(shown[:,0]/1000,shown[:,2]/1000,-shown[:,1]/1000,c=shown[:,3:]/255,s=.35,linewidths=0,depthshade=False)
            ax.set_xlim(lo[0]/1000,hi[0]/1000); ax.set_ylim(lo[2]/1000,hi[2]/1000); ax.set_zlim(-hi[1]/1000,-lo[1]/1000)
            span=hi-lo; ax.set_box_aspect((span[0],span[2],span[1]))
            ax.view_init(elev=elev,azim=azim)
            ax.set_title(f'{title} · {len(c):,} points\n{len(shown):,} inside display bounds')
            ax.set_xlabel('X (m)'); ax.set_ylabel('Z / depth (m)'); ax.set_zlabel('-Y / up (m)')
            ax.tick_params(labelsize=8)
            if row==0: ax.set_yticks([])
    fig.suptitle(f'{scene} · reconstructed shape, same camera views and bounds',fontsize=18)
    fig.subplots_adjust(left=.01,right=.98,bottom=.075,top=.9,wspace=.02,hspace=.16)
    fig.text(.025,.025,'Display bounds: pooled prediction 0.5–99.5 percentiles +3% padding. All points remain in PLY/JSON. No mesh or hole filling.',fontsize=10)
    fig.savefig(dest/'shape_comparison.png',dpi=170); plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,6))
    conf=attrs[1]['confidence_map']
    im=ax.imshow(np.ma.masked_where(~attrs[1]['valid_mask'],conf),vmin=0,vmax=1,cmap='viridis')
    ax.set_title(f'{scene} · accepted SGM confidence (heuristic, not probability)'); ax.axis('off')
    fig.colorbar(im,ax=ax,shrink=.8); fig.tight_layout(); fig.savefig(dest/'confidence.png',dpi=150); plt.close(fig)
    print('Saved visualizations:',dest,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data-root',required=True); p.add_argument('--results',required=True)
    p.add_argument('--scenes',nargs='+',default=['Motorcycle','Pipes','Adirondack'])
    a=p.parse_args()
    for scene in a.scenes: visualize(a.data_root,a.results,scene)
