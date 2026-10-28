"""KITTI 실제 누적 map과 포즈/증분 수 비교. Matplotlib는 시각화만 수행."""
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
from point_stream import RECORD


def draw(run):
    root=Path(run); summary=json.loads((root/'summary.json').read_text())
    points=np.fromfile(root/'map_points.bin',dtype=RECORD)
    frames=summary['frames']; poses=np.array([f['T_world_from_observation_camera'] for f in frames])
    centers=poses[:,:3,3]
    fig=plt.figure(figsize=(15,10),facecolor='#f7f9fc')
    grid=fig.add_gridspec(2,2,left=.06,right=.96,top=.88,bottom=.09,hspace=.32,wspace=.22)
    ax=fig.add_subplot(grid[0,0]); ax.imshow(plt.imread(root/f'frames/{frames[0]["frame_id"]:010d}/left_rectified.png'))
    ax.set_title('Actual KITTI frame 0 · left, resized + rectified'); ax.axis('off')
    ax=fig.add_subplot(grid[0,1])
    ax.scatter(points['xyz'][:,0],points['xyz'][:,2],c=points['rgba'][:,:3]/255,s=.35,linewidths=0)
    ax.plot(centers[:,0],centers[:,2],'-o',color='#ed3b64',markersize=5,lw=2)
    for f,p in zip(frames,centers): ax.annotate(str(f['frame_id']),(p[0],p[2]),xytext=(6,0),textcoords='offset points',color='#b4143a')
    ax.set_xlabel('World X / right (m)'); ax.set_ylabel('World Z / forward (m)')
    ax.set_title('Accumulated map · top view + OXTS camera path'); ax.set_aspect('equal'); ax.grid(alpha=.2)
    ax=fig.add_subplot(grid[1,0],projection='3d')
    xyz=points['xyz']; lo=xyz.min(0); hi=xyz.max(0)
    ax.scatter(xyz[:,0],xyz[:,2],-xyz[:,1],c=points['rgba'][:,:3]/255,s=.3,linewidths=0,depthshade=False)
    ax.plot(centers[:,0],centers[:,2],-centers[:,1],color='#ed3b64',marker='o',lw=2)
    ax.set_box_aspect((hi[0]-lo[0],hi[2]-lo[2],hi[1]-lo[1])); ax.view_init(24,-65)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Z / forward (m)'); ax.set_zlabel('-Y / up (m)')
    ax.set_zticks(np.linspace(-hi[1],-lo[1],3)); ax.tick_params(labelsize=8)
    ax.set_title(f'RGB 3D map · all {len(points):,} exported points')
    ax=fig.add_subplot(grid[1,1]); index=np.arange(len(frames)); width=.25
    for offset,key,label,color in [(-1,'input_points','Depth-valid observations','#b6c2d2'),(0,'within_frame_voxels','Frame voxel representatives','#7a9edf'),(1,'new_points','New voxels sent','#18745d')]:
        ax.bar(index+offset*width,[f[key] for f in frames],width,label=label,color=color)
    ax.set_xticks(index,[str(f['frame_id']) for f in frames]); ax.set_xlabel('KITTI frame ID'); ax.set_ylabel('Points')
    ax.set_title('Only new voxel observations are transmitted'); ax.legend(fontsize=8); ax.grid(axis='y',alpha=.2)
    fig.suptitle(f'KITTI Raw · custom SGM → OXTS pose → {summary["config"]["voxel_size_m"]:.2f} m voxels → STP1',fontsize=19,weight='bold')
    fig.text(.06,.032,'World = frame-0 left camera. Pose is supplied GPS/IMU, not estimated SLAM. No dynamic-object removal, hole filling or meshing.',fontsize=10)
    fig.savefig(root/'accumulated_map.png',dpi=170); plt.close(fig)
    print('Saved',root/'accumulated_map.png',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--run',required=True); a=p.parse_args(); draw(a.run)
