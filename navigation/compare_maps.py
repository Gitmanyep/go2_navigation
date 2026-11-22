"""Compare world truth with exported perception, without feeding truth to the planner."""
from pathlib import Path
import argparse
import json
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run');p.add_argument('--output',required=True);a=p.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Polygon
    run=Path(a.run);cfg=json.loads((run/'config.json').read_text())
    names=['u_obstacle','sparse_lidar','manual_confirmation']
    fig,axes=plt.subplots(2,3,figsize=(15,9))
    for col,name in enumerate(names):
        folder=run/name;archive=np.load(folder/'maps.npz');m={k:archive[k] for k in archive.files}
        rows=[json.loads(line) for line in (folder/'trace.jsonl').read_text().splitlines()];last=rows[-1]
        summary=json.loads((folder/'summary.json').read_text());res=float(m['resolution'])
        origin=m['world_origin'];z=m['world_z'];extent=[origin[0],origin[0]+z.shape[1]*res,origin[1],origin[1]+z.shape[0]*res]
        truth=np.where(~m['world_ground'],0,np.where(m['world_obstacle'],2,1))
        axes[0,col].imshow(truth,origin='lower',extent=extent,cmap=ListedColormap(['#37445a','#edf1df','#ce5862']),vmin=0,vmax=2)
        source=m.get('source_position',np.array([1.,0.,2.3]) if name=='manual_confirmation' else np.array([4.2,0.,2.3]))
        axes[0,col].scatter(*source[:2],c='#8b3fc0',s=50)
        axes[0,col].set_title(f'{name} | truth (source z={source[2]:.1f} m)')
        grid=m['grid'];origin=m['origin'];extent=[origin[0],origin[0]+grid.shape[1]*res,origin[1],origin[1]+grid.shape[0]*res]
        colors=np.where(grid<0,0,np.where(grid>0,3,np.where(m['blocked'],2,1)))
        axes[1,col].imshow(colors,origin='lower',extent=extent,cmap=ListedColormap(['#505d6e','#f4f5f0','#e8bc72','#ce5862']),vmin=0,vmax=3)
        axes[1,col].set_title('Perceived map: gray=unknown, orange=margin',fontsize=11)
        position=np.array([last['x'],last['y']]);yaw=last['yaw'];rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
        for row in range(2):
            ax=axes[row,col];ax.plot(m['xy'][:,0],m['xy'][:,1],c='#236bd1',lw=2)
            for box in cfg['geometry_boxes']:
                lo,hi=box['minimum'],box['maximum']
                corners=np.array([[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]]])@rotation.T+position
                ax.add_patch(Polygon(corners,fill=False,edgecolor='#ca8900' if box['name']=='head_frame' else '#1769d2',lw=1.5))
            ax.set_aspect('equal');ax.set_xlabel('x (m)');ax.set_ylabel('y (m)')
        axes[1,col].text(.03,.03,f"task={summary['controller_state']}\ncompleted={summary['task_completed']}",transform=axes[1,col].transAxes,bbox=dict(facecolor='white',alpha=.9,edgecolor='none'))
    fig.suptitle('Go2 framework v3 | fixed 3D source + back-mounted lidar + unmeasured head-frame envelope',fontsize=14)
    fig.tight_layout(rect=[0,0,1,.96]);fig.savefig(a.output,dpi=130);plt.close(fig)

if __name__=='__main__':main()
