"""Inspect a supplied PCD using the same 2.5D terrain analyzer; no navigation claim."""
from pathlib import Path
import argparse
import json
import numpy as np
from core import TerrainMap,config
from evidence import pcd_xyz,digest


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('pcd');p.add_argument('--stride',type=int,default=8)
    p.add_argument('--centre',nargs=2,type=float)
    p.add_argument('--output',required=True);a=p.parse_args()
    cfg=config();points=pcd_xyz(a.pcd,a.stride)
    if not len(points):raise ValueError('empty_pcd')
    if a.centre is None:
        # Large mountain scans can have an empty median location. Choose the
        # densest coarse tile explicitly for inspection, never as a robot pose.
        tiles,counts=np.unique(np.floor(points[:,:2]/5.).astype(int),axis=0,return_counts=True)
        centre=(tiles[np.argmax(counts)]+.5)*5.
    else:
        centre=np.asarray(a.centre)
    terrain=TerrainMap(cfg,centre);terrain.integrate(points,[*centre,0.],0.)
    t=terrain.analyze(0.)
    output=Path(a.output);output.mkdir(parents=True,exist_ok=False)
    report=dict(mode='OFFLINE_UNCALIBRATED_PCD_INSPECTION',source=str(Path(a.pcd).resolve()),
                sha256=digest(a.pcd),stride=a.stride,sampled_points=len(points),centre=centre.tolist(),
                observed_cells=int(t['known'].sum()),free_cells=int((t['grid']==0).sum()),
                blocked_cells=int((t['grid']==100).sum()),unknown_cells=int((t['grid']<0).sum()),
                note='Unknown provenance, datum, extrinsics and ground truth. No start/goal or ability validation.',
                executable=False,motion_commands_sent=0)
    np.savez_compressed(output/'terrain.npz',**t,origin=terrain.origin,resolution=terrain.res)
    (output/'summary.json').write_text(json.dumps(report,indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(13,4))
    for ax,key,title in zip(axes,['elevation','slope','grid'],['Elevation (m)','Slope (deg)','Unknown=-1 / obstacle=100']):
        image=np.where(t['known'],t[key],np.nan) if key!='grid' else t[key]
        im=ax.imshow(image,origin='lower');fig.colorbar(im,ax=ax);ax.set_title(title)
    fig.suptitle('Historical PCD inspection | single-surface assumption | uncalibrated')
    fig.tight_layout();fig.savefig(output/'terrain.png',dpi=130);plt.close(fig)
    print(json.dumps(report))

if __name__=='__main__':main()
