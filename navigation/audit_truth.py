"""Independent swept envelope audit of exported traces, including walls and head frame.
No truth data is returned to navigation. Wall surfaces sampled at <=5 cm.
"""
from pathlib import Path
import argparse
import json
import numpy as np
from scipy import ndimage
from core import Pose,config
from geometry import Geometry
from simulation import World,quaternion


def wall_surfaces(world):
    boundary=world.obstacle&~ndimage.binary_erosion(world.obstacle)
    y,x=np.where(boundary);centres=world.xy[y,x];base=world.z[y,x]
    parts=[]
    for ox in np.linspace(-world.res/2,world.res/2,4):
        for oy in np.linspace(-world.res/2,world.res/2,4):
            for z in np.linspace(.04,.60,13):
                parts.append(np.column_stack([centres+[ox,oy],base+z]))
    return np.concatenate(parts+[world.overhang_points]) if parts else world.overhang_points


def interpolate_pose(previous, pose, fraction):
    delta=np.asarray(pose)-previous
    delta[2]=(delta[2]+np.pi)%(2*np.pi)-np.pi
    sample=np.asarray(previous)+fraction*delta
    sample[2]=(sample[2]+np.pi)%(2*np.pi)-np.pi
    return sample


def audit(run):
    run=Path(run);cfg=config(run/'config.json');results=[]
    for folder in sorted(run.iterdir()):
        if not (folder/'summary.json').is_file():continue
        summary=json.loads((folder/'summary.json').read_text())
        world=World(summary['scenario'],dict(cfg,layout_seed=summary['seed']))
        geometry=Geometry(cfg);points=wall_surfaces(world)
        hits=[];previous=None
        for line in (folder/'trace.jsonl').read_text().splitlines():
            row=json.loads(line)
            # Adjacent trace intervals are swept at <=2 cm or 3 degrees.
            pose=np.array([row['x'],row['y'],row['yaw']])
            steps=1 if previous is None else max(1,int(np.ceil(np.linalg.norm(pose[:2]-previous[:2])/.02)),int(np.ceil(abs((pose[2]-previous[2]+np.pi)%(2*np.pi)-np.pi)/np.deg2rad(3))))
            for fraction in np.linspace(0,1,steps+1):
                sample=pose if previous is None else interpolate_pose(previous,pose,fraction)
                z,p,r=world.height_angles(sample[:2],sample[2])
                body=Pose(0.,np.array([*sample[:2],z+cfg['body_reference_height_m']]),quaternion(r,p,sample[2]))
                # Crop truth points before exact box tests; this is a scoring-only optimization.
                nearby=points[np.linalg.norm(points[:,:2]-body.position[:2],axis=1)<1.1]
                hit=geometry.collision(nearby,body.position,body.rotation)
                if hit:
                    hits.append(dict(t=row['t'],part=hit));break
            previous=pose
        results.append(dict(scenario=summary['scenario'],seed=summary['seed'],collision_samples=len(hits),first_hits=hits[:3]))
    report=dict(all_clear=all(r['collision_samples']==0 for r in results),results=results,
                note='Independent point-sampled wall and overhead envelope audit; does not verify legs, contact or dynamics.')
    (run/'truth_geometry_audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run');a=p.parse_args();audit(a.run)
