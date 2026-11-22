#!/usr/bin/env python3
"""Offline comparison of legacy row sorting and current sector voxel selection.

No ROS or robot connection. Timings describe this host and this input only.
"""
import argparse
import datetime
import json
import math
import os
from pathlib import Path
import platform
import sys
import time
from unittest.mock import patch

# Match the deployed display supervisor before importing NumPy.
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
os.environ['MKL_NUM_THREADS']='1'
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from perception.spatial_roi import SpatialROI
from perception.pipeline import Pose


def legacy_representatives(points,indexes,voxel):
    if not len(indexes):return indexes
    cells=np.floor(points[indexes]/voxel).astype(np.int64)
    _,keep=np.unique(cells,axis=0,return_index=True)
    return indexes[keep]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--points',type=Path,help='Optional recorded world XYZ .npy; no pickle')
    p.add_argument('--centre',type=float,nargs=3,default=[0.,0.,0.])
    p.add_argument('--yaw-deg',type=float,default=0.)
    p.add_argument('--repeats',type=int,default=15)
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    if not 1<=a.repeats<=100:p.error('repeats must be 1..100')
    if not all(math.isfinite(x) for x in [*a.centre,a.yaw_deg]):p.error('finite centre and yaw required')
    if a.points:
        if a.points.stat().st_size>100000000:p.error('recording exceeds 100 MB')
        points=np.load(a.points,allow_pickle=False)
        source='recorded_file_offline'
    else:
        rng=np.random.default_rng(130)
        base=rng.uniform([-3.,-3.,-.8],[3.,3.,2.],size=(10000,3))
        points=np.tile(base,(6,1))
        source='synthetic_60000_points_with_repeated_returns'
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=3 or not 3<=len(points)<=200000:
        p.error('expected 3..200000 XYZ points')
    if not np.isfinite(points).all() or np.any(np.abs(points)>1e6):p.error('invalid world XYZ')
    settings=json.loads((ROOT/'config/lidar_sector_preview.json').read_text())
    roi=SpatialROI(settings['spatial_roi'])
    half=math.radians(a.yaw_deg)/2
    pose=Pose(100.,np.array(a.centre),np.array([0.,0.,math.sin(half),math.cos(half)]),'camera_init')
    report=dict(source=source,host=platform.node(),architecture=platform.machine(),numpy=np.__version__,
                time=datetime.datetime.now().astimezone().isoformat(),argv=sys.argv,
                input_points=len(points),repeats=a.repeats,roi=roi.config,
                centre=a.centre,yaw_deg=a.yaw_deg,robot_connected=False,robot_commands_sent=0,
                live_data_age_verified=False,hardware_acceptance='pending_onboard_retest',timings={})
    outputs={}
    current=SpatialROI.representatives
    for name,fn in [('legacy_xyz_rows',legacy_representatives),('integer_voxel_keys',current)]:
        with patch.object(SpatialROI,'representatives',staticmethod(fn)):
            result,metrics=roi.select(points,pose)  # Warm up outside the measurements.
            timings=[]
            for _ in range(a.repeats):
                start=time.perf_counter();result,metrics=roi.select(points,pose)
                timings.append((time.perf_counter()-start)*1000.)
        outputs[name]=result
        report['timings'][name]=dict(median_ms=float(np.median(timings)),p95_ms=float(np.quantile(timings,.95)),
                                     output_points=len(result),samples_ms=timings)
    report['same_original_points_in_same_order']=bool(np.array_equal(outputs['legacy_xyz_rows'],outputs['integer_voxel_keys']))
    report['median_speedup_this_host_only']=report['timings']['legacy_xyz_rows']['median_ms']/report['timings']['integer_voxel_keys']['median_ms']
    report['selection_metrics']=metrics
    out=a.output or ROOT/'logs'/('sector_offline_'+datetime.datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir(parents=True,exist_ok=False)
    text=json.dumps(report,indent=2)+'\n';(out/'report.json').write_text(text)
    print(text);print('LOG',out,flush=True)
    return 0 if report['same_original_points_in_same_order'] else 1


if __name__=='__main__':raise SystemExit(main())
