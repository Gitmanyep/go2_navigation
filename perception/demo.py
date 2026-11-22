#!/usr/bin/env python3
"""Deterministic synthetic P0/P1 exercise. Never connects to ROS or a robot."""
import argparse
import copy
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from perception.pipeline import MappingEngine,load_settings,expired_snapshot


def run(settings_path,output):
    settings,cal,cfg,_=load_settings(settings_path)
    settings=copy.deepcopy(settings);cal=copy.deepcopy(cal);cfg=copy.deepcopy(cfg)
    # These are exact declared dimensions of the SYNTHETIC scene, not robot calibration.
    cal.update(lidar_extrinsics_calibrated=True,geometry_calibrated=True,measurement_record='synthetic_test_scene',
        reference_frame='demo_lidar',lidar_position_body_m=[0.,0.,.15],lidar_rotation_body=np.eye(3).tolist(),
        body_reference_height_m=.3,geometry_boxes=cfg['geometry_boxes'])
    for key in ('lidar_extrinsics_calibrated','geometry_calibrated','lidar_position_body_m','lidar_rotation_body','body_reference_height_m'):cfg[key]=cal[key]
    engine=MappingEngine(settings,cal,cfg);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    for index in range(35):
        t=index*.1;stamp=100.+t;x=index*.08
        engine.pose_input(stamp,[x,0.,.45],[0.,0.,0.,1.],settings['fixed_frame'],'demo_lidar',stamp,t)
        yy,xx=np.mgrid[-1.8:1.81:.15,-1.8:1.81:.15]
        floor=np.column_stack([xx.ravel()+x+.02,yy.ravel()+.02,np.zeros(xx.size)])
        wall=np.array([[3.5,y,z] for y in np.arange(-.4,.41,.08) for z in np.arange(.2,.81,.08)])
        wall=wall[np.abs(wall[:,0]-x)<=1.8]
        engine.cloud_input(stamp,np.r_[floor,wall],settings['fixed_frame'],stamp,t)
    snap=engine.snapshot(stamp,t);stale=expired_snapshot(snap,stamp+.6,t+.6,settings)
    np.savez_compressed(output/'map.npz',occupancy=snap['occupancy'],origin=snap['origin'],elevation=snap['elevation_points'],obstacles=snap['obstacle_points'])
    report=dict(source='synthetic',hardware_acceptance='pending',motion_commands_sent=0,map_data_valid=snap['valid'],
        accepted_clouds=engine.accepted_clouds,known_cells=int((snap['occupancy']>=0).sum()),unknown_cells=int((snap['occupancy']<0).sum()),
        elevation_points=len(snap['elevation_points']),obstacle_points=len(snap['obstacle_points']),rolling_origin=snap['origin'].tolist(),
        stale_map_unknown=bool((stale['occupancy']==-1).all()),stale_points_empty=len(stale['obstacle_points'])==0,
        source_stamp_preserved=stale['source_stamp']==snap['source_stamp'],
        static_obstacle_world_x_correct=bool(len(snap['obstacle_points']) and np.allclose(snap['obstacle_points'][:,0],3.5)),audit=snap['audit'])
    report['passed']=bool(report['map_data_valid'] and report['known_cells'] and report['unknown_cells'] and report['obstacle_points'] and
                          report['stale_map_unknown'] and report['stale_points_empty'] and report['source_stamp_preserved'] and report['static_obstacle_world_x_correct'])
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
    return 0 if report['passed'] else 1


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--settings',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();return run(a.settings,a.output)

if __name__=='__main__':raise SystemExit(main())
