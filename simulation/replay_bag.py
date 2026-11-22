"""ROS1 bag replay through the navigation core, preserving original source and receipt times.
Offline preview only. Requires Noetic rosbag + sensor_msgs; does not need roscore.
"""
from pathlib import Path
import argparse
import json
import time
import numpy as np
from core import Navigator,Pose,config
from geometry import body_pose_from_lidar


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag');p.add_argument('--output',required=True)
    p.add_argument('--odom-reference',choices=['body','lidar'],default='lidar')
    p.add_argument('--allow-provisional-extrinsics',action='store_true')
    p.add_argument('--direction',nargs=3,type=float,help='Optional explicit synthetic body direction; not a measured acoustic source.')
    a=p.parse_args()
    import rosbag
    from sensor_msgs import point_cloud2
    output=Path(a.output);output.mkdir(parents=True,exist_ok=False)
    nav=Navigator(config());first=None;seq=0;rows=[]
    with rosbag.Bag(a.bag,'r') as bag:
        for topic,msg,receipt in bag.read_messages(topics=['/aft_mapped_to_init','/cloud_registered','/go2/navigation/direction','/go2/framework_v3/operator']):
            now=receipt.to_sec()
            if first is None:first=now
            mono=now-first
            if topic=='/aft_mapped_to_init':
                pp=msg.pose.pose.position;q=msg.pose.pose.orientation
                pose=Pose(msg.header.stamp.to_sec(),np.array([pp.x,pp.y,pp.z]),np.array([q.x,q.y,q.z,q.w]),msg.header.frame_id)
                if a.odom_reference=='lidar':pose=body_pose_from_lidar(pose,nav.cfg,a.allow_provisional_extrinsics)
                nav.pose_input(pose,now,mono)
            elif topic=='/cloud_registered':
                xyz=np.asarray(list(point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=False)),float)
                nav.cloud_input(xyz,msg.header.stamp.to_sec(),msg.header.frame_id,now,mono)
                if a.direction is not None:
                    seq+=1
                    nav.direction_input(dict(seq=seq,stamp_sec=msg.header.stamp.to_sec(),frame_id='body',kind='direction',vector=a.direction,confidence=1.,valid=True),now,mono)
            elif topic=='/go2/framework_v3/operator':
                try:nav.task.confirm(json.loads(msg.data),now,mono)
                except (ValueError,KeyError,TypeError):pass
            else:
                try:nav.direction_input(json.loads(msg.data),now,mono)
                except (ValueError,KeyError,TypeError) as e:nav.direction.invalidate(str(e))
            state=nav.step(now,mono)
            rows.append(dict(receipt=now,elapsed_s=mono,topic=topic,**state))
    with (output/'replay.jsonl').open('w') as f:
        for row in rows:f.write(json.dumps(row,allow_nan=False)+'\n')
    report=dict(mode='OFFLINE_OPEN_LOOP_BAG_REPLAY',bag=str(Path(a.bag).resolve()),
                records=len(rows),synthetic_direction=a.direction,odometry_reference=a.odom_reference,
                lidar_extrinsics_calibrated=nav.cfg['lidar_extrinsics_calibrated'],provisional_extrinsics=a.allow_provisional_extrinsics,
                note='The recorded trajectory is fixed; candidate commands do not move the recorded robot.',
                motion_commands_sent=0)
    (output/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))

if __name__=='__main__':main()
