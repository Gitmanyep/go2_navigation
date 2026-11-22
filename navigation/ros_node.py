#!/usr/bin/env python3
"""ROS1 preview adapter: bounded ingress, one navigation worker, independent wall-clock publisher.
All velocity outputs are candidates in /go2/framework_v3. No robot command publisher.
"""
from pathlib import Path
import argparse
import json
import math
import time
import numpy as np
from core import Navigator,Pose,config
from simulation import Simulation,SCENARIOS
from geometry import body_pose_from_lidar
from runtime import PreviewRuntime


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped,TwistStamped
    from nav_msgs.msg import OccupancyGrid,Odometry,Path as NavPath
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs import point_cloud2
    from std_msgs.msg import String
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=['synthetic','preview'],default='synthetic')
    p.add_argument('--scenario',choices=SCENARIOS,default='flat_detour')
    p.add_argument('--config');p.add_argument('--mic-rotation-file')
    p.add_argument('--odom-reference',choices=['body','lidar'],default='lidar')
    p.add_argument('--allow-provisional-extrinsics',action='store_true')
    a=p.parse_args(rospy.myargv()[1:]);cfg=config(a.config)
    rospy.init_node('go2_framework_sim_v3');prefix='/go2/framework_v3'
    pub_status=rospy.Publisher(prefix+'/status',String,queue_size=1)
    pub_cmd=rospy.Publisher(prefix+'/candidate_velocity',TwistStamped,queue_size=1)
    pub_path=rospy.Publisher(prefix+'/path',NavPath,queue_size=1)
    pub_grid=rospy.Publisher(prefix+'/map',OccupancyGrid,queue_size=1)
    pub_odom=rospy.Publisher(prefix+'/simulated_odom',Odometry,queue_size=1) if a.mode=='synthetic' else None
    rotation=json.loads(Path(a.mic_rotation_file).read_text()) if a.mic_rotation_file else None
    nav=Navigator(cfg,rotation);sim=Simulation(a.scenario,cfg) if a.mode=='synthetic' else None
    adaptation_error=None

    def process(events):
        nonlocal adaptation_error
        now=rospy.Time.now().to_sec();mono=time.monotonic();active=sim.nav if sim else nav
        for msg in events['poses']:
            pp=msg.pose.pose.position;q=msg.pose.pose.orientation
            try:
                pose=Pose(msg.header.stamp.to_sec(),np.array([pp.x,pp.y,pp.z]),np.array([q.x,q.y,q.z,q.w]),msg.header.frame_id)
                if a.odom_reference=='lidar':pose=body_pose_from_lidar(pose,cfg,a.allow_provisional_extrinsics)
                nav.pose_input(pose,now,time.monotonic());adaptation_error=None
            except (ValueError,TypeError) as exc:
                adaptation_error=str(exc);nav.pose_deadline=-math.inf;nav.task.invalidate_observation()
        if 'cloud' in events:
            msg=events['cloud']
            try:
                if msg.width*msg.height>1000000:raise ValueError('oversize_cloud')
                xyz=np.asarray(list(point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=False)),float)
                nav.cloud_input(xyz,msg.header.stamp.to_sec(),msg.header.frame_id,rospy.Time.now().to_sec(),time.monotonic())
            except (ValueError,TypeError,KeyError) as exc:
                nav.fault='invalid_cloud:'+str(exc);nav.cloud_deadline=-math.inf
        if 'direction' in events:
            try:nav.direction_input(json.loads(events['direction'].data),rospy.Time.now().to_sec(),time.monotonic())
            except (ValueError,TypeError,KeyError) as exc:nav.direction.invalidate(str(exc))
        if 'error' in events:nav.fault=str(events['error']);nav.cloud_deadline=-math.inf
        if 'operator' in events:
            try:active.task.confirm(json.loads(events['operator'].data),rospy.Time.now().to_sec(),sim.t if sim else time.monotonic())
            except (ValueError,TypeError,KeyError):pass
        if sim:
            sim.wall0=now-sim.t
            state=sim.step() if not sim.done else dict(sim.state,reason='operator_confirmed_and_stopped',candidate=[0.,0.])
            control_mono=sim.t-cfg['simulation_dt_s']
            deadline=mono+min(active.cloud_deadline,active.pose_deadline,active.direction.deadline)-control_mono
        else:
            mono=time.monotonic();state=nav.step(rospy.Time.now().to_sec(),mono,cfg['simulation_dt_s'])
            control_mono=mono;deadline=min(nav.cloud_deadline,nav.pose_deadline,nav.direction.deadline)
        pose=active.pose
        state=dict(state,mode=a.mode,source='synthetic' if sim else 'external_pointlio_preview',
                   odometry_reference='body' if sim else a.odom_reference,
                   lidar_extrinsics_calibrated=cfg['lidar_extrinsics_calibrated'],
                   geometry_calibrated=cfg['geometry_calibrated'],adaptation_error=adaptation_error)
        snap=dict(state=state,frame=pose.frame if pose else 'camera_init')
        if pose:snap['pose']=dict(position=pose.position.tolist(),quaternion=pose.quaternion.tolist())
        if active.map:
            # Cached analysis is reused, not recomputed by the publisher.
            terrain=active.map.analyze(control_mono)
            snap['map']=dict(res=active.map.res,n=active.map.n,origin=active.map.origin.tolist(),grid=terrain['grid'].ravel().tolist())
        return snap,deadline

    runtime=PreviewRuntime(process,cfg['simulation_dt_s'],cfg['candidate_lease_s'])
    rospy.Subscriber(prefix+'/operator',String,lambda msg:runtime.submit('operator',msg),queue_size=1)
    if a.mode=='preview':
        # Callbacks only queue references. Point decoding/fusion/planning never hold
        # the publication lock; dropped clouds cannot accumulate a stale backlog.
        rospy.Subscriber('/aft_mapped_to_init',Odometry,lambda msg:runtime.submit('pose',msg),queue_size=30)
        rospy.Subscriber('/cloud_registered',PointCloud2,lambda msg:runtime.submit('cloud',msg),queue_size=1,buff_size=2**24)
        rospy.Subscriber('/go2/navigation/direction',String,lambda msg:runtime.submit('direction',msg),queue_size=1)
    runtime.start();rate_s=cfg['simulation_dt_s'];next_tick=time.monotonic();next_visual=0.
    try:
        while not rospy.is_shutdown():
            now=rospy.Time.now();snap=runtime.read();state=snap['state'];frame=snap.get('frame','camera_init')
            cmd=TwistStamped();cmd.header.stamp=now;cmd.header.frame_id='body'
            cmd.twist.linear.x,cmd.twist.angular.z=state['candidate'];pub_cmd.publish(cmd)
            pub_status.publish(String(data=json.dumps(state,allow_nan=False)))
            if time.monotonic()>=next_visual:
                path=NavPath();path.header.stamp=now;path.header.frame_id=frame
                for xy in state.get('path',[]):
                    point=PoseStamped();point.header=path.header;point.pose.position.x,point.pose.position.y=xy
                    point.pose.orientation.w=1;path.poses.append(point)
                pub_path.publish(path)
                if 'map' in snap:
                    m=snap['map'];grid=OccupancyGrid();grid.header=path.header;grid.info.resolution=m['res']
                    grid.info.width=grid.info.height=m['n'];grid.info.origin.position.x,grid.info.origin.position.y=m['origin']
                    grid.info.origin.orientation.w=1;grid.data=m['grid'];pub_grid.publish(grid)
                if sim and 'pose' in snap:
                    odom=Odometry();odom.header=path.header;odom.child_frame_id='sim_body'
                    odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=snap['pose']['position']
                    odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w=snap['pose']['quaternion']
                    pub_odom.publish(odom)
                next_visual=time.monotonic()+.5
            next_tick+=rate_s
            time.sleep(max(0.,min(rate_s,next_tick-time.monotonic())))
            if time.monotonic()-next_tick>rate_s:next_tick=time.monotonic()
    finally:runtime.close()

if __name__=='__main__':main()
