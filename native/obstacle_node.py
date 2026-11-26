"""ROS-only obstacle route PREVIEW: no SDK, cmd_vel or actuator publisher."""
import argparse
from collections import deque
import json
from pathlib import Path
import sys
import threading
import time
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from native.obstacle_plan import ObstaclePreview
from perception.ros_messages import pointcloud_xyz


def main():
    import rospy
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry, Path as NavPath
    from geometry_msgs.msg import PoseStamped, PointStamped, Point, TwistStamped
    from visualization_msgs.msg import Marker
    from std_msgs.msg import String, Header, ColorRGBA
    p=argparse.ArgumentParser()
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--settings',type=Path,default=ROOT/'config/native_obstacle_plan.json')
    a=p.parse_args(rospy.myargv()[1:])
    settings=json.loads(a.settings.read_text());engine=ObstaclePreview(settings)
    rospy.init_node('go2_obstacle_only_path_preview')
    prefix='/go2/obstacle_preview/'
    pubs={k:rospy.Publisher(prefix+k,cls,queue_size=1) for k,cls in [
        ('path',NavPath),('candidate_velocity',TwistStamped),('map',Marker),('label',Marker),('goal',Marker),('footprint',Marker),('status',String)]}
    lock=threading.Lock();latest={};events=deque(maxlen=20);metadata={};drops={}
    a.output.mkdir(parents=True,exist_ok=True)
    (a.output/'obstacle_settings.json').write_text(json.dumps(settings,indent=2)+'\n')
    log=(a.output/'obstacle_status.jsonl').open('w',buffering=1)
    inputs=(a.output/'obstacle_inputs.jsonl').open('w',buffering=1)
    def submit(kind,msg):
        item=(kind,msg,time.monotonic())
        with lock:
            if kind in ('odom','cloud','metadata','direction'):
                if kind in latest:drops[kind]=drops.get(kind,0)+1
                latest[kind]=item
            else:events.append(item)
    subs=[
        rospy.Subscriber(settings['topics']['cloud'],PointCloud2,lambda m:submit('cloud',m),queue_size=1,buff_size=2**24),
        rospy.Subscriber('/go2/native/odom',Odometry,lambda m:submit('odom',m),queue_size=1),
        rospy.Subscriber('/go2/native/status',String,lambda m:submit('metadata',m),queue_size=1),
        rospy.Subscriber('/go2/obstacle_preview/floor_input',PointStamped,lambda m:submit('floor',m),queue_size=1),
        rospy.Subscriber('/go2/obstacle_preview/goal_input',PoseStamped,lambda m:submit('goal',m),queue_size=1),
        rospy.Subscriber('/go2/navigation/direction',String,lambda m:submit('direction',m),queue_size=1)]
    next_report=0.;last_input_error=None;direction_seq={}
    def input_one(item):
        nonlocal last_input_error,metadata
        kind,m,mono=item
        try:
            if kind=='odom':
                pp,q=m.pose.pose.position,m.pose.pose.orientation
                engine.pose_input(m.header.stamp.to_sec(),[pp.x,pp.y,pp.z],[q.x,q.y,q.z,q.w],m.header.frame_id,m.child_frame_id,mono)
            elif kind=='cloud':
                if len(m.data)>32000000:raise ValueError('oversized_voxel_message')
                info=metadata.get('details',{}).get('voxel_decoded',{})
                size=info.get('resolution_m') if info.get('frame')==m.header.frame_id else None
                engine.cloud_input(m.header.stamp.to_sec(),pointcloud_xyz(m),m.header.frame_id,mono,size)
            elif kind=='metadata':
                if len(m.data)<100000:metadata=json.loads(m.data)
            elif kind=='floor':
                engine.set_floor(m.point.z,m.header.frame_id,[m.point.x,m.point.y])
            elif kind=='goal':
                engine.set_goal([m.pose.position.x,m.pose.position.y],m.header.frame_id)
            elif kind=='direction':
                if len(m.data)>100000:raise ValueError('oversized_direction')
                d=json.loads(m.data)
                if d.get('source_id')!='operator' or d.get('frame_id')!='body' or d.get('kind')!='direction':
                    raise ValueError('operator_body_direction_required')
                age=rospy.Time.now().to_sec()-float(d['stamp_sec'])
                if age<-.05 or age>settings['direction_timeout_s']:raise ValueError('direction_timestamp_invalid')
                key=(d['source_id'],d['session_id']);seq=d['seq']
                if type(seq) is not int or seq<=direction_seq.get(key,-1):raise ValueError('direction_replay')
                if len(direction_seq)>100:raise ValueError('too_many_direction_sessions')
                direction_seq[key]=seq
                engine.set_direction(d['vector'] if d.get('valid') else [0,0,0],mono-max(0.,age))
            last_input_error=None
        except (ValueError,KeyError,TypeError) as exc:
            last_input_error=kind+':'+str(exc)
            if kind=='cloud':engine.voxel_clock.invalidate(last_input_error)
            if kind=='odom':engine.pose_clock.invalidate(last_input_error)
            if kind=='goal' or (kind=='direction' and engine.direction is not None):
                engine.goal=engine.direction=None
        if kind in ('floor','goal','direction'):
            inputs.write(json.dumps(dict(kind=kind,mono=mono,error=last_input_error,floor_z=engine.floor_z,
                area_centre=engine.area_centre.tolist() if engine.area_centre is not None else None,
                goal=engine.goal.tolist() if engine.goal is not None else None))+'\n')
    try:
        while not rospy.is_shutdown():
            start=time.monotonic()
            with lock:
                batch=dict(latest);latest.clear();commands=list(events);events.clear();dropped=dict(drops)
            for k in ('odom','metadata'):
                if k in batch:input_one(batch[k])
            for e in commands:
                if e[0]=='floor':input_one(e)
            if 'cloud' in batch:input_one(batch['cloud'])
            if 'direction' in batch:input_one(batch['direction'])
            for e in commands:
                if e[0]!='floor':input_one(e)
            snap=engine.snapshot(time.monotonic())
            header=Header(frame_id=snap['frame'],stamp=rospy.Time.from_sec(snap['source_stamp']))
            path=NavPath(header=header)
            for xyz in snap['path']:
                pose=PoseStamped(header=header);pose.pose.orientation.w=1.
                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=xyz
                path.poses.append(pose)
            pubs['path'].publish(path)
            velocity=TwistStamped();velocity.header.frame_id=settings['body_frame'];velocity.header.stamp=header.stamp
            velocity.twist.linear.x,velocity.twist.angular.z=snap['candidate']
            pubs['candidate_velocity'].publish(velocity)
            marker=Marker(header=header,ns='obstacle_only_assumed_flat_area',type=Marker.CUBE_LIST)
            marker.pose.orientation.w=1.;marker.scale.x=marker.scale.y=snap['resolution']*.95;marker.scale.z=.015
            marker.color.a=1.;marker.action=Marker.ADD if snap['valid'] else Marker.DELETE
            if snap['valid']:
                y,x=np.nonzero(snap['grid']>=0)
                for cy,cx in zip(y,x):
                    xy=snap['origin']+(np.array([cx,cy])+.5)*snap['resolution']
                    marker.points.append(Point(xy[0],xy[1],snap['floor_z']+engine.voxel_size*.5+.01))
                    marker.colors.append(ColorRGBA(1.,.05,.05,1.) if snap['grid'][cy,cx]==100 else ColorRGBA(1.,.55,.05,.55) if snap['grid'][cy,cx]==50 else ColorRGBA(.20,.30,.42,.15))
            pubs['map'].publish(marker)
            label=Marker(header=header,ns='obstacle_preview_label',type=Marker.TEXT_VIEW_FACING)
            label.pose.orientation.w=1.;label.scale.z=.12;label.color.r=label.color.g=label.color.a=1.
            if snap['pose']:
                label.pose.position.x,label.pose.position.y,z=snap['pose']['position'];label.pose.position.z=z+.7
            label.text='OBSTACLE ONLY | ASSUMED FLAT | NO CONTROL\n'+snap['reason']+'   v %.2f   w %.2f'%tuple(snap['candidate'])
            pubs['label'].publish(label)
            goal=Marker(header=header,ns='obstacle_preview_goal',type=Marker.SPHERE)
            goal.action=Marker.ADD if snap['goal'] is not None and snap['valid'] else Marker.DELETE
            goal.pose.orientation.w=1.;goal.scale.x=goal.scale.y=goal.scale.z=.15;goal.color.r=goal.color.g=goal.color.a=1.
            if goal.action==Marker.ADD:
                goal.pose.position.x,goal.pose.position.y=snap['goal'];goal.pose.position.z=snap['floor_z']+.08
            pubs['goal'].publish(goal)
            footprint=Marker(header=header,ns='assumed_safe_start_footprint',type=Marker.LINE_STRIP)
            footprint.pose.orientation.w=1.;footprint.scale.x=.025;footprint.color.g=footprint.color.a=1.
            footprint.action=Marker.DELETE
            if snap['pose'] and snap['floor_z'] is not None:
                from scipy.spatial.transform import Rotation
                rotation=Rotation.from_quat(snap['pose']['quaternion']).as_matrix()[:2,:2]
                length=settings.get('robot_length_m',.67)/2;width=settings.get('robot_width_m',.36)/2
                for xy in ((length,width),(length,-width),(-length,-width),(-length,width),(length,width)):
                    pt=rotation@np.asarray(xy)+snap['pose']['position'][:2]
                    footprint.points.append(Point(pt[0],pt[1],snap['floor_z']+.12))
                footprint.action=Marker.ADD
            pubs['footprint'].publish(footprint)
            status={k:v for k,v in snap.items() if k not in ('grid','pose','origin','path')}
            status.update(path_points=len(snap['path']),input_error=last_input_error,queue_drops=dropped,
                          received_voxels=engine.received_points,excluded_floor_voxels=engine.filtered_floor_points,
                          excluded_above_envelope_voxels=engine.out_of_envelope_points,
                          floor_z=engine.floor_z,area_centre=engine.area_centre.tolist() if engine.area_centre is not None else None)
            pubs['status'].publish(String(data=json.dumps(status,allow_nan=False)))
            if start>=next_report:
                log.write(json.dumps(status,allow_nan=False)+'\n')
                print('OBSTACLE_PREVIEW',snap['reason'],'path',len(snap['path']),'candidate',snap['candidate'],flush=True)
                next_report=start+1.
            time.sleep(max(.01,.2-(time.monotonic()-start)))
    finally:
        log.close();inputs.close()


if __name__=='__main__':main()
