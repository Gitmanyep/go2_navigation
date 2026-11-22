#!/usr/bin/env python3
"""Noetic read-only P0 audit/P1 local map. No SDK, command or mode publishers."""
import argparse
from collections import deque
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import threading
import time
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from perception.pipeline import MappingEngine,ReceiveMeter,expired_snapshot,load_settings


def main():
    import rospy
    from sensor_msgs.msg import PointCloud2,Imu
    from sensor_msgs import point_cloud2
    from nav_msgs.msg import Odometry,OccupancyGrid
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from perception.ros_messages import messages,pointcloud_xyz
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--settings',type=Path,required=True);p.add_argument('--calibration',type=Path)
    p.add_argument('--output',type=Path,required=True)
    group=p.add_mutually_exclusive_group()
    group.add_argument('--audit-only',action='store_true');group.add_argument('--reference-preview',action='store_true')
    p.add_argument('--seconds',type=float)
    a=p.parse_args(rospy.myargv()[1:])
    if a.seconds is not None and (not math.isfinite(a.seconds) or a.seconds<=0):p.error('seconds must be positive and finite')
    if a.reference_preview and a.calibration is not None:p.error('reference preview does not accept a body calibration override')
    settings,calibration,cfg,cal_path=load_settings(a.settings,a.calibration)
    if a.reference_preview:
        from perception.reference_preview import ReferencePreviewEngine,preview_messages
        from visualization_msgs.msg import Marker
        engine=ReferencePreviewEngine(settings,calibration,cfg)
    else:engine=MappingEngine(settings,calibration,cfg,a.audit_only)
    meter=ReceiveMeter()
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'settings.json').write_text(json.dumps(settings,indent=2)+'\n')
    (a.output/'calibration.json').write_text(json.dumps(calibration,indent=2)+'\n')
    (a.output/'metadata.json').write_text(json.dumps(dict(mode='uncalibrated_reference_preview' if a.reference_preview else 'data_audit' if a.audit_only else 'local_mapping_preview',
        source='external_pointlio',settings_path=str(a.settings.resolve()),calibration_path=str(cal_path),
        calibration_sha256=hashlib.sha256(cal_path.read_bytes()).hexdigest(),
        calibration_issues=engine.calibration_issues,motion_commands_sent=0,hardware_acceptance='pending'),indent=2)+'\n')
    rospy.init_node('go2_lidar_reference_preview' if a.reference_preview else 'go2_perception_audit' if a.audit_only else 'go2_perception_mapping')
    prefix='/go2/lidar_preview' if a.reference_preview else '/go2/perception';pubs={}
    if a.reference_preview:
        for key in ('points','lowest_returns','highest_returns'):
            pubs[key]=rospy.Publisher(prefix+'/'+key,PointCloud2,queue_size=1)
        for key in ('window','warning','observed_cells','reference_pose'):
            pubs[key]=rospy.Publisher(prefix+'/'+key,Marker,queue_size=1)
    elif not a.audit_only:
        for key,kind in [('map',OccupancyGrid),('elevation',PointCloud2),('obstacles',PointCloud2),('body_pose',PoseStamped)]:
            pubs[key]=rospy.Publisher(prefix+'/'+key,kind,queue_size=1)
    status_pub=rospy.Publisher(prefix+('/audit/status' if a.audit_only else '/status'),String,queue_size=1)
    lock=threading.Lock();poses=deque(maxlen=settings['pose_buffer_size']);latest={};dropped={};tf_edges={}
    stream=(a.output/'inputs.csv').open('w',newline='',buffering=1);writer=csv.writer(stream)
    writer.writerow(['kind','source_stamp','receipt_ros_time','receipt_monotonic','frame','processed_ok','reason'])
    status_log=(a.output/'status.jsonl').open('w',buffering=1)
    def submit(kind,msg):
        wall,mono=rospy.Time.now().to_sec(),time.monotonic()
        try:
            stamp=float(json.loads(msg.data)['stamp_sec']) if kind=='direction' and len(msg.data)<100000 else msg.header.stamp.to_sec() if kind!='direction' else float('nan')
        except (ValueError,TypeError,KeyError):stamp=float('nan')
        meter.observe(kind,stamp,mono)
        with lock:
            if kind=='odom':
                if len(poses)==poses.maxlen:dropped[kind]=dropped.get(kind,0)+1
                poses.append((msg,wall,mono))
            else:
                if kind in latest:dropped[kind]=dropped.get(kind,0)+1
                latest[kind]=(msg,wall,mono)
    def tf_callback(msg,static=False):
        with lock:
            for transform in msg.transforms:
                parent,child=transform.header.frame_id,transform.child_frame_id
                if not parent or not child:continue
                t=transform.transform.translation;q=transform.transform.rotation
                values=[t.x,t.y,t.z,q.x,q.y,q.z,q.w]
                if not all(math.isfinite(x) for x in values):continue
                key=parent+' -> '+child
                if key not in tf_edges and len(tf_edges)>=128:tf_edges.pop(next(iter(tf_edges)))
                tf_edges[key]=dict(parent=parent,child=child,static=static,source_stamp=transform.header.stamp.to_sec(),
                                   translation=values[:3],quaternion=values[3:],received_mono=time.monotonic())
    subscribers=[rospy.Subscriber(settings['topics']['odom'],Odometry,lambda m:submit('odom',m),queue_size=100),
                 rospy.Subscriber(settings['topics']['cloud'],PointCloud2,lambda m:submit('cloud',m),queue_size=1,buff_size=2**24),
                 rospy.Subscriber('/tf',TFMessage,tf_callback,queue_size=10),
                 rospy.Subscriber('/tf_static',TFMessage,lambda m:tf_callback(m,True),queue_size=10)]
    if not a.reference_preview:subscribers.append(rospy.Subscriber(settings['topics']['direction'],String,lambda m:submit('direction',m),queue_size=1))
    if settings['topics'].get('imu'):subscribers.append(rospy.Subscriber(settings['topics']['imu'],Imu,lambda m:submit('imu',m),queue_size=10))
    finished=threading.Event();shared=dict(snapshot=None,worker_error=None)
    def process_one(kind,item):
        msg,wall,mono=item;stamp=0.;frame='';ok=False;reason=None
        try:
            if kind=='direction':
                if len(msg.data)>100000:raise ValueError('oversize_direction')
                payload=json.loads(msg.data);stamp=float(payload.get('stamp_sec',0.));frame=payload.get('frame_id','')
                ok=engine.direction_input(payload,wall,mono)
            else:
                stamp=msg.header.stamp.to_sec();frame=msg.header.frame_id
                if kind=='odom':
                    pp=msg.pose.pose.position;q=msg.pose.pose.orientation
                    ok=engine.pose_input(stamp,[pp.x,pp.y,pp.z],[q.x,q.y,q.z,q.w],frame,msg.child_frame_id,wall,mono)
                elif kind=='cloud':
                    if msg.width*msg.height>settings['max_cloud_points']:raise ValueError('oversize_cloud')
                    points=pointcloud_xyz(msg)
                    ok=engine.cloud_input(stamp,points,frame,wall,mono)
                else:ok=engine.audit.observe(kind,stamp,wall,mono,frame)
            reason=engine.audit.topics.get(kind,{}).get('reason') if not ok else None
            if kind=='cloud' and not ok:reason=engine.last_error or reason
        except Exception as exc:
            reason=type(exc).__name__+':'+str(exc)
            if kind=='cloud':engine.cloud_deadline=-math.inf;engine.last_error=reason
            if kind=='odom':engine.pose_deadline=-math.inf;engine.last_error=reason
        writer.writerow([kind,stamp,wall,mono,frame,ok,reason])
    def work():
        while not finished.is_set():
            started=time.monotonic()
            with lock:
                events=list(poses);poses.clear();batch=dict(latest);latest.clear()
            try:
                engine.clock_check(rospy.Time.now().to_sec())
                for item in events:process_one('odom',item)
                for kind in ('imu','cloud','direction'):
                    if kind in batch:process_one(kind,batch[kind])
                snap=engine.snapshot(rospy.Time.now().to_sec(),time.monotonic())
                with lock:shared.update(snapshot=snap,worker_error=None)
            except Exception as exc:
                with lock:shared['worker_error']=type(exc).__name__+':'+str(exc)
            finished.wait(max(0.,settings['period_s']-(time.monotonic()-started)))
    worker=threading.Thread(target=work,name='perception-worker',daemon=True);worker.start()
    start=time.monotonic();next_visual=0.;next_report=0.;last_status={};last_visual_valid=None
    print('READONLY_PERCEPTION',a.output,'CALIBRATION_ISSUES',engine.calibration_issues,flush=True)
    try:
        while not rospy.is_shutdown() and (a.seconds is None or time.monotonic()-start<a.seconds):
            mono=time.monotonic();wall=rospy.Time.now().to_sec()
            with lock:
                snap=shared['snapshot'];error=shared['worker_error'];drops=dict(dropped);transforms=list(tf_edges.values())
            if snap is not None:
                snap=expired_snapshot(snap,wall,mono,settings)
                if error:
                    snap=dict(snap,valid=False,reason='worker_error:'+error,occupancy=np.full_like(snap['occupancy'],-1),
                              elevation_points=np.empty((0,3)),obstacle_points=np.empty((0,3)))
                state=dict(mode='uncalibrated_reference_preview' if a.reference_preview else 'audit_only' if a.audit_only else 'mapping_preview',map_data_valid=snap['valid'],reason=snap['reason'],
                    fixed_frame=snap['frame'],source_stamp=snap['source_stamp'],published_ros_time=wall,
                    map_source_age_s=wall-snap['source_stamp'] if snap['source_stamp'] else None,
                    support_valid=snap['support_valid'],queue_drops=drops,callback_rates=meter.report(),tf_edges=transforms,
                    imu_topic_configured=settings['topics'].get('imu'),**snap['audit'])
                if a.reference_preview:
                    state.update(displayed_points=len(snap['obstacle_points']) if snap['valid'] else 0,
                                 displayed_cells=len(snap['elevation_points']) if snap['valid'] else 0,
                                 window_origin=snap['origin'].tolist(),window_m=engine.window_m,resolution_m=engine.resolution)
                last_status=state;status_pub.publish(String(data=json.dumps(state,allow_nan=False)))
                if not a.audit_only and (mono>=next_visual or snap['valid']!=last_visual_valid):
                    for key,message in (preview_messages(snap) if a.reference_preview else messages(snap)).items():pubs[key].publish(message)
                    next_visual=mono+settings['publish_period_s'];last_visual_valid=snap['valid']
                if mono>=next_report:
                    status_log.write(json.dumps(state,allow_nan=False)+'\n')
                    print('P0P1',state['reason'],'cloud_hz',state['callback_rates'].get('cloud',{}).get('receive_hz'),
                          'pair_skew_p95_s',state['pair_skew_p95_s'],'calibration',state['calibration_issues'],flush=True)
                    next_report=mono+1.
            time.sleep(.05)
    finally:
        finished.set();worker.join(timeout=3.)
        # Do not close a stream still owned by a slow worker; process teardown will close it.
        if not worker.is_alive():stream.close()
        status_log.close()
        (a.output/'report.json').write_text(json.dumps(dict(last_status=last_status,worker_still_running=worker.is_alive(),
            motion_commands_sent=0,hardware_acceptance='pending'),indent=2,allow_nan=False)+'\n')
    return 0

if __name__=='__main__':raise SystemExit(main())
