#!/usr/bin/env python3
"""ROS1 read-only raw Livox display converter; never changes Point-LIO parameters."""
import argparse
from collections import deque
import csv
import json
import math
from pathlib import Path
import sys
import threading
import time
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from perception.dense_cloud import DenseProjector,xyz_message
from perception.pipeline import load_settings


def main():
    import rospy
    from livox_ros_driver2.msg import CustomMsg
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Header,String
    from sensor_msgs.msg import PointCloud2
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--settings',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(rospy.myargv()[1:])
    settings,_,_,_=load_settings(a.settings);d=settings['dense_visualization']
    rospy.init_node('go2_lidar_dense_display')
    namespace=d['pointlio_namespace'].rstrip('/')
    def parameters():
        values={k:rospy.get_param(namespace+'/mapping/'+k) for k in ('extrinsic_T','extrinsic_R','extrinsic_est_en')}
        values['scan_line']=rospy.get_param(namespace+'/preprocess/scan_line');return values
    params=parameters();engine=DenseProjector(settings,params)
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'metadata.json').write_text(json.dumps(dict(source='raw_livox_with_PointLIO_odometry',settings=settings,
        internal_extrinsics_snapshot=params,motion_commands_sent=0,navigation_usable=False,hardware_acceptance='pending'),indent=2)+'\n')
    pub=rospy.Publisher(d['output_topic'],PointCloud2,queue_size=1)
    status_pub=rospy.Publisher('/go2/lidar_dense/status',String,queue_size=1)
    lock=threading.Lock();poses=deque(maxlen=settings['pose_buffer_size']);raw=deque(maxlen=d['queue_size'])
    drops={'raw_callback':0,'pose_callback':0,'superseded_scans':0,'expired_or_rejected':0}
    def receive_pose(m):
        wall,mono=rospy.Time.now().to_sec(),time.monotonic()
        with lock:
            if len(poses)==poses.maxlen:drops['pose_callback']+=1
            poses.append((m,wall,mono))
    def receive_raw(m):
        wall,mono=rospy.Time.now().to_sec(),time.monotonic()
        with lock:
            if len(raw)==raw.maxlen:drops['raw_callback']+=1
            raw.append((m,wall,mono))
    subscribers=[rospy.Subscriber(settings['topics']['odom'],Odometry,receive_pose,queue_size=100),
                 rospy.Subscriber(d['raw_topic'],CustomMsg,receive_raw,queue_size=1,buff_size=2**24)]
    pending=deque(maxlen=d['queue_size']);last_output=None;last_valid=False;deadline=-math.inf;next_report=0.;next_params=0.
    period=.2
    stream=(a.output/'frames.csv').open('w',newline='',buffering=1);writer=csv.writer(stream)
    writer.writerow(['scan_begin','scan_end','raw_points','output_points','processing_ms','result'])
    status_log=(a.output/'status.jsonl').open('w',buffering=1);last_status={}
    def header(stamp):
        h=Header();h.frame_id=settings['fixed_frame'];h.stamp=rospy.Time.from_sec(stamp);return h
    try:
        while not rospy.is_shutdown():
            started=time.monotonic();wall=rospy.Time.now().to_sec();engine.clock_check(wall)
            with lock:ps=list(poses);poses.clear();rs=list(raw);raw.clear();drop_snapshot=dict(drops)
            for msg,rwall,rmono in ps:
                pp=msg.pose.pose.position;q=msg.pose.pose.orientation
                engine.pose_input(msg.header.stamp.to_sec(),[pp.x,pp.y,pp.z],[q.x,q.y,q.z,q.w],msg.header.frame_id,msg.child_frame_id,rwall,rmono)
            # Decode only the newest raw scan already covered by the pose buffer.
            # Decoding every queued 20k-point message before dropping it creates backlog.
            chosen=None
            for item in reversed(rs):
                msg,rwall,rmono=item
                end=msg.header.stamp.to_sec()+max((q.offset_time for q in msg.points),default=0)*1e-9
                if engine.poses and end<=engine.poses[-1].stamp+1e-7:
                    chosen=item;break
            if chosen is not None:
                drops['superseded_scans']+=len(rs)-1
                msg,rwall,rmono=chosen
                try:
                    frame=engine.decode(msg,rwall,rmono)
                    if len(pending)==pending.maxlen:drops['superseded_scans']+=1
                    pending.append(frame)
                except (ValueError,TypeError,AttributeError) as exc:
                    engine.last_reason=str(exc);drops['expired_or_rejected']+=1
                    if 'raw' in engine.audit.topics:engine.audit.reject('raw',str(exc))
                    writer.writerow([msg.header.stamp.to_sec(),None,msg.point_num,0,0,str(exc)])
            elif rs:
                # Keep unmatched scans bounded while waiting for the upper pose bracket.
                with lock:
                    for item in reversed(rs):
                        if len(raw)==raw.maxlen:break
                        raw.appendleft(item)
            while pending and wall-pending[0]['stamp']>settings['max_source_age_s']:
                pending.popleft();drops['expired_or_rejected']+=1;engine.last_reason='raw_scan_expired'
            selected=None;points=None
            for index in range(len(pending)-1,-1,-1):
                frame=pending[index]
                try:
                    before=time.monotonic();points=engine.project(frame,rospy.Time.now().to_sec(),before)
                    engine.processing_ms=(time.monotonic()-before)*1000;selected=index;break
                except ValueError as exc:
                    engine.last_reason=str(exc)
            if selected is not None:
                for _ in range(selected):pending.popleft();drops['superseded_scans']+=1
                frame=pending.popleft();last_output=frame
                # The OLDEST point source time sets the freshness lease, not publication time.
                deadline=frame['mono']+settings['max_source_age_s']-max(0.,frame['wall']-frame['stamp'])
                if time.monotonic()<=deadline:
                    pub.publish(xyz_message(header(frame['end']),points));last_valid=True
                else:engine.last_reason='dense_processing_deadline_expired'
                writer.writerow([frame['stamp'],frame['end'],frame['input_points'],len(points),engine.processing_ms,'published'])
            if started>=next_params:
                if parameters()!=params:engine.reset_reason='PointLIO_internal_extrinsics_changed_restart_required'
                next_params=started+2.
            valid=bool(last_output is not None and not engine.reset_reason and time.monotonic()<=deadline and engine.poses and wall-engine.poses[-1].stamp<=settings['max_source_age_s'])
            if not valid and last_output is not None and not engine.reset_reason:
                engine.last_reason='dense_source_or_odometry_expired'
            if not valid and (last_valid or started>=next_report):
                pub.publish(xyz_message(header(engine.last_source_stamp),np.empty((0,3))));last_valid=False
            last_status=engine.status(time.monotonic())
            last_status.update(data_valid=valid,queue_drops=drop_snapshot,pending_scans=len(pending),
                output_header_is_scan_end=True,oldest_point_age_s=wall-last_output['stamp'] if last_output else None)
            status_pub.publish(String(data=json.dumps(last_status,allow_nan=False)))
            if started>=next_report:
                status_log.write(json.dumps(last_status,allow_nan=False)+'\n')
                print('DENSE_DISPLAY',valid,engine.last_reason,'points',engine.last_metrics.get('output_points'),'processing_ms',round(engine.processing_ms,2),flush=True)
                next_report=started+1.
            time.sleep(max(0.,period-(time.monotonic()-started)))
    finally:
        stream.close();status_log.close()
        (a.output/'report.json').write_text(json.dumps(last_status,indent=2,allow_nan=False)+'\n')
    return 0

if __name__=='__main__':raise SystemExit(main())
