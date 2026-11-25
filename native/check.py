"""Measure native lidar rates, source age and observed ground around a stationary Go2."""
import argparse
import collections
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from perception.ros_messages import pointcloud_xyz


def main():
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from scipy.spatial.transform import Rotation
    p=argparse.ArgumentParser()
    p.add_argument('--seconds',type=float,default=8.)
    args=p.parse_args()
    if not 2<=args.seconds<=30:p.error('seconds must be within 2..30')
    rospy.init_node('go2_native_data_check',anonymous=True)
    ages=collections.defaultdict(list)
    samples=collections.defaultdict(list)
    source_times=collections.defaultdict(list)
    receive_times=collections.defaultdict(list)
    poses=collections.deque(maxlen=500)
    clouds=collections.deque(maxlen=20)
    status={}
    def cloud(m,key):
        stamp=m.header.stamp.to_sec()
        ages[key].append(rospy.Time.now().to_sec()-stamp)
        source_times[key].append(stamp)
        receive_times[key].append(time.monotonic())
        samples[key].append(m.width*m.height)
        if key=='deskewed':
            clouds.append((stamp,pointcloud_xyz(m)))
    def odom(m):
        poses.append(m)
        stamp=m.header.stamp.to_sec()
        ages['odom'].append(rospy.Time.now().to_sec()-stamp)
        source_times['odom'].append(stamp)
        receive_times['odom'].append(time.monotonic())
    def state(m):
        status.update(json.loads(m.data))
    subs=[
        rospy.Subscriber('/go2/native/cloud',PointCloud2,lambda m:cloud(m,'raw'),queue_size=1,buff_size=2**22),
        rospy.Subscriber('/go2/native/cloud_deskewed',PointCloud2,lambda m:cloud(m,'deskewed'),queue_size=1,buff_size=2**22),
        rospy.Subscriber('/go2/native/odom',Odometry,odom,queue_size=50),
        rospy.Subscriber('/go2/native/status',String,state,queue_size=1)]
    start=time.monotonic()
    rospy.sleep(args.seconds)
    for s in subs:s.unregister()
    duration=time.monotonic()-start
    report=dict(duration_s=duration,source='native DDS ROS bridge',motion_commands_sent=0)
    report['topics']={key:dict(messages=len(a),receive_hz=round(len(a)/duration,2),
        median_header_wall_difference_s=round(float(np.median(a)),4),
        p95_header_wall_difference_s=round(float(np.percentile(a,95)),4),
        source_stamp_nonincreasing=int(np.sum(np.diff(source_times[key])<=0)),
        max_receive_gap_s=round(float(np.max(np.diff(receive_times[key]))),4) if len(a)>1 else None,
        median_points=float(np.median(samples[key])) if samples[key] else None) for key,a in ages.items() if a}
    report['timestamp_alignment_verified']=False
    report['clock_warning']='Header-to-wall differences include device clock offset and are not verified processing latency.'
    report['header_wall_within_one_second']=all(abs(float(np.median(v)))<1 for v in ages.values() if v) and bool(ages)
    report['bridge_errors']=status.get('errors',{})
    report['last_lidar_state']=status.get('details',{}).get('lidar_state')
    relative=[]
    skew=[]
    poses=list(poses)
    for stamp,xyz in list(clouds):
        if not poses:break
        o=min(poses,key=lambda o:abs(o.header.stamp.to_sec()-stamp))
        dt=abs(o.header.stamp.to_sec()-stamp)
        if dt>.1:continue
        q=o.pose.pose.orientation
        p=o.pose.pose.position
        xyz=xyz[np.isfinite(xyz).all(axis=1)]
        local=(xyz-[p.x,p.y,p.z])@Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        relative.append(local)
        skew.append(dt)
    if relative:
        xyz=np.concatenate(relative)
        radius=np.linalg.norm(xyz[:,:2],axis=1)
        ground_candidates=xyz[(radius>.2)&(radius<5)&(xyz[:,2]<-.04)&(xyz[:,2]>-.8)]
        if len(ground_candidates)>30:
            rng=np.random.default_rng(19)
            a=ground_candidates[rng.choice(len(ground_candidates),min(4000,len(ground_candidates)),replace=False)]
            best_count=0
            fit=None
            for _ in range(160):
                tri=a[rng.choice(len(a),3,replace=False)]
                n=np.cross(tri[1]-tri[0],tri[2]-tri[0])
                norm=np.linalg.norm(n)
                if norm<1e-5 or abs(n[2])/norm<math.cos(math.radians(10)):continue
                n=n/n[2]
                h=float(tri[0]@n)
                if not -.8<h<-.04:continue
                count=int(np.sum(abs(a@n-h)<.025))
                if count>best_count:
                    best_count=count
                    fit=(n,h)
            if fit:
                n,h=fit
                ground=ground_candidates[abs(ground_candidates@n-h)<.025]
                front=ground[(ground[:,0]>0)&(abs(ground[:,1])<.5)]
                report['ground']=dict(reference='native odometry child',plane_depth_m=round(-h,3),
                    fit_points=len(ground),nearest_horizontal_m=round(float(np.linalg.norm(ground[:,:2],axis=1).min()),3),
                    forward_band_width_m=1.,forward_bins=[
                        dict(from_m=lo,to_m=hi,returns=int(np.sum((front[:,0]>=lo)&(front[:,0]<hi))))
                        for lo,hi in ((0,.5),(.5,1),(1,2),(2,3),(3,4))],
                    note='Plane fit checks return coverage; it does not certify free space.')
        report['matched_clouds']=len(relative)
        report['max_pose_pair_skew_s']=round(max(skew),4)
    if poses:
        positions=np.array([[o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z] for o in poses])
        report['position_span_m']=np.ptp(positions,axis=0).tolist()
    f=ROOT/'logs'/('native_check_'+time.strftime('%Y%m%d_%H%M%S')+'.json')
    f.parent.mkdir(exist_ok=True)
    with f.open('x') as stream:json.dump(report,stream,indent=2)
    print(json.dumps(report,indent=2))
    print('REPORT',f)
    return 0 if all(report['topics'].get(k,{}).get('messages',0)>0 for k in ('raw','deskewed','odom')) and not report['bridge_errors'] else 1


if __name__=='__main__':raise SystemExit(main())
