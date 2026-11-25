"""Bridge existing native Go2 data to local ROS visualization topics."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from native.pointcloud import compact_registered_cloud

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    cfg=json.loads((ROOT/'config/native_lidar.json').read_text())
    sys.path.insert(0,str(ROOT/'vendor/unitree_sdk2_python'))
    sys.path.append(cfg['dds_runtime'])
    import rospy
    from sensor_msgs.msg import PointCloud2,PointField
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped,TransformStamped
    from std_msgs.msg import String,Header
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import Marker
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize,ChannelFactory,ChannelPublisher
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
    from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
    from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PoseStamped_
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    from native.state_message import LidarState_
    from cyclonedds.internal import InvalidSample
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from cyclonedds.qos import Qos,Policy
    rospy.init_node('go2_native_lidar_view')
    ChannelFactoryInitialize(0,cfg['interface'])
    participant=next(v for v in vars(ChannelFactory()).values() if isinstance(v,DomainParticipant))
    qos=Qos(Policy.Reliability.BestEffort,Policy.History.KeepLast(1))
    schema=[
        ('cloud','rt/utlidar/cloud',PointCloud2_,PointCloud2),
        ('cloud_deskewed','rt/utlidar/cloud_deskewed',PointCloud2_,PointCloud2),
        ('height_cloud','rt/utlidar/height_map',PointCloud2_,PointCloud2),
        ('grid_cloud','rt/utlidar/grid_map',PointCloud2_,PointCloud2),
        ('slam_cloud','rt/uslam/cloud_map',PointCloud2_,PointCloud2),
        ('mapping_cloud','rt/uslam/frontend/cloud_world_ds',PointCloud2_,PointCloud2),
        ('mapping_odom','rt/uslam/frontend/odom',Odometry_,Odometry),
        ('localization_cloud','rt/uslam/localization/cloud_world',PointCloud2_,PointCloud2),
        ('localization_odom','rt/uslam/localization/odom',Odometry_,Odometry),
        ('global_path','rt/uslam/navigation/global_path',PointCloud2_,PointCloud2),
        ('odom','rt/utlidar/robot_odom',Odometry_,Odometry),
        ('pose','rt/utlidar/robot_pose',PoseStamped_,PoseStamped),
        ('lidar_state','rt/utlidar/lidar_state',LidarState_,String),
        ('lidar_log','rt/utlidar/server_log',String_,String),
        ('slam_log','rt/uslam/server_log',String_,String)]
    readers={k:DataReader(participant,Topic(participant,t,d),qos) for k,t,d,_ in schema}
    pubs={k:rospy.Publisher('/go2/native/'+k,r,queue_size=1) for k,_,_,r in schema}
    raw_deskewed=rospy.Publisher('/go2/native/cloud_deskewed_unfiltered',PointCloud2,queue_size=1)
    status=rospy.Publisher('/go2/native/status',String,queue_size=1,latch=True)
    marker_pub=rospy.Publisher('/go2/native/status_text',Marker,queue_size=1)
    tf_pub=rospy.Publisher('/tf',TFMessage,queue_size=1)
    counts,errors=Counter(),Counter()
    cleanup_totals=Counter()
    received,details={},{}
    position=[0.,0.,0.]
    def header(h):
        return Header(stamp=rospy.Time(h.stamp.sec,h.stamp.nanosec),frame_id=h.frame_id)
    def pose(out,src):
        for k in ('x','y','z'):setattr(out.position,k,getattr(src.position,k))
        for k in ('x','y','z','w'):setattr(out.orientation,k,getattr(src.orientation,k))
    def convert(k,m):
        if isinstance(m,PointCloud2_):
            if m.width*m.height>1000000 or len(m.data)>32000000:raise ValueError('oversized cloud')
            out=PointCloud2(header=header(m.header),height=m.height,width=m.width,
                is_bigendian=m.is_bigendian,point_step=m.point_step,row_step=m.row_step,
                data=bytes(m.data),is_dense=m.is_dense)
            out.fields=[PointField(name=f.name,offset=f.offset,datatype=f.datatype,count=f.count) for f in m.fields]
            if k=='cloud_deskewed':
                if raw_deskewed.get_num_connections():raw_deskewed.publish(out)
                started=time.monotonic()
                layout,cleanup=compact_registered_cloud(out)
                out=PointCloud2(header=out.header,fields=out.fields,
                               is_bigendian=out.is_bigendian,**layout)
                cleanup['processing_ms']=round((time.monotonic()-started)*1000,3)
                for field in ('input_points','input_bytes','removed_zero_points','removed_nonfinite_points','output_points','output_bytes'):
                    cleanup_totals[field]+=cleanup[field]
                details[k]=dict(points=out.width*out.height,frame=m.header.frame_id,cleanup=cleanup)
            else:details[k]=dict(points=m.width*m.height,frame=m.header.frame_id)
            return out
        if isinstance(m,PoseStamped_):
            out=PoseStamped(header=header(m.header));pose(out.pose,m.pose)
            return out
        if isinstance(m,Odometry_):
            out=Odometry(header=header(m.header),child_frame_id=m.child_frame_id)
            pose(out.pose.pose,m.pose.pose)
            out.pose.covariance=list(m.pose.covariance)
            out.twist.covariance=list(m.twist.covariance)
            for part in ('linear','angular'):
                for axis in ('x','y','z'):
                    setattr(getattr(out.twist.twist,part),axis,getattr(getattr(m.twist.twist,part),axis))
            p=out.pose.pose.position
            if k=='odom':position[:]=[p.x,p.y,p.z]
            tf=TransformStamped(header=out.header,child_frame_id=out.child_frame_id)
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=position
            tf.transform.rotation=out.pose.pose.orientation
            if k=='odom' and out.child_frame_id and out.child_frame_id!=out.header.frame_id:
                tf_pub.publish(TFMessage([tf]))
            details[k]=dict(frame=out.header.frame_id,child=out.child_frame_id)
            return out
        if isinstance(m,LidarState_):
            details[k]={f:getattr(m,f) for f in ('firmware_version','software_version','sdk_version','error_state','dirty_percentage',
                'cloud_frequency','cloud_packet_loss_rate','cloud_size','cloud_scan_num','imu_frequency','imu_packet_loss_rate')}
            return String(data=json.dumps(details[k]))
        details[k]=m.data[:2500]
        return String(data=m.data[:10000])
    if cfg['radar_on']:
        switch=ChannelPublisher('rt/utlidar/switch',String_);switch.Init()
        time.sleep(.5);switch.Write(String_('ON'))
    args.output.mkdir(parents=True,exist_ok=True)
    next_report=time.monotonic()
    with (args.output/'status.jsonl').open('a',buffering=1) as log:
        while not rospy.is_shutdown():
            now=time.monotonic()
            for k,reader in readers.items():
                for m in reader.take(1):
                    if isinstance(m,InvalidSample):
                        continue
                    try:
                        pubs[k].publish(convert(k,m));counts[k]+=1;received[k]=now
                    except (AttributeError,TypeError,ValueError,OverflowError) as exc:
                        errors[k]+=1;details[k+'_error']=str(exc)
            if now>=next_report:
                ages={k:round(now-v,3) for k,v in received.items()}
                report=dict(source=str(ROOT),sdk=str(ROOT/'vendor/unitree_sdk2_python'),
                    counts=dict(counts),receive_age_s=ages,details=details,errors=dict(errors),
                    cloud_deskewed_cleanup_totals=dict(cleanup_totals),motion_commands_sent=0)
                status.publish(String(data=json.dumps(report)))
                log.write(json.dumps(report)+'\n')
                marker=Marker()
                marker.header=Header(stamp=rospy.Time.now(),frame_id=cfg['fixed_frame'])
                marker.ns='native_status';marker.type=Marker.TEXT_VIEW_FACING
                marker.pose.orientation.w=1.
                marker.pose.position.x,marker.pose.position.y=position[:2]
                marker.pose.position.z=position[2]+.7
                marker.scale.z=.12
                marker.color.r=marker.color.g=marker.color.a=1.
                marker.text=('Native cloud: waiting for data' if 'cloud' not in ages else
                    'Native cloud: %s pts | received %.1fs ago'%(details['cloud']['points'],ages['cloud']))
                marker.text+='\nNative odometry: '+('receiving' if ages.get('odom',999)<1 else 'waiting')
                marker_pub.publish(marker)
                for k,age in ages.items():
                    if age>1 and (k.endswith('cloud') or k=='cloud_deskewed'):
                        out=PointCloud2();out.header.frame_id=details.get(k,{}).get('frame',cfg['fixed_frame'])
                        pubs[k].publish(out)
                        if k=='cloud_deskewed' and raw_deskewed.get_num_connections():raw_deskewed.publish(out)
                print(marker.text.replace('\n',' | '),flush=True)
                next_report=now+1
            time.sleep(.015)
if __name__=='__main__':main()
