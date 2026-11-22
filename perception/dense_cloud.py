"""Raw Livox-to-world visualization using per-point odometry interpolation.

This is approximate visual deskew at the odometry rate, not the Point-LIO IMU
estimator, a Go2 body calibration, or a navigation input.
"""
from collections import deque
import math
import struct
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from perception.pipeline import Audit, Pose
from perception.spatial_roi import SpatialROI


CUSTOM_MSG_MD5='e4d6829bdfe657cb6c21a746c86b21a6'
POINT_DTYPE=np.dtype([('offset','<u4'),('x','<f4'),('y','<f4'),('z','<f4'),('reflectivity','u1'),('tag','u1'),('line','u1')])


def raw_packet(msg):
    """Verified ROS1 CustomMsg wire layout; no per-point Python deserialization."""
    if hasattr(msg,'_buff'):
        connection=getattr(msg,'_connection_header',{})
        if connection.get('type')!='livox_ros_driver2/CustomMsg' or connection.get('md5sum')!=CUSTOM_MSG_MD5:
            raise ValueError('unsupported raw ROS message schema')
        data=msg._buff
        if len(data)<36:raise ValueError('truncated raw header')
        _,sec,nsec,length=struct.unpack_from('<4I',data,0)
        if length>256 or nsec>=1000000000 or len(data)<36+length:raise ValueError('invalid raw header length or time')
        frame=data[16:16+length].decode('utf-8')
        offset=16+length;timebase,n,_=struct.unpack_from('<QIB',data,offset)
        count=struct.unpack_from('<I',data,offset+16)[0]
        if n!=count or not 3<=n<=200000 or len(data)!=offset+20+n*POINT_DTYPE.itemsize:
            raise ValueError('raw count or wire length mismatch')
        records=np.frombuffer(data,dtype=POINT_DTYPE,count=n,offset=offset+20)
        stamp=sec+nsec*1e-9
    else:
        stamp=float(msg.header.stamp.to_sec());frame=msg.header.frame_id;timebase=msg.timebase;n=int(msg.point_num)
        if n!=len(msg.points) or not 3<=n<=200000:raise ValueError('raw_count_invalid')
        if any(not 0<=q.offset_time<=0xffffffff for q in msg.points):raise ValueError('invalid_point_time_or_flags')
        records=np.array([(q.offset_time,q.x,q.y,q.z,getattr(q,'reflectivity',0),q.tag,q.line) for q in msg.points],dtype=POINT_DTYPE)
    return SimpleNamespace(stamp=stamp,frame=frame,timebase=timebase,n=n,records=records,end=stamp+float(records['offset'].max())*1e-9)


def sensor_transform(parameters):
    if parameters.get('extrinsic_est_en') is not False:
        raise ValueError('dense display requires fixed Point-LIO internal extrinsics')
    t=np.asarray(parameters.get('extrinsic_T'),float)
    r=np.asarray(parameters.get('extrinsic_R'),float)
    if t.shape!=(3,) or r.size!=9:raise ValueError('missing Point-LIO internal transform')
    r=r.reshape(3,3)
    if not np.isfinite(t).all() or not np.isfinite(r).all() or not np.allclose(r.T@r,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(r),1.):
        raise ValueError('invalid Point-LIO internal transform')
    return t,r


def deskew(points, times, poses, translation, rotation, max_gap):
    """Interpolate translation/SLERP at EVERY point time; never extrapolate."""
    if len(poses)<2:raise ValueError('waiting_for_pose_brackets')
    stamps=np.array([p.stamp for p in poses])
    if np.any(np.diff(stamps)<=0):raise ValueError('nonincreasing_pose_buffer')
    if times.min()<stamps[0]-1e-7 or times.max()>stamps[-1]+1e-7:
        raise ValueError('waiting_for_pose_brackets')
    index=np.clip(np.searchsorted(stamps,times,side='right')-1,0,len(stamps)-2)
    gap=stamps[index+1]-stamps[index]
    if np.any(gap>max_gap):raise ValueError('pose_bracket_gap')
    f=np.clip((times-stamps[index])/gap,0.,1.)
    positions=np.array([p.position for p in poses]);quats=np.array([p.quaternion for p in poses])
    quats=quats/np.linalg.norm(quats,axis=1)[:,None]
    qa=quats[index];qb=quats[index+1].copy()
    dots=np.sum(qa*qb,axis=1);negative=dots<0;qb[negative]*=-1;dots=np.abs(dots)
    dots=np.clip(dots,0.,1.);linear=dots>.9995
    q=qa*(1-f[:,None])+qb*f[:,None]
    curved=~linear
    theta=np.arccos(dots[curved]);den=np.sin(theta)
    q[curved]=(qa[curved]*np.sin((1-f[curved])*theta)[:,None]+qb[curved]*np.sin(f[curved]*theta)[:,None])/den[:,None]
    q/=np.linalg.norm(q,axis=1)[:,None]
    local=points@rotation.T+translation
    return Rotation.from_quat(q).apply(local)+positions[index]*(1-f[:,None])+positions[index+1]*f[:,None]


class DenseProjector:
    def __init__(self,settings,parameters):
        self.settings=settings;self.d=settings['dense_visualization']
        self.translation,self.rotation=sensor_transform(parameters)
        self.parameters=dict(parameters);self.poses=deque(maxlen=settings['pose_buffer_size'])
        self.audit=Audit(settings['max_source_age_s']);self.reset_reason=None;self.last_wall=None
        self.last_reason='waiting_for_inputs';self.last_source_stamp=0.;self.last_metrics={}
        self.accepted=0;self.processing_ms=0.
        self.roi=SpatialROI(settings.get('spatial_roi'),expected_frame='pointlio_reference')
        for key in ('max_pose_gap_s','max_scan_span_s','voxel_m'):
            if not math.isfinite(self.d[key]) or self.d[key]<=0:raise ValueError('invalid dense setting: '+key)
        if not .005<=self.d['voxel_m']<=.25:raise ValueError('visual voxel outside supported range')
        if type(parameters.get('scan_line')) is not int or not 1<=parameters['scan_line']<=128:raise ValueError('invalid Livox scan line count')
        if type(self.d['queue_size']) is not int or not 1<=self.d['queue_size']<=10:raise ValueError('invalid raw queue size')
        if not self.d['raw_frame'] or not self.d['reference_frame']:raise ValueError('dense frame names required')
        if not 0<=self.d['minimum_range_m']<self.d['maximum_range_m']<=200:raise ValueError('invalid raw range')
    def clock_check(self,wall):
        if self.last_wall is not None and wall<self.last_wall-.001:
            self.reset_reason='ROS_clock_rollback_restart_required';self.poses.clear()
        self.last_wall=wall
    def pose_input(self,stamp,position,quaternion,frame,child,wall,mono):
        if not self.audit.observe('odom',stamp,wall,mono,frame):return False
        try:
            pose=Pose(stamp,np.asarray(position,float),np.asarray(quaternion,float),frame).validate()
            if frame!=self.settings['fixed_frame'] or child!=self.d['reference_frame']:
                self.reset_reason='dense_pose_frame_changed_restart_required';raise ValueError(self.reset_reason)
            if self.poses and np.linalg.norm(pose.position-self.poses[-1].position)>self.settings['max_pose_jump_m']:
                self.reset_reason='odometry_jump_restart_required';raise ValueError(self.reset_reason)
            if self.poses and (Rotation.from_quat(self.poses[-1].quaternion).inv()*Rotation.from_quat(pose.quaternion)).magnitude()>1.:
                self.reset_reason='odometry_rotation_jump_restart_required';raise ValueError(self.reset_reason)
            self.poses.append(pose);return True
        except (ValueError,TypeError) as exc:
            self.audit.reject('odom',str(exc));self.last_reason=str(exc);return False
    def decode(self,msg,wall,mono):
        packet=raw_packet(msg);stamp=packet.stamp;n=packet.n;data=packet.records
        if not self.audit.observe('raw',stamp,wall,mono,packet.frame):raise ValueError('raw_timestamp_rejected')
        if packet.frame!=self.d['raw_frame']:raise ValueError('raw_frame_mismatch')
        if n>self.settings['max_cloud_points']:raise ValueError('raw_count_invalid')
        if abs(float(packet.timebase)*1e-9-stamp)>1e-5:raise ValueError('raw_timebase_header_mismatch')
        times=stamp+data['offset'].astype(float)*1e-9
        if times.max()-stamp>self.d['max_scan_span_s']:raise ValueError('oversize_scan_time_span')
        if times.max()>wall+.05:raise ValueError('raw_future_point_time')
        xyz=np.column_stack([data[name] for name in ('x','y','z')]).astype(float)
        finite=np.isfinite(xyz).all(axis=1)
        radius=np.linalg.norm(np.where(finite[:,None],xyz,0.),axis=1)
        flags=data['tag']&0x30
        valid=finite&(radius>=self.d['minimum_range_m'])&(radius<=self.d['maximum_range_m'])&((flags==0)|(flags==0x10))&(data['line']<self.parameters['scan_line'])
        if valid.sum()<3:raise ValueError('insufficient_valid_raw_points')
        return dict(stamp=stamp,end=float(times.max()),points=xyz[valid],times=times[valid],wall=wall,mono=mono,input_points=n,filtered_points=int(n-valid.sum()))
    def project(self,frame,wall,mono):
        if self.reset_reason:raise ValueError(self.reset_reason)
        if wall-frame['stamp']>self.settings['max_source_age_s'] or wall<frame['wall']-.001:
            raise ValueError('raw_scan_expired')
        if not self.poses or wall-self.poses[-1].stamp>self.settings['max_source_age_s']:
            raise ValueError('waiting_for_fresh_odometry')
        points=deskew(frame['points'],frame['times'],list(self.poses),self.translation,self.rotation,self.d['max_pose_gap_s'])
        if not np.isfinite(points).all() or np.any(np.abs(points)>1e6):raise ValueError('projected_world_points_invalid')
        half=self.settings['reference_preview']['window_m']/2+self.settings['reference_preview']['resolution_m']
        points=points[np.all(np.abs(points[:,:2]-self.poses[-1].position[:2])<=half,axis=1)]
        if len(points)<3:raise ValueError('no_returns_in_dense_window')
        # Only the visualization branch uses this finer voxel; estimator settings are untouched.
        if self.roi.enabled:
            result,roi_metrics=self.roi.select(points,self.poses[-1])
        else:
            voxel=np.floor(points/self.d['voxel_m']).astype(np.int64)
            _,indices=np.unique(voxel,axis=0,return_index=True);indices.sort()
            result=points[indices];roi_metrics=dict(enabled=False)
        if len(result)<3:raise ValueError('insufficient_dense_voxels')
        self.accepted+=1;self.last_reason='dense_visualization_only';self.last_source_stamp=frame['end']
        self.last_metrics=dict(input_points=frame['input_points'],filtered_points=frame['filtered_points'],output_points=len(result),
                               scan_begin=frame['stamp'],scan_end=frame['end'],scan_span_s=frame['end']-frame['stamp'],voxel_m=self.d['voxel_m'])
        self.last_metrics['spatial_roi']=roi_metrics
        return result
    def status(self,mono):
        return dict(mode='dense_visualization_only',reason=self.reset_reason or self.last_reason,accepted_scans=self.accepted,
                    last_source_stamp=self.last_source_stamp,processing_ms=self.processing_ms,**self.last_metrics,
                    internal_extrinsics_source='Point-LIO ROS parameter snapshot, not measured Go2 extrinsics',
                    approximate_per_point_odometry_deskew=True,navigation_usable=False,motion_commands_sent=0,
                    hardware_acceptance='pending',audit=self.audit.report(mono))


def xyz_message(header,points):
    """Vectorized ROS serialization, bounded to XYZ float32 rather than Python loops."""
    from sensor_msgs.msg import PointCloud2,PointField
    cloud=PointCloud2();cloud.header=header;cloud.height=1
    xyz=np.asarray(points,dtype='<f4').reshape(-1,3)
    cloud.width=len(xyz);cloud.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(('x','y','z'))]
    cloud.is_bigendian=False;cloud.point_step=12;cloud.row_step=len(xyz)*12
    cloud.data=xyz.tobytes();cloud.is_dense=True
    return cloud
