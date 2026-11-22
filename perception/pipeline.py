"""ROS-free P0 diagnostics and P1 rolling map. Never imports a robot SDK."""
from collections import deque
import copy
import json
import math
from pathlib import Path
import threading
import sys
import numpy as np

SIM=Path(__file__).resolve().parents[1]/'simulation'
if str(SIM) not in sys.path:sys.path.insert(0,str(SIM))
from core import Pose,TerrainMap,config as terrain_config
from geometry import body_pose_from_lidar
from perception.spatial_roi import SpatialROI


def load_settings(path,calibration_path=None):
    path=Path(path).resolve();settings=json.loads(path.read_text())
    calibration_path=Path(calibration_path).resolve() if calibration_path else path.parent/settings['calibration_file']
    calibration=json.loads(calibration_path.read_text())
    cfg=terrain_config();cfg.update(settings.get('terrain_overrides',{}))
    for key in ('lidar_extrinsics_calibrated','geometry_calibrated','self_filter_enabled'):
        if type(calibration.get(key)) is not bool:raise ValueError('calibration boolean required: '+key)
        cfg[key]=calibration[key]
    for key in ('lidar_position_body_m','lidar_rotation_body','body_reference_height_m','geometry_boxes'):
        if calibration.get(key) is not None and calibration[key]!=[]:cfg[key]=calibration[key]
    cfg['self_filter_enabled']=False  # Filtering is performed here using only measured solid parts.
    cfg['source_max_age_s']=settings['max_source_age_s'];cfg['source_max_skew_s']=settings['pair_max_skew_s']
    for key in ('period_s','publish_period_s','snapshot_lease_s','max_source_age_s','pair_max_skew_s','max_pose_jump_m'):
        if not math.isfinite(settings[key]) or settings[key]<=0:raise ValueError('invalid setting: '+key)
    for key in ('pose_buffer_size','max_cloud_points'):
        if type(settings[key]) is not int or settings[key]<=0:raise ValueError('invalid setting: '+key)
    if settings['odom_reference'] not in ('lidar','body'):raise ValueError('unsupported odom reference')
    if not settings['fixed_frame'] or not settings['body_frame']:raise ValueError('frame names required')
    if type(settings.get('direction')) is not dict:raise ValueError('direction contract required')
    hz=settings['direction'].get('expected_hz')
    if hz is not None and (not math.isfinite(hz) or hz<=0):raise ValueError('invalid direction frequency')
    if not math.isfinite(settings['direction']['max_age_s']) or settings['direction']['max_age_s']<=0:raise ValueError('invalid direction age')
    if settings['direction'].get('expected_frame') not in (None,'body','mic_array'):raise ValueError('invalid declared direction frame')
    for key in ('resolution_m','map_size_m','fusion_window_s','map_lease_s','obstacle_hold_s','body_reference_height_m'):
        if not math.isfinite(cfg[key]) or cfg[key]<=0:raise ValueError('invalid terrain parameter: '+key)
    if not 8<=round(cfg['map_size_m']/cfg['resolution_m'])<=256:raise ValueError('map size outside supported range')
    SpatialROI(settings.get('spatial_roi'))
    return settings,calibration,cfg,calibration_path


def calibration_issues(cal,cfg):
    issues=[]
    if cal.get('lidar_extrinsics_calibrated') is not True:issues.append('extrinsics_unmeasured')
    else:
        try:
            t=np.asarray(cal.get('lidar_position_body_m'),float);rotation=np.asarray(cal.get('lidar_rotation_body'),float)
            if t.shape!=(3,) or rotation.shape!=(3,3) or not np.isfinite(t).all() or not np.isfinite(rotation).all() or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(rotation),1.):raise ValueError()
        except (ValueError,TypeError):issues.append('invalid_extrinsics')
    if cal.get('geometry_calibrated') is not True:issues.append('geometry_unmeasured')
    else:
        try:
            height=float(cal.get('body_reference_height_m'))
            boxes=cal.get('geometry_boxes')
            if not math.isfinite(height) or height<=0 or not boxes:raise ValueError()
            for box in boxes:validate_box(box)
        except (ValueError,TypeError,KeyError):issues.append('invalid_geometry')
    if not isinstance(cal.get('measurement_record'),str) or not cal['measurement_record'].strip():issues.append('measurement_record_missing')
    if not isinstance(cal.get('reference_frame'),str) or not cal['reference_frame']:issues.append('odom_reference_frame_unconfirmed')
    if cal.get('self_filter_enabled'):
        try:
            if not cal.get('self_filter_boxes'):raise ValueError()
            for box in cal['self_filter_boxes']:validate_box(box)
        except (ValueError,TypeError,KeyError):issues.append('solid_self_filter_parts_unmeasured')
    return issues


def validate_box(box):
    lo,hi=np.asarray(box['minimum'],float),np.asarray(box['maximum'],float)
    if lo.shape!=(3,) or hi.shape!=(3,) or not np.isfinite([lo,hi]).all() or np.any(hi<=lo):raise ValueError('invalid solid box')
    return lo,hi


class Audit:
    def __init__(self,max_age):
        self.max_age=max_age;self.topics={};self.pairs=deque(maxlen=1000)
    def observe(self,kind,stamp,wall,mono,frame=''):
        row=self.topics.setdefault(kind,dict(count=0,rejected=0,last_valid=None,last_seen=None,frames=set(),samples=deque(maxlen=1000),reason='not_received'))
        row['count']+=1;row['last_seen']=mono
        if len(row['frames'])<32:row['frames'].add(str(frame))
        reason=None
        if not all(math.isfinite(x) for x in (stamp,wall,mono)) or stamp<=0:reason='invalid_timestamp'
        elif wall-stamp<-.05:reason='future_timestamp'
        elif wall-stamp>self.max_age:reason='stale_timestamp'
        elif row['last_valid'] and stamp<=row['last_valid'][0]:reason='reordered_or_duplicate_timestamp'
        if reason:self.reject(kind,reason);return False
        row['last_valid']=(stamp,wall,mono);row['samples'].append((stamp,mono,max(0.,wall-stamp)))
        row['reason']=None;return True
    def reject(self,kind,reason):
        row=self.topics[kind];row['rejected']+=1;row['reason']=reason
    def report(self,mono):
        result={}
        for kind,row in self.topics.items():
            samples=list(row['samples']);last=row['last_valid']
            result[kind]=dict(processed_count=row['count'],rejected_count=row['rejected'],frames=sorted(row['frames']),last_rejection=row['reason'],
                timestamp_valid_source_stamp=last[0] if last else None,timestamp_valid_receive_age_s=max(0.,mono-last[2]) if last else None,
                source_rate_hz=(len(samples)-1)/(samples[-1][0]-samples[0][0]) if len(samples)>1 else None,
                processed_receive_rate_hz=(len(samples)-1)/(samples[-1][1]-samples[0][1]) if len(samples)>1 and samples[-1][1]>samples[0][1] else None,
                source_age_p95_s=float(np.quantile([x[2] for x in samples],.95)) if samples else None,
                nonfinite_points_dropped=row.get('nonfinite_points_dropped',0))
        pairs=list(self.pairs)
        return dict(topics=result,pair_skew_p95_s=float(np.quantile(pairs,.95)) if pairs else None,
                    pair_skew_max_s=max(pairs) if pairs else None,
                    synchronization_verified=False,hardware_acceptance='pending_physical_measurement_and_motion_validation')


def interpolate_pose(poses,stamp,max_skew):
    if not poses:raise ValueError('waiting_for_odometry')
    nearest=min(poses,key=lambda p:abs(p.stamp-stamp));skew=abs(nearest.stamp-stamp)
    if skew>max_skew:raise ValueError('cloud_odometry_skew')
    if skew<1e-8:return nearest,skew,'exact'
    lower=[p for p in poses if p.stamp<stamp];upper=[p for p in poses if p.stamp>stamp]
    if lower and upper:
        a,b=lower[-1],upper[0]
        if max(stamp-a.stamp,b.stamp-stamp)<=max_skew:
            fraction=(stamp-a.stamp)/(b.stamp-a.stamp)
            qa,qb=a.quaternion/np.linalg.norm(a.quaternion),b.quaternion/np.linalg.norm(b.quaternion)
            dot=float(np.dot(qa,qb))
            if dot<0:qb=-qb;dot=-dot
            dot=min(1.,max(-1.,dot))
            if dot>.9995:q=qa+fraction*(qb-qa)
            else:
                theta=math.acos(dot);q=(math.sin((1-fraction)*theta)*qa+math.sin(fraction*theta)*qb)/math.sin(theta)
            q/=np.linalg.norm(q)
            return Pose(stamp,a.position+fraction*(b.position-a.position),q,a.frame),skew,'interpolated'
    return nearest,skew,'nearest'


def expired_snapshot(snapshot,wall,mono,settings):
    result=dict(snapshot)
    age=max(0.,mono-snapshot['built_mono'])
    expired=(age>settings['snapshot_lease_s'] or mono>snapshot['valid_until_mono'] or
             wall-snapshot['source_stamp']>settings['max_source_age_s'] or wall<snapshot['built_wall']-.001)
    if expired:
        reason='map_or_worker_expired' if snapshot['valid'] or age>settings['snapshot_lease_s'] else snapshot['reason']
        result.update(valid=False,reason=reason,occupancy=np.full_like(snapshot['occupancy'],-1),
                      elevation_points=np.empty((0,3)),obstacle_points=np.empty((0,3)))
    # Source stamp is deliberately never replaced by publication time.
    result['publication_age_s']=age
    return result


class MappingEngine:
    def __init__(self,settings,calibration,cfg,audit_only=False,roi_frame='body'):
        self.settings,self.calibration,self.cfg=settings,calibration,cfg
        self.audit_only=audit_only;self.audit=Audit(settings['max_source_age_s'])
        self.calibration_issues=calibration_issues(calibration,cfg)
        self.poses=deque(maxlen=settings['pose_buffer_size']);self.map=None
        self.cloud_stamp=0.;self.cloud_deadline=-math.inf;self.pose_deadline=-math.inf
        self.last_cloud_receipt=None;self.last_wall=None;self.reset_required=None
        self.last_error='waiting_for_inputs';self.pairing=None;self.accepted_clouds=0
        self.direction_sequence={};self.last_direction=None;self.child_frames=set()
        self.roi=SpatialROI(settings.get('spatial_roi'),expected_frame=roi_frame)
        self.roi_metrics=dict(enabled=self.roi.enabled)
    def clock_check(self,wall):
        if self.last_wall is not None and wall<self.last_wall-.001:
            self.reset_required='ROS_clock_rollback_restart_required';self.map=None;self.poses.clear()
        self.last_wall=wall
    def pose_input(self,stamp,position,quaternion,frame,child,wall,mono):
        if len(self.child_frames)<32:self.child_frames.add(child)
        if not self.audit.observe('odom',stamp,wall,mono,frame):self.pose_deadline=-math.inf;return False
        try:
            pose=Pose(stamp,np.asarray(position,float),np.asarray(quaternion,float),frame).validate()
            if frame!=self.settings['fixed_frame']:
                self.reset_required='odometry_world_frame_mismatch_restart_required';self.map=None
                raise ValueError(self.reset_required)
            if self.calibration.get('reference_frame') and child!=self.calibration['reference_frame']:
                self.reset_required='odometry_child_reference_changed_restart_required';self.map=None
                raise ValueError(self.reset_required)
            if self.poses and np.linalg.norm(pose.position-self.poses[-1].position)>self.settings['max_pose_jump_m']:
                self.reset_required='odometry_jump_restart_required';self.map=None;raise ValueError(self.reset_required)
            self.poses.append(pose);self.pose_deadline=mono+self.settings['max_source_age_s']-max(0.,wall-stamp)
            return True
        except (ValueError,TypeError) as exc:
            self.audit.reject('odom',str(exc));self.pose_deadline=-math.inf;self.last_error=str(exc);return False
    def direction_input(self,message,wall,mono):
        try:
            stamp=float(message['stamp_sec']);frame=message['frame_id']
        except (ValueError,TypeError,KeyError):
            stamp=0.;frame=''
        if not self.audit.observe('direction',stamp,wall,mono,frame):return False
        try:
            if wall-stamp>self.settings['direction']['max_age_s']:raise ValueError('direction_expired')
            vector=np.asarray(message['vector'],float)
            if vector.shape!=(3,) or not np.isfinite(vector).all() or np.linalg.norm(vector)<1e-8:raise ValueError('invalid_direction_vector')
            if message.get('kind')!='direction' or message.get('valid') is not True:raise ValueError('invalid_direction_contract')
            if type(message.get('seq')) is not int or message['seq']<0:raise ValueError('invalid_direction_sequence')
            source,session=message.get('source_id'),message.get('session_id')
            if not isinstance(source,str) or not source or not isinstance(session,str) or not session:raise ValueError('source_and_session_required')
            confidence=float(message.get('confidence',float('nan')))
            if not math.isfinite(confidence) or not 0<=confidence<=1:raise ValueError('invalid_confidence')
            expected=self.settings['direction'].get('expected_frame')
            if frame not in ('body','mic_array') or (expected and frame!=expected):raise ValueError('direction_frame_mismatch')
            if frame=='mic_array':
                rotation=np.asarray(self.calibration.get('mic_rotation_body'),float)
                if rotation.shape!=(3,3) or not np.isfinite(rotation).all() or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(rotation),1.):raise ValueError('microphone_rotation_unmeasured')
            key=(source,session)
            if message['seq']<=self.direction_sequence.get(key,-1):raise ValueError('direction_sequence_replay')
            if len(self.direction_sequence)>=100 and key not in self.direction_sequence:raise ValueError('too_many_direction_sessions_restart_required')
            self.direction_sequence[key]=message['seq'];self.last_direction=dict(frame=frame,source_id=source,session_id=session)
            return True
        except (ValueError,TypeError,KeyError) as exc:self.audit.reject('direction',str(exc));return False
    def cloud_input(self,stamp,points,frame,wall,mono):
        if not self.audit.observe('cloud',stamp,wall,mono,frame):self.cloud_deadline=-math.inf;return False
        try:
            if self.reset_required:raise ValueError(self.reset_required)
            if frame!=self.settings['fixed_frame']:raise ValueError('registered_cloud_frame_mismatch')
            points=np.asarray(points,float)
            if points.ndim!=2 or points.shape[1]!=3 or not 3<=len(points)<=self.settings['max_cloud_points']:raise ValueError('invalid_or_oversize_cloud')
            finite=np.isfinite(points).all(axis=1);invalid=int((~finite).sum());points=points[finite]
            if invalid:self.audit.topics['cloud']['nonfinite_points_dropped']=self.audit.topics['cloud'].get('nonfinite_points_dropped',0)+invalid
            if len(points)<3:raise ValueError('insufficient_finite_cloud_points')
            pose,skew,pairing=interpolate_pose(self.poses,stamp,self.settings['pair_max_skew_s'])
            self.audit.pairs.append(skew);self.pairing=pairing
            if self.audit_only:raise ValueError('audit_only_no_map')
            if self.calibration_issues:raise ValueError('calibration_blocked:'+','.join(self.calibration_issues))
            if self.settings['odom_reference']=='lidar':pose=body_pose_from_lidar(pose,self.cfg)
            if self.calibration.get('self_filter_enabled'):
                body=(points-pose.position)@pose.rotation;keep=np.ones(len(points),bool)
                for box in self.calibration['self_filter_boxes']:
                    lo,hi=validate_box(box);keep&=~np.all((body>lo)&(body<hi),axis=1)
                points=points[keep]
            if len(points)<3:raise ValueError('self_filter_removed_cloud')
            points,self.roi_metrics=self.roi.select(points,pose)
            if len(points)<3:raise ValueError('insufficient_returns_in_spatial_roi')
            if self.map is None:self.map=TerrainMap(self.cfg,pose.position)
            evidence=mono-max(0.,wall-stamp)
            if evidence<=getattr(self,'last_evidence',-math.inf):raise ValueError('nonincreasing_evidence_time')
            # Registered points stay in world frame; do not apply extrinsics twice.
            self.map.integrate(points,pose.position,evidence,pose.rotation)
            self.last_evidence=evidence;self.cloud_stamp=stamp;self.last_cloud_receipt=(wall,mono)
            self.cloud_deadline=evidence+self.settings['max_source_age_s']
            self.accepted_clouds+=1;self.last_error=None;return True
        except (ValueError,TypeError) as exc:
            if str(exc)!='audit_only_no_map' and not str(exc).startswith('calibration_blocked:'):self.audit.reject('cloud',str(exc))
            self.last_error=str(exc);self.cloud_deadline=-math.inf;return False
    def snapshot(self,wall,mono):
        self.clock_check(wall)
        n=int(round(self.cfg['map_size_m']/self.cfg['resolution_m']))
        origin=self.map.origin.copy() if self.map else np.array([-n*self.cfg['resolution_m']/2]*2)
        valid=bool(self.map is not None and not self.reset_required and mono<=min(self.cloud_deadline,self.pose_deadline))
        reason=self.reset_required or self.last_error or ('inputs_expired' if not valid else 'mapping_preview')
        occupancy=np.full((n,n),-1,dtype=np.int8);elevation=np.empty((0,3));obstacles=np.empty((0,3));support=False
        if valid:
            terrain=self.map.analyze(mono);occupancy=terrain['grid'].copy();support=bool(terrain['support_valid'])
            yy,xx=np.nonzero(terrain['known'])
            elevation=np.column_stack([origin[0]+(xx+.5)*self.map.res,origin[1]+(yy+.5)*self.map.res,self.map.elevation[yy,xx]])
            retained=[]
            for timestamp,(raw,mask) in self.map.obstacle_frames.items():
                if mono-timestamp>self.cfg['obstacle_hold_s']:continue
                cloud=raw[mask];cells=np.floor((cloud[:,:2]-origin)/self.map.res).astype(int)
                inside=(cells[:,0]>=0)&(cells[:,0]<n)&(cells[:,1]>=0)&(cells[:,1]<n)
                cloud,cells=cloud[inside],cells[inside]
                retained.append(cloud[timestamp>self.map.obstacle_cleared[cells[:,1],cells[:,0]]])
            if retained:obstacles=np.concatenate(retained)
        body=None
        if valid and self.poses:
            pose=self.poses[-1]
            if self.settings['odom_reference']=='lidar':pose=body_pose_from_lidar(pose,self.cfg)
            body=dict(stamp=pose.stamp,position=pose.position.tolist(),quaternion=pose.quaternion.tolist())
            if self.roi.enabled:
                yy,xx=np.indices(occupancy.shape)
                centres=np.column_stack([origin[0]+(xx.ravel()+.5)*self.cfg['resolution_m'],
                                         origin[1]+(yy.ravel()+.5)*self.cfg['resolution_m'],np.zeros(xx.size)])
                occupancy[~self.roi.coverage(centres,pose).reshape(occupancy.shape)]=-1
                elevation=elevation[self.roi.coverage(elevation,pose)]
                obstacles=obstacles[self.roi.coverage(obstacles,pose)]
        audit=self.audit.report(mono)
        audit.update(calibration_issues=list(self.calibration_issues),odometry_child_frames=sorted(self.child_frames),
                     direction_contract=self.settings['direction'],direction_source=self.last_direction,
                     tf_chain_verified=False,accepted_clouds=self.accepted_clouds,pairing_method=self.pairing,
                     obstacle_evidence_hold_s=self.cfg['obstacle_hold_s'],ground_fusion_window_s=self.cfg['fusion_window_s'],
                     motion_commands_sent=0)
        audit['spatial_roi']=dict(self.roi_metrics)
        return dict(valid=valid,reason=reason,frame=self.settings['fixed_frame'],source_stamp=self.cloud_stamp,
                    built_wall=wall,built_mono=mono,valid_until_mono=min(self.cloud_deadline,self.pose_deadline),
                    resolution=self.cfg['resolution_m'],origin=origin,occupancy=occupancy,elevation_points=elevation,
                    obstacle_points=obstacles,body_pose=body,support_valid=support,audit=audit)


class ReceiveMeter:
    """Callback-delivery rates, measured before application queue drops."""
    def __init__(self):self.lock=threading.Lock();self.topics={}
    def observe(self,kind,stamp,mono):
        with self.lock:
            row=self.topics.setdefault(kind,dict(count=0,values=deque(maxlen=1000)))
            row['count']+=1
            if math.isfinite(stamp) and math.isfinite(mono):row['values'].append((stamp,mono))
    def report(self):
        with self.lock:rows={key:(row['count'],list(row['values'])) for key,row in self.topics.items()}
        result={}
        for key,(count,values) in rows.items():
            delta=values[-1][1]-values[0][1] if len(values)>1 else 0.
            source_delta=values[-1][0]-values[0][0] if len(values)>1 else 0.
            gaps=np.diff([v[1] for v in values]) if len(values)>1 else []
            result[key]=dict(callback_count=count,receive_hz=(len(values)-1)/delta if delta>0 else None,
                source_hz=(len(values)-1)/source_delta if source_delta>0 else None,
                max_callback_gap_s=float(max(gaps)) if len(gaps) else None)
        return result
