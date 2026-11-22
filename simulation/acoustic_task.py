"""Direction-only seek/listen task. Completion requires an independent human command.
No source coordinates, range or evaluation truth enter this state machine.
"""
import math
import uuid
import numpy as np


class AcousticTask:
    def __init__(self,cfg):
        self.cfg=cfg
        self.state='WAITING'
        self.state_since=0.
        self.next_listen=None
        self.confirmed=False
        self.confirm_seq=-1
        self.confirm_stamp=-math.inf
        self.confirmed_task_id=None
        self.task_id='bird-search-'+uuid.uuid4().hex[:12]
        self.stop_since=None
        self.stop_source_since=None
        self.last_observed_mono=None
        self._source_origin=None
        self.last_observed_pose=None
        self.linear_speed=None
        self.yaw_speed=None
        self.reason='waiting_for_direction'
        self.transitions=[]

    def transition(self,state,mono,reason):
        if state!=self.state:
            self.transitions.append(dict(t=float(mono),old=self.state,new=state,reason=reason))
            self.state=state;self.state_since=mono
        self.reason=reason

    def invalidate_observation(self):
        self.stop_since=None;self.stop_source_since=None

    def observe_pose(self,pose,mono=None):
        # Optional source-relative clock keeps standalone replay/tests deterministic.
        if self._source_origin is None:self._source_origin=pose.stamp
        if mono is None:mono=pose.stamp-self._source_origin
        prev=self.last_observed_pose
        gap=self.cfg['task_observation_gap_s']
        continuous=(prev is not None and 0<pose.stamp-prev.stamp<=gap and
                    self.last_observed_mono is not None and 0<mono-self.last_observed_mono<=gap)
        if prev is not None and pose.stamp>prev.stamp:
            dt=pose.stamp-prev.stamp
            self.linear_speed=float(np.linalg.norm(pose.position-prev.position)/dt)
            angle=(pose.yaw-prev.yaw+math.pi)%(2*math.pi)-math.pi
            self.yaw_speed=abs(angle/dt)
        stopped=(continuous and self.linear_speed<self.cfg['task_stop_linear_mps'] and
                 self.yaw_speed<self.cfg['task_stop_yaw_rps'])
        if not stopped:self.invalidate_observation()
        elif self.confirmed and self.stop_since is None:
            self.stop_since=mono;self.stop_source_since=pose.stamp
        self.last_observed_pose=pose;self.last_observed_mono=mono

    def confirm(self,message,now,mono):
        # This is a separate operator channel, never read from acoustic direction packets.
        try:
            if message.get('task_id')!=self.task_id or message.get('action')!='confirm_found' or message.get('confirmed') is not True:
                raise ValueError('invalid_operator_confirmation')
            seq=message['seq'];stamp=float(message['stamp_sec'])
            if type(seq) is not int or seq<=self.confirm_seq or not math.isfinite(stamp) or stamp<=self.confirm_stamp or not -.05<=now-stamp<=.5:
                raise ValueError('stale_or_replayed_confirmation')
            self.confirm_seq,self.confirm_stamp=seq,stamp
            self.confirmed=True;self.confirmed_task_id=self.task_id
            self.invalidate_observation()
            self.transition('VERIFY_STOP',mono,'operator_confirmed_found')
            return True
        except (KeyError,TypeError,ValueError,OverflowError):return False

    def tick(self,mono,direction_valid,pose_fresh=True):
        observed_recent=(self.last_observed_mono is not None and -1e-8<=mono-self.last_observed_mono<=self.cfg['task_observation_gap_s'])
        stopped=(pose_fresh and observed_recent and self.linear_speed is not None and self.yaw_speed is not None and
                 self.linear_speed<self.cfg['task_stop_linear_mps'] and self.yaw_speed<self.cfg['task_stop_yaw_rps'])
        if self.state=='COMPLETE':return False
        if self.confirmed:
            if stopped:
                # Count only consecutive observed stationary intervals; timer ticks
                # alone cannot establish standstill between missing observations.
                span=0. if self.stop_since is None else min(self.last_observed_mono-self.stop_since,self.last_observed_pose.stamp-self.stop_source_since)
                if span+1e-9>=self.cfg['task_settle_s']:
                    self.transition('COMPLETE',mono,'operator_confirmed_and_odometry_settled')
            else:
                self.invalidate_observation()
                self.transition('VERIFY_STOP',mono,'waiting_for_fresh_stopped_odometry')
            return False
        if not pose_fresh:
            self.transition('LOST',mono,'pose_unavailable');return False
        if not direction_valid:
            if self.state not in ['LOST','SEARCH','WAITING']:
                self.transition('LOST',mono,'direction_lost')
            if self.state=='LOST' and mono-self.state_since>=self.cfg['task_search_timeout_s']:
                self.transition('SEARCH',mono,'waiting_for_reacquisition_or_operator')
            return False
        if self.state in ['WAITING','LOST','SEARCH']:
            self.transition('STOP_LISTEN',mono,'direction_acquired_listen_first')
        if self.state=='STOP_LISTEN':
            if mono-self.state_since>=self.cfg['task_listen_duration_s'] and stopped:
                self.transition('APPROACH',mono,'listen_complete')
                self.next_listen=mono+self.cfg['task_listen_interval_s']
                return True
            return False
        if self.state=='APPROACH' and mono>=self.next_listen:
            self.transition('STOP_LISTEN',mono,'periodic_stationary_listening');return False
        return self.state=='APPROACH'

    def status(self):
        return dict(state=self.state,reason=self.reason,task_id=self.task_id,
                    completion_requires='fresh_operator_confirm_found_and_stopped_odometry',
                    operator_confirmed=self.confirmed,observed_speed_mps=self.linear_speed,
                    observed_yaw_rate_rps=self.yaw_speed)
