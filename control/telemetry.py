"""Passive telemetry recording and estimates; no SDK imports or motion APIs."""
import csv
import datetime
import json
import math
import threading
import time
from collections import deque
from pathlib import Path


def finite_vector(value, size=3):
    try:
        result=[float(x) for x in value]
        return result if len(result)==size and all(math.isfinite(x) for x in result) else None
    except (TypeError,ValueError,OverflowError):
        return None


def decode(message, received):
    velocity=finite_vector(getattr(message,'velocity',None))
    if velocity is None:
        raise ValueError('速度反馈缺失或无效')
    position=finite_vector(getattr(message,'position',None))
    rpy=finite_vector(getattr(getattr(message,'imu_state',None),'rpy',None))
    try:
        yaw_rate=float(message.yaw_speed)
        if not math.isfinite(yaw_rate):yaw_rate=None
    except (AttributeError,TypeError,ValueError,OverflowError):yaw_rate=None
    speed=math.hypot(*velocity[:2])
    if not math.isfinite(speed):raise ValueError('平面速度计算溢出')
    return dict(received=received,velocity=velocity,position=position,
                yaw=rpy[2] if rpy else None,yaw_rate=yaw_rate,
                speed=speed)


class TelemetryRecorder:
    COLUMNS=('wall_time','monotonic_s','segment','vx','vy','vz','speed_mps',
             'yaw_rate_rps','position_x','position_y','position_z','yaw_rad')
    def __init__(self,output,clock=time.monotonic,source='robot_internal_estimate'):
        self.clock,self.lock=clock,threading.RLock()
        self.source=source
        self.output=Path(output);self.output.mkdir(parents=True,exist_ok=False)
        self.stream=(self.output/'feedback.csv').open('w',newline='',buffering=1)
        self.writer=csv.writer(self.stream);self.writer.writerow(self.COLUMNS)
        self.history=deque(maxlen=1200)
        self.latest=None;self.closed=False;self.invalid_samples=0;self.write_error=None
        self.last_saved=None;self.segment=0
        self.reset()

    def reset(self):
        with self.lock:
            self.segment+=1;self.origin=None;self.last_yaw=None
            self.yaw_change=0.;self.yaw_samples=0;self.peak_speed=0.;self.peak_yaw_rate=None
            self.samples=0;self.started=None;self.yaw_continuous=True
            self.history.clear()

    def observe(self,message):
        now=self.clock()
        try:sample=decode(message,now)
        except ValueError:
            with self.lock:self.invalid_samples+=1
            return False
        with self.lock:
            if self.closed:return False
            if self.last_saved is not None and now-self.last_saved<.049:return False
            # Do not join missing yaw or long reception gaps into a turn estimate.
            if self.samples and self.latest and now-self.latest['received']>.3:
                self.last_yaw=None;self.yaw_continuous=False
            if self.started is None:self.started=now
            self.latest=sample;self.history.append(sample);self.last_saved=now;self.samples+=1
            self.peak_speed=max(self.peak_speed,sample['speed'])
            if sample['yaw_rate'] is not None:
                self.peak_yaw_rate=max(self.peak_yaw_rate or 0.,abs(sample['yaw_rate']))
            if self.origin is None and sample['position'] is not None:self.origin=list(sample['position'])
            if sample['yaw'] is None:
                self.last_yaw=None;self.yaw_continuous=False
            else:
                if self.last_yaw is not None:
                    delta=math.atan2(math.sin(sample['yaw']),math.cos(sample['yaw']))-math.atan2(math.sin(self.last_yaw),math.cos(self.last_yaw))
                    self.yaw_change+=math.atan2(math.sin(delta),math.cos(delta))
                self.last_yaw=sample['yaw'];self.yaw_samples+=1
            position=sample['position'] or [None]*3
            wall=datetime.datetime.now(datetime.timezone.utc).isoformat()
            try:
                self.writer.writerow([wall,now,self.segment,*sample['velocity'],sample['speed'],
                                      sample['yaw_rate'],*position,sample['yaw']])
            except (OSError,ValueError) as exc:self.write_error=str(exc)
            return True

    def snapshot(self):
        with self.lock:
            latest=dict(self.latest) if self.latest else None
            age=max(0.,self.clock()-latest['received']) if latest else None
            displacement=None
            if latest and latest['position'] is not None and self.origin is not None:
                displacement=math.hypot(latest['position'][0]-self.origin[0],latest['position'][1]-self.origin[1])
                if not math.isfinite(displacement):displacement=None
            return dict(latest=latest,receive_age_s=age,fresh=age is not None and age<=.3,
                        segment=self.segment,samples=self.samples,peak_speed_mps=self.peak_speed if self.samples else None,
                        peak_yaw_rate_rps=self.peak_yaw_rate,displacement_m=displacement,
                        yaw_change_rad=self.yaw_change if self.yaw_samples>=2 and self.yaw_continuous else None,
                        invalid_samples=self.invalid_samples,write_error=self.write_error,
                        duration_s=latest['received']-self.started if latest and self.started is not None else None)

    def save_summary(self):
        with self.lock:
            result=self.snapshot()
            result.update(saved_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                          source=self.source,physical_motion_independently_verified=False)
            with (self.output/'segments.jsonl').open('a') as stream:
                stream.write(json.dumps(result,ensure_ascii=False,allow_nan=False)+'\n')
            return result

    def close(self):
        with self.lock:
            if self.closed:return
            try:self.save_summary()
            finally:self.closed=True;self.stream.close()
