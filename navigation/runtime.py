"""Bounded ingress and expiring immutable results. No ROS or robot SDK dependency.

One worker owns navigation. Publishers never acquire its map/planner lock.
A result lease starts BEFORE computation, so late work cannot revive old velocity.
This is a preview watchdog, not a certified hardware safety controller.
"""
from collections import deque
import copy
import math
import threading
import time


class PreviewRuntime:
    def __init__(self,processor,period=.1,lease=.15,clock=time.monotonic):
        self.processor=processor;self.period=period;self.lease=lease;self.clock=clock
        self.lock=threading.Lock();self.stop_event=threading.Event()
        self.poses=deque(maxlen=300);self.latest={};self.epoch=0
        self.snapshot=None;self.deadline=-math.inf;self.forced_reason=None
        self.thread=None

    def submit(self,topic,payload):
        with self.lock:
            if topic in ('operator','error'):self.epoch+=1
            if topic=='pose':self.poses.append(payload)
            else:self.latest[topic]=payload
            if topic in ('operator','error'):
                self.forced_reason='operator_command_pending' if topic=='operator' else str(payload)
                self.deadline=-math.inf

    def compute_once(self):
        with self.lock:
            epoch=self.epoch;events=dict(self.latest);self.latest.clear()
            events['poses']=list(self.poses);self.poses.clear()
        started=self.clock()
        try:
            snapshot,source_deadline=self.processor(events)
        except Exception as exc:
            snapshot=dict(state=dict(reason='worker_error:'+type(exc).__name__,candidate=[0.,0.],path=[]))
            source_deadline=-math.inf
        snapshot=copy.deepcopy(snapshot)
        finished=self.clock()
        with self.lock:
            # Invalidation/confirmation wins over in-flight work. Normal high-rate
            # observations queue for the next batch without starving publication.
            # They cannot extend this batch's original result/source lease.
            if self.epoch!=epoch:return False
            self.snapshot=snapshot
            self.deadline=min(source_deadline,started+self.lease)
            self.forced_reason=None
            return finished<self.deadline

    def read(self):
        # Copy only references under the lock; published snapshots are never mutated.
        with self.lock:
            snap=self.snapshot;deadline=self.deadline;reason=self.forced_reason
        if snap is None:snap=dict(state=dict(reason='worker_initializing',candidate=[0.,0.],path=[]))
        result=dict(snap);state=dict(snap['state']);result['state']=state
        if reason or self.clock()>=deadline:
            state.update(candidate=[0.,0.],reason=reason or 'preview_result_or_sensor_expired')
        state['watchdog_expired']=bool(reason or self.clock()>=deadline)
        return result

    def start(self):
        def work():
            while not self.stop_event.is_set():
                started=self.clock();self.compute_once()
                self.stop_event.wait(max(0.,self.period-(self.clock()-started)))
        self.thread=threading.Thread(target=work,name='navigation-worker',daemon=True);self.thread.start()

    def close(self):
        self.stop_event.set()
        if self.thread:self.thread.join(timeout=1.)
