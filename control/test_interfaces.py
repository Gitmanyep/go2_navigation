#!/usr/bin/env python3
import copy,datetime,importlib.abc,json,math,sys
from pathlib import Path
import numpy as np

class NoHardware(importlib.abc.MetaPathFinder):
 def find_spec(self,fullname,path=None,target=None):
  if fullname.startswith(('unitree_sdk2py','cyclonedds')):raise RuntimeError('SDK/DDS forbidden in static tests')
sys.meta_path.insert(0,NoHardware())
import manual_panel as manual
from direction_adapter import DirectionAdapter
results=[]
def check(name,fn):
 fn();results.append({'case':name,'passed':True});print('PASS',name,flush=True)
def expect_error(fn,error=ValueError):
 try:fn()
 except error:return
 raise AssertionError('Expected '+error.__name__)
class Fake:
 def __init__(self):self.calls=[];self.move_error=False;self.stop_error=False
 def Move(self,*args):
  self.calls.append(('Move',args))
  return 5 if self.move_error and any(args) else 0
 def StopMove(self):self.calls.append(('StopMove',()));return 5 if self.stop_error else 0
def setup():
 t=[0.];sink=Fake();c=manual.PulseController(sink,clock=lambda:t[0]);return t,sink,c
def no_startup_motion():
 t,s,c=setup();c.tick();c.close();assert s.calls==[]
check('no_startup_or_unused_close_motion',no_startup_motion)
for direction,velocity in manual.DIRECTIONS.items():
 def run(v=velocity):
  t,s,c=setup();assert c.request(v);c.tick();assert s.calls[-1]==('Move',v)
  t[0]=.31;c.tick();assert s.calls[-2:]==[('Move',(0.,0.,0.)),('StopMove',())]
  assert not c.state()['active']
 check('pulse_'+direction+'_and_deadline_stop',run)
def no_extension():
 t,s,c=setup();c.request((.1,0,0));t[0]=.29
 assert not c.request((-.1,0,0),.5);assert c.deadline==.3
 t[0]=.31;c.tick();assert not c.state()['active'];assert all(not any(args) for name,args in s.calls if name=='Move')
check('busy_click_does_not_queue_or_extend_deadline',no_extension)
for label,v,d in [('nan_velocity',(float('nan'),0,0),.3),('inf_velocity',(0,0,float('inf')),.3),
 ('over_speed',(.2,0,0),.3),('diagonal_over_speed',(.1,.1,0),.3),('over_turn',(0,0,.5),.3),
 ('nan_duration',(.1,0,0),float('nan')),('unbounded_duration',(.1,0,0),20.)]:
 def run(v=v,d=d):
  t,s,c=setup();c.request((.1,0,0));expect_error(lambda:c.request(v,d));c.tick()
  assert s.calls==[('Move',(0.,0.,0.)),('StopMove',())]
 check('reject_'+label+'_and_cancel_active',run)
def manual_stop():
 t,s,c=setup();c.request((.1,0,0));c.tick();c.stop('focus_lost');c.tick()
 assert not c.state()['active'] and s.calls[-1][0]=='StopMove'
check('focus_or_operator_stop_cancels_active',manual_stop)
def close_active():
 t,s,c=setup();c.request((.1,0,0));c.close();assert s.calls[-1][0]=='StopMove'
 expect_error(lambda:c.request((.1,0,0)),RuntimeError)
check('close_active_stops_and_rejects_new_commands',close_active)
def move_fault():
 t,s,c=setup();s.move_error=True;c.request((.1,0,0));c.tick()
 assert c.fault and s.calls[-1][0]=='StopMove'
 expect_error(lambda:c.request((.1,0,0)),RuntimeError)
check('move_error_requests_stop_and_locks_out',move_fault)
def stop_fault():
 t,s,c=setup();s.stop_error=True;c.request((.1,0,0));t[0]=1;c.tick()
 assert c.fault;expect_error(lambda:c.request((.1,0,0)),RuntimeError)
check('unconfirmed_stop_locks_out',stop_fault)

now=1800000000.
base={'seq':1,'stamp_sec':now,'frame_id':'body','kind':'direction','vector':[1.,0.,0.],'confidence':.9,'valid':True}
pose={'stamp_sec':now,'frame_id':'camera_init','position':[1.,2.,.3],'orientation_xyzw':[0.,0.,0.,1.]}
for label,v,expected in [('forward',[1,0,0],[1.5,2,.3]),('backward',[-1,0,0],[.5,2,.3]),
 ('left',[0,1,0],[1,2.5,.3]),('right',[0,-1,0],[1,1.5,.3]),('right_front',[1,-1,0],[1+math.sqrt(.125),2-math.sqrt(.125),.3])]:
 def run(v=v,expected=expected):
  m=copy.deepcopy(base);m['vector']=v;r=DirectionAdapter().convert(m,pose,now)
  assert r['status']=='candidate_only' and np.allclose(r['candidate_position_m'],expected)
  assert r['executable'] is False and r['route_status']=='not_planned'
 check('direction_'+label,run)
for name,changes in [('zero',{'vector':[0,0,0]}),('vertical',{'vector':[0,0,1]}),
 ('nan_vector',{'vector':[float('nan'),0,0]}),('inf_vector',{'vector':[float('inf'),0,0]}),
 ('old',{'stamp_sec':now-3}),('future',{'stamp_sec':now+3}),('low_confidence',{'confidence':.1}),
 ('invalid',{'valid':False}),('unknown_frame',{'frame_id':'map'}),('uncalibrated_mic',{'frame_id':'mic_array'}),
 ('wrong_kind',{'kind':'relative_position'}),('bad_sequence',{'seq':True}),('wrong_dimension',{'vector':[1,2]})]:
 def run(changes=changes):
  m=copy.deepcopy(base);m.update(changes);r=DirectionAdapter().convert(m,pose,now)
  assert r['status']=='waiting' and r['executable'] is False and r['candidate_position_m'] is None
 check('direction_reject_'+name,run)
for name,changes in [('stale_pose',{'stamp_sec':now-2}),('mismatched_pose_time',{'stamp_sec':now-.2}),
 ('nan_pose',{'position':[float('nan'),0,0]}),('bad_quaternion',{'orientation_xyzw':[0,0,0,0]})]:
 def run(changes=changes):
  pp=copy.deepcopy(pose);pp.update(changes);assert DirectionAdapter().convert(base,pp,now)['status']=='waiting'
 check(name,run)
def sequence():
 a=DirectionAdapter();assert a.convert(base,pose,now)['status']=='candidate_only'
 assert a.convert(base,pose,now)['status']=='waiting'
check('direction_replay_rejected',sequence)
def calibrated_mic():
 a=DirectionAdapter([[0,-1,0],[1,0,0],[0,0,1]])
 m=copy.deepcopy(base);m['frame_id']='mic_array'
 r=a.convert(m,pose,now);assert np.allclose(r['candidate_position_m'],[1,2.5,.3])
check('calibrated_mic_rotation',calibrated_mic)
def yaw():
 pp=copy.deepcopy(pose);pp['orientation_xyzw']=[0,0,math.sqrt(.5),math.sqrt(.5)]
 r=DirectionAdapter().convert(base,pp,now);assert np.allclose(r['candidate_position_m'],[1,2.5,.3])
check('body_direction_transformed_with_pose',yaw)
check('reflection_is_not_a_rotation',lambda:expect_error(lambda:DirectionAdapter([[-1,0,0],[0,1,0],[0,0,1]])))
report={'recorded_at':datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),
 'mode':'Static test using fake client and fake time; SDK/DDS imports blocked',
 'case_count':len(results),'all_passed':all(r['passed'] for r in results),'cases':results,
 'robot_commands_sent':0,'limitations':['No physical stop or controller-takeover test','No path planner or obstacle avoidance implemented by the manual panel or direction preview']}
Path(__file__).with_name('interface_test_results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print('SUMMARY',json.dumps({k:report[k] for k in ['case_count','all_passed','robot_commands_sent']}),flush=True)
