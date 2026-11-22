#!/usr/bin/env python3
"""Bird direction -> expiring candidate goal. Never publishes motion commands."""
import argparse,collections,json,math,sys,time
import numpy as np

class DirectionAdapter:
    def __init__(self, mic_rotation=None, step_m=.5, max_age_s=.5, min_confidence=.6):
        self.rotation=None if mic_rotation is None else np.asarray(mic_rotation,dtype=float)
        if self.rotation is not None:
            r=self.rotation
            if r.shape!=(3,3) or not np.isfinite(r).all() or not np.allclose(r.T@r,np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(r),1.,atol=1e-5):
                raise ValueError('mic rotation must be a calibrated proper rotation matrix')
        for v in [step_m,max_age_s,min_confidence]:
            if not math.isfinite(v):raise ValueError('nonfinite configuration')
        if not 0<step_m<=1 or not 0<max_age_s<=2 or not 0<=min_confidence<=1:
            raise ValueError('configuration outside preview limits')
        self.step_m,self.max_age,self.min_confidence=step_m,max_age_s,min_confidence
        self.last_seq=-1
    def convert(self,message,pose,now=None):
        now=time.time() if now is None else now
        try:
            if not isinstance(message,dict) or not isinstance(pose,dict):raise ValueError('message and pose must be objects')
            if message.get('kind')!='direction':raise ValueError('only kind=direction is supported')
            if message.get('valid') is not True:raise ValueError('source marked invalid')
            seq=message.get('seq')
            if not isinstance(seq,int) or isinstance(seq,bool) or seq<0 or seq<=self.last_seq:
                raise ValueError('invalid or replayed sequence')
            stamp=float(message['stamp_sec']);confidence=float(message['confidence'])
            if not all(math.isfinite(v) for v in [now,stamp,confidence]):raise ValueError('nonfinite metadata')
            if not 0<=confidence<=1 or confidence<self.min_confidence:raise ValueError('low or invalid confidence')
            if not -.05<=now-stamp<=self.max_age:raise ValueError('stale or future source time')
            vector=np.asarray(message['vector'],dtype=float)
            if vector.shape!=(3,) or not np.isfinite(vector).all():raise ValueError('vector must contain 3 finite components')
            norm=float(np.linalg.norm(vector))
            if not math.isfinite(norm) or norm<1e-8:raise ValueError('zero or invalid direction')
            direction=vector/norm
            frame=message['frame_id']
            if frame=='mic_array':
                if self.rotation is None:raise ValueError('mic-to-body rotation is not calibrated')
                direction=self.rotation@direction
            elif frame!='body':raise ValueError('unsupported source frame')
            p=np.asarray(pose['position'],dtype=float);q=np.asarray(pose['orientation_xyzw'],dtype=float)
            pt=float(pose['stamp_sec'])
            if p.shape!=(3,) or q.shape!=(4,) or not np.isfinite(p).all() or not np.isfinite(q).all() or not math.isfinite(pt):
                raise ValueError('invalid pose')
            if not -.05<=now-pt<=self.max_age or abs(pt-stamp)>.12:raise ValueError('pose is stale or not matched to the source time')
            if abs(float(np.linalg.norm(q))-1)>.01:raise ValueError('invalid pose quaternion')
            q=q/np.linalg.norm(q);x,y,z,w=q
            rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                               [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                               [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
            world=rotation@direction;horizontal=float(np.linalg.norm(world[:2]))
            if horizontal<.1:raise ValueError('direction is nearly vertical; no reliable ground direction')
            goal=p.copy();goal[:2]+=self.step_m*world[:2]/horizontal
            world_frame=pose.get('frame_id')
            if not isinstance(world_frame,str) or not world_frame:raise ValueError('pose frame is missing')
            self.last_seq=seq
            return dict(status='candidate_only',seq=seq,source_stamp_sec=stamp,
                expires_at_sec=stamp+self.max_age,frame_id=world_frame,
                direction_body=direction.tolist(),direction_world=world.tolist(),
                azimuth_body_rad=math.atan2(direction[1],direction[0]),
                elevation_world_rad=math.atan2(world[2],horizontal),
                candidate_position_m=goal.tolist(),step_m=self.step_m,
                requires_terrain_and_collision_check=True,route_status='not_planned',executable=False)
        except (ValueError,TypeError,KeyError,OverflowError) as e:
            return dict(status='waiting',reason=str(e),candidate_position_m=None,
                        expires_at_sec=0,route_status='not_planned',executable=False)

def ros_main(adapter):
    import rospy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    poses=collections.deque(maxlen=300)
    rospy.init_node('go2_direction_preview')
    output=rospy.Publisher('/go2/navigation/goal_preview',String,queue_size=1,latch=False)
    def odom(msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        poses.append(dict(stamp_sec=msg.header.stamp.to_sec(),frame_id=msg.header.frame_id,
                          position=[p.x,p.y,p.z],orientation_xyzw=[q.x,q.y,q.z,q.w]))
    last=[None,0.]
    def publish(result):
        text=json.dumps(result,ensure_ascii=False,allow_nan=False)
        output.publish(String(data=text));print(text,flush=True)
    def direction(msg):
        try:
            message=json.loads(msg.data,parse_constant=lambda s:float(s))
            if not isinstance(message,dict):raise ValueError('message must be an object')
            stamp=float(message['stamp_sec']);history=list(poses)
            if not math.isfinite(stamp) or not history:raise ValueError('valid timestamp and odometry required')
            pose=min(history,key=lambda p:abs(p['stamp_sec']-stamp))
            result=adapter.convert(message,pose,now=rospy.Time.now().to_sec())
        except (ValueError,KeyError,TypeError,OverflowError) as e:
            result=dict(status='waiting',reason=str(e),candidate_position_m=None,expires_at_sec=0,executable=False)
        last[0]=result
        # Bound validity by elapsed time even if the wall/ROS clock goes backwards.
        last[1]=time.monotonic()+max(0.,min(adapter.max_age,result.get('expires_at_sec',0)-rospy.Time.now().to_sec()))
        publish(result)
    def expire(_):
        current=last[0]
        if current and current.get('status')=='candidate_only' and (time.monotonic()>=last[1] or rospy.Time.now().to_sec()>current['expires_at_sec']):
            result=dict(status='waiting',reason='direction expired',candidate_position_m=None,expires_at_sec=0,executable=False)
            last[0]=result;publish(result)
    rospy.Subscriber('/aft_mapped_to_init',Odometry,odom,queue_size=30)
    rospy.Subscriber('/go2/navigation/direction',String,direction,queue_size=1)
    rospy.Timer(rospy.Duration(.05),expire)
    print('Preview interface: /go2/navigation/direction -> /go2/navigation/goal_preview; no motion output',flush=True)
    rospy.spin()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    mode=p.add_mutually_exclusive_group(required=True);mode.add_argument('--demo',action='store_true');mode.add_argument('--stdin',action='store_true');mode.add_argument('--ros',action='store_true')
    p.add_argument('--mic-rotation-file');p.add_argument('--step-m',type=float,default=.5)
    args=p.parse_args();r=None
    if args.mic_rotation_file:
        with open(args.mic_rotation_file) as f:r=json.load(f)
    adapter=DirectionAdapter(r,step_m=args.step_m)
    if args.ros:return ros_main(adapter)
    if args.demo:
        now=time.time();message={'seq':1,'stamp_sec':now,'frame_id':'body','kind':'direction','vector':[1.,-1.,.2],'confidence':.9,'valid':True}
        pose={'stamp_sec':now,'frame_id':'camera_init','position':[0.,0.,0.],'orientation_xyzw':[0.,0.,0.,1.]}
        print(json.dumps({'demo_only':True,'input':message,'result':adapter.convert(message,pose,now)},ensure_ascii=False,indent=2))
        return
    for line in sys.stdin:
        try:
            request=json.loads(line);result=adapter.convert(request['message'],request['pose'])
        except (ValueError,KeyError,TypeError) as e:result={'status':'waiting','reason':str(e),'executable':False}
        print(json.dumps(result,ensure_ascii=False,allow_nan=False),flush=True)
if __name__=='__main__':main()
