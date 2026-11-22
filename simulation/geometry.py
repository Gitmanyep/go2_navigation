"""Configurable body/head boxes and explicitly calibrated sensor transforms."""
import math
import numpy as np
from scipy import ndimage


class Geometry:
    def __init__(self,cfg):
        self.cfg=cfg
        self.boxes=[]
        self._kernels={}
        for box in cfg['geometry_boxes']:
            lo,hi=np.asarray(box['minimum'],float),np.asarray(box['maximum'],float)
            if lo.shape!=(3,) or hi.shape!=(3,) or not np.isfinite([lo,hi]).all() or np.any(hi<=lo):
                raise ValueError('invalid_geometry_box')
            self.boxes.append((box['name'],lo,hi))

    def collision(self,points,position,rotation,margin=None):
        if not len(points):return None
        p=(np.asarray(points)-position)@rotation
        m=self.cfg['geometry_margin_m'] if margin is None else margin
        for name,lo,hi in self.boxes:
            if np.any(np.all((p>=lo-m)&(p<=hi+m),axis=1)):
                return name
        return None

    def footprint_unknown(self,unknown_xy,position,yaw,res):
        if not len(unknown_xy):return False
        c,s=math.cos(yaw),math.sin(yaw)
        local=(unknown_xy-np.asarray(position)[:2])@np.array([[c,-s],[s,c]])
        margin=self.cfg['geometry_margin_m']+res*math.sqrt(.5)
        return any(np.any(np.all((local>=lo[:2]-margin)&(local<=hi[:2]+margin),axis=1)) for _,lo,hi in self.boxes)

    def kernel(self,lo,hi,angle,res):
        key=(tuple(lo),tuple(hi),angle,res)
        if key in self._kernels:return self._kernels[key]
        radius=int(math.ceil(max(np.linalg.norm(lo[:2]),np.linalg.norm(hi[:2]))/res))+2
        yy,xx=np.mgrid[-radius:radius+1,-radius:radius+1]
        # Test cell centres with half a cell diagonal of conservative padding.
        x=(xx*math.cos(angle)+yy*math.sin(angle))*res
        y=(-xx*math.sin(angle)+yy*math.cos(angle))*res
        margin=self.cfg['geometry_margin_m']+res*math.sqrt(.5)
        kernel=((x>=lo[0]-margin)&(x<=hi[0]+margin)&(y>=lo[1]-margin)&(y<=hi[1]+margin)).astype(np.int16)
        self._kernels[key]=kernel
        return kernel

    def heading_masks(self,terrain,res):
        masks=[]
        low,high=terrain['obstacle_low'],terrain['obstacle_high']
        for i in range(16):
            angle=i*math.pi/8
            blocked=terrain['blocked'].copy()
            for _,lo,hi in self.boxes:
                base=self.cfg['body_reference_height_m']
                source=((low<=base+hi[2]+self.cfg['geometry_margin_m'])&(high>=base+lo[2]-self.cfg['geometry_margin_m']))
                if 'known' in terrain:source|=~terrain['known']
                if not source.any():continue
                kernel=self.kernel(lo,hi,angle,res)
                blocked|=ndimage.correlate(source.astype(np.int16),kernel,mode='constant',cval=0)>0
            masks.append(blocked)
        return masks


class CloudAdapter:
    """Raw sensor points require a declared calibrated lidar-to-body transform.
    Registered Point-LIO points must remain in their existing world frame.
    """
    def __init__(self,cfg):
        self.cfg=cfg
        self.geometry=Geometry(cfg)
        self.translation=np.asarray(cfg['lidar_position_body_m'],float)
        self.rotation=np.asarray(cfg['lidar_rotation_body'],float)
        if self.translation.shape!=(3,) or not np.isfinite(self.translation).all() or self.rotation.shape!=(3,3) or not np.isfinite(self.rotation).all() or not np.allclose(self.rotation.T@self.rotation,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(self.rotation),1.):
            raise ValueError('invalid_lidar_extrinsics')

    def transform(self,points,pose):
        if not self.cfg['lidar_extrinsics_calibrated']:
            raise ValueError('lidar_extrinsics_unmeasured')
        p=np.asarray(points,float)
        if p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(p).all():raise ValueError('invalid_raw_cloud')
        body=p@self.rotation.T+self.translation
        removed=0
        if self.cfg['self_filter_enabled']:
            if not self.cfg['geometry_calibrated']:raise ValueError('self_filter_geometry_unmeasured')
            keep=np.ones(len(body),bool)
            for _,lo,hi in self.geometry.boxes:
                # Only the declared solid interior; never suppress a surrounding radius.
                keep&=~np.all((body>lo)&(body<hi),axis=1)
            removed=int((~keep).sum());body=body[keep]
        return body@pose.rotation.T+pose.position,removed


def visible_past_boxes(endpoints_body,sensor_body,boxes):
    """Conservative ray/solid-box occlusion, for optional simulated frame shadowing.
    A bounding box is an envelope, not a measurement of the real frame's bars.
    """
    p=np.asarray(endpoints_body,float);origin=np.asarray(sensor_body,float)
    directions=p-origin;visible=np.ones(len(p),bool)
    for _,lo,hi in boxes:
        enter=np.zeros(len(p));leave=np.ones(len(p));possible=np.ones(len(p),bool)
        for axis in range(3):
            d=directions[:,axis];parallel=np.abs(d)<1e-12
            possible&=~parallel|((origin[axis]>=lo[axis])&(origin[axis]<=hi[axis]))
            divisor=np.where(parallel,1.,d)
            first=(lo[axis]-origin[axis])/divisor;second=(hi[axis]-origin[axis])/divisor
            enter=np.maximum(enter,np.where(parallel,-np.inf,np.minimum(first,second)))
            leave=np.minimum(leave,np.where(parallel,np.inf,np.maximum(first,second)))
        hit=possible&(leave>=enter)&(leave>.01)&(enter<.995)
        visible&=~hit
    return visible

    # Conversion is defined below rather than assuming the sensor IMU is Go2's base.


def body_pose_from_lidar(lidar_pose,cfg,allow_provisional=False):
    from core import Pose
    from scipy.spatial.transform import Rotation
    adapter=CloudAdapter(cfg)
    if not cfg['lidar_extrinsics_calibrated'] and not allow_provisional:
        raise ValueError('lidar_to_body_transform_unmeasured')
    lidar_pose.validate()
    rotation=lidar_pose.rotation@adapter.rotation.T
    position=lidar_pose.position-rotation@adapter.translation
    return Pose(lidar_pose.stamp,position,Rotation.from_matrix(rotation).as_quat(),lidar_pose.frame)
