"""Display-only raw projection math, timing, ROS serialization and supervision."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace as NS,ModuleType
import unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from perception.pipeline import load_settings,Pose
from perception.dense_cloud import DenseProjector,deskew,sensor_transform,xyz_message


class DenseCloudTests(unittest.TestCase):
    def setUp(self):
        self.settings,_,_,_=load_settings(ROOT/'config/lidar_dense_preview.json')
        self.params=dict(extrinsic_T=[.011,.02329,-.04412],extrinsic_R=np.eye(3).ravel().tolist(),extrinsic_est_en=False,scan_line=4)
        self.engine=DenseProjector(self.settings,self.params)
    def pose(self,stamp=100.,x=0.,yaw=0.,frame='camera_init',child='body',wall=None):
        return self.engine.pose_input(stamp,[x,0.,0.],Rotation.from_euler('z',yaw).as_quat(),frame,child,stamp if wall is None else wall,stamp-100.)
    def raw(self,stamp=100.,points=None,offsets=None):
        points=np.array([[1.,0.,0.],[2.,.2,0.],[3.,.4,.5]]) if points is None else np.asarray(points)
        offsets=[0,40000000,80000000] if offsets is None else offsets
        return NS(header=NS(stamp=NS(to_sec=lambda:stamp),frame_id='livox_frame'),timebase=int(stamp*1e9),point_num=len(points),
                  points=[NS(x=p[0],y=p[1],z=p[2],offset_time=o,tag=0,line=0) for p,o in zip(points,offsets)])
    def bracketed(self):
        self.assertTrue(self.pose(99.95,wall=100.12));self.assertTrue(self.pose(100.1,wall=100.12))
        return self.engine.decode(self.raw(),100.12,.12)
    def test_fixed_internal_extrinsics_required(self):
        for change in [dict(extrinsic_est_en=True),dict(extrinsic_T=None),dict(extrinsic_R=[1]*9),dict(extrinsic_R=np.diag([1,1,-1]).ravel().tolist())]:
            p=dict(self.params,**change)
            with self.assertRaises(ValueError):sensor_transform(p)
    def test_static_points_use_internal_lidar_to_imu_once(self):
        frame=self.bracketed();world=self.engine.project(frame,100.12,.12)
        np.testing.assert_allclose(world,frame['points']+self.params['extrinsic_T'])
        self.assertFalse(self.engine.status(.12)['navigation_usable'])
    def test_per_point_translation_keeps_world_wall_fixed(self):
        times=np.linspace(100.,100.1,101);positions=np.c_[times-100.,np.zeros((101,2))]
        world=np.tile([3.,.3,.2],(101,1));local=world-positions-self.params['extrinsic_T']
        poses=[Pose(100.,np.zeros(3),np.array([0,0,0,1.]),'camera_init'),Pose(100.1,np.array([.1,0,0]),np.array([0,0,0,1.]),'camera_init')]
        result=deskew(local,times,poses,*sensor_transform(self.params),.15)
        np.testing.assert_allclose(result,world,atol=1e-10)
    def test_per_point_rotation_slerp_keeps_world_wall_fixed(self):
        times=np.linspace(100.,100.1,101);rotations=Rotation.from_euler('z',(times-100.)/.1*np.pi/2)
        world=np.tile([3.,1.,.2],(101,1));local=rotations.inv().apply(world)-self.params['extrinsic_T']
        poses=[Pose(100.,np.zeros(3),Rotation.from_euler('z',0.).as_quat(),'camera_init'),Pose(100.1,np.zeros(3),Rotation.from_euler('z',np.pi/2).as_quat(),'camera_init')]
        np.testing.assert_allclose(deskew(local,times,poses,*sensor_transform(self.params),.15),world,atol=1e-10)
    def test_antipodal_quaternions_have_same_motion(self):
        poses=[Pose(100.,np.zeros(3),np.array([0,0,0,1.]),'camera_init'),Pose(100.1,np.zeros(3),np.array([0,0,0,-1.]),'camera_init')]
        points=np.ones((3,3));result=deskew(points,np.array([100.,100.05,100.1]),poses,np.zeros(3),np.eye(3),.15)
        np.testing.assert_allclose(result,points)
    def test_no_pose_extrapolation(self):
        self.pose(100.1);frame=self.engine.decode(self.raw(),100.12,.12)
        with self.assertRaisesRegex(ValueError,'brackets'):self.engine.project(frame,100.12,.12)
    def test_excess_pose_gap_is_rejected(self):
        self.pose(99.9,wall=100.12);self.pose(100.1,wall=100.12);frame=self.engine.decode(self.raw(),100.12,.12)
        with self.assertRaisesRegex(ValueError,'gap'):self.engine.project(frame,100.12,.12)
    def test_bad_raw_timebase_frame_and_count_rejected(self):
        for key,value in [('timebase',42),('point_num',99)]:
            self.setUp();msg=self.raw();setattr(msg,key,value)
            with self.assertRaises(ValueError):self.engine.decode(msg,100.12,.12)
        self.setUp();msg=self.raw();msg.header.frame_id='body'
        with self.assertRaisesRegex(ValueError,'frame'):self.engine.decode(msg,100.12,.12)
    def test_invalid_point_times_and_long_scan_rejected(self):
        for offsets in [[0,1,-1],[0,1,300000000]]:
            self.setUp()
            with self.assertRaises(ValueError):self.engine.decode(self.raw(offsets=offsets),100.4,.4)
    def test_duplicate_and_future_scans_rejected(self):
        self.engine.decode(self.raw(),100.12,.12)
        with self.assertRaises(ValueError):self.engine.decode(self.raw(),100.13,.13)
        self.setUp()
        with self.assertRaises(ValueError):self.engine.decode(self.raw(101.),100.12,.12)
    def test_source_age_is_not_refreshed_by_processing(self):
        frame=self.bracketed()
        with self.assertRaisesRegex(ValueError,'expired'):self.engine.project(frame,100.6,.6)
    def test_nan_range_and_livox_flags_filtered(self):
        msg=self.raw(points=[[1,0,0],[2,0,0],[3,0,0],[np.nan,0,0],[.01,0,0],[4,0,0]],offsets=[0,1,2,3,4,5]);msg.points[-1].tag=0x30
        frame=self.engine.decode(msg,100.12,.12)
        self.assertEqual(len(frame['points']),3);self.assertEqual(frame['filtered_points'],3)
    def test_fine_voxel_reduces_duplicates_only_in_display(self):
        self.pose(99.95,wall=100.12);self.pose(100.1,wall=100.12)
        frame=self.engine.decode(self.raw(points=[[1,0,0],[1.001,0,0],[2,0,0]],offsets=[0,1,2]),100.12,.12)
        with self.assertRaisesRegex(ValueError,'voxels'):self.engine.project(frame,100.12,.12)
    def test_pose_frame_jump_and_clock_rollback_latch_reset(self):
        self.pose();self.assertFalse(self.pose(100.1,x=2.));self.assertIn('jump',self.engine.reset_reason)
        self.setUp();self.assertFalse(self.pose(child='other'));self.assertIsNotNone(self.engine.reset_reason)
        self.setUp();self.engine.clock_check(100.);self.engine.clock_check(99.);self.assertIn('rollback',self.engine.reset_reason)
    def test_unrepresentable_world_points_do_not_become_ros_infinities(self):
        self.pose(99.95,x=1e20,wall=100.12);self.pose(100.1,x=1e20,wall=100.12)
        frame=self.engine.decode(self.raw(),100.12,.12)
        with self.assertRaisesRegex(ValueError,'world_points_invalid'):self.engine.project(frame,100.12,.12)

    def test_vectorized_cloud_binary_contract(self):
        class Cloud:pass
        class Field:
            FLOAT32=7
            def __init__(self,**kw):self.__dict__.update(kw)
        modules={'sensor_msgs':ModuleType('sensor_msgs'),'sensor_msgs.msg':ModuleType('sensor_msgs.msg')}
        modules['sensor_msgs.msg'].PointCloud2=Cloud;modules['sensor_msgs.msg'].PointField=Field
        with patch.dict(sys.modules,modules):
            cloud=xyz_message('source_header',[[1.,2.,3.],[-1.,4.,5.]])
            np.testing.assert_allclose(np.frombuffer(cloud.data,dtype='<f4').reshape(-1,3),[[1,2,3],[-1,4,5]])
            self.assertEqual(cloud.header,'source_header');self.assertEqual(cloud.row_step,24)
            self.assertEqual([f.offset for f in cloud.fields],[0,4,8])
            self.assertEqual(xyz_message('old_source',[]).width,0)
    def test_numpy_xyz_decoder_handles_endian_offsets_and_row_padding(self):
        import struct
        from perception.ros_messages import pointcloud_xyz
        fields=[NS(name=n,offset=o,datatype=7,count=1) for n,o in [('x',4),('y',8),('z',16)]]
        data=bytearray(112);expected=[]
        for row in range(2):
            for col in range(2):
                p=[row*2+col+.1,row+1.,col+2.];expected.append(p)
                for offset,value in zip([4,8,16],p):struct.pack_into('>f',data,row*56+col*24+offset,value)
        msg=NS(width=2,height=2,point_step=24,row_step=56,is_bigendian=True,data=bytes(data),fields=fields)
        np.testing.assert_allclose(pointcloud_xyz(msg),expected,rtol=1e-6)

    def test_numpy_xyz_decoder_rejects_truncation_and_field_errors(self):
        from perception.ros_messages import pointcloud_xyz
        msg=NS(width=1,height=1,point_step=12,row_step=12,is_bigendian=False,data=bytes(11),fields=[NS(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))])
        with self.assertRaises(ValueError):pointcloud_xyz(msg)
        msg.data=bytes(12);msg.fields[-1].datatype=3
        with self.assertRaises(ValueError):pointcloud_xyz(msg)

    def test_numpy_xyz_decoder_preserves_nonfinite_for_input_rejection(self):
        from perception.ros_messages import pointcloud_xyz
        xyz=np.array([[1.,np.nan,np.inf]],dtype='<f4')
        msg=NS(width=1,height=1,point_step=12,row_step=12,is_bigendian=False,data=xyz.tobytes(),fields=[NS(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))])
        result=pointcloud_xyz(msg);self.assertTrue(np.isnan(result[0,1]));self.assertTrue(np.isinf(result[0,2]))

    def test_numpy_xyz_decoder_accepts_float64_and_empty_clouds(self):
        from perception.ros_messages import pointcloud_xyz
        xyz=np.array([[1.,2.,3.]],dtype='<f8')
        msg=NS(width=1,height=1,point_step=24,row_step=24,is_bigendian=False,data=xyz.tobytes(),fields=[NS(name=n,offset=i*8,datatype=8,count=1) for i,n in enumerate(('x','y','z'))])
        np.testing.assert_allclose(pointcloud_xyz(msg),[[1.,2.,3.]])
        msg.width=0;msg.data=b'';self.assertEqual(pointcloud_xyz(msg).shape,(0,3))

    def test_supervisor_failure_stops_other_child(self):
        from perception import dense_preview
        made=[]
        class Child:
            def __init__(self,cmd,**kw):
                self.pid=100+len(made);self.returncode=7 if not made else None;self.terminated=False;made.append(self)
            def poll(self):return self.returncode
            def terminate(self):self.terminated=True;self.returncode=-15
            def wait(self,timeout=None):return self.returncode
        with tempfile.TemporaryDirectory() as folder,patch.object(sys,'argv',['dense_preview.py','--settings',str(ROOT/'config/lidar_dense_preview.json'),'--output',str(Path(folder)/'run')]),patch.object(dense_preview.subprocess,'Popen',Child),patch.object(dense_preview.signal,'signal'):
            self.assertEqual(dense_preview.main(),7);self.assertTrue(made[1].terminated)
            report=json.loads((Path(folder)/'run/report.json').read_text());self.assertEqual(report['motion_commands_sent'],0)

    def test_actual_dense_ros_adapter_clears_expired_output(self):
        from perception import dense_node
        modules={};pubs={};start=time.monotonic()
        def module(name,**kw):
            m=ModuleType(name)
            for k,v in kw.items():setattr(m,k,v)
            modules[name]=m;return m
        def publisher(topic,kind,**kw):pubs[topic]=[];return NS(publish=lambda msg:pubs[topic].append(msg))
        def subscribe(topic,kind,callback,**kw):
            if topic=='/aft_mapped_to_init':
                for stamp in [99.95,100.1]:
                    callback(NS(header=NS(stamp=NS(to_sec=lambda stamp=stamp:stamp),frame_id='camera_init'),child_frame_id='body',pose=NS(pose=NS(position=NS(x=0.,y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.)))))
            if topic=='/livox/lidar':
                callback(self.raw());callback(self.raw(100.05))
            return NS()
        param={('/laserMapping/mapping/'+k):v for k,v in self.params.items() if k!='scan_line'};param['/laserMapping/preprocess/scan_line']=4
        module('rospy',Time=NS(now=lambda:NS(to_sec=lambda:100.12+time.monotonic()-start),from_sec=lambda x:x),get_param=lambda key:param[key],myargv=lambda:sys.argv,init_node=lambda name:None,
               Publisher=publisher,Subscriber=subscribe,is_shutdown=lambda:time.monotonic()-start>.7)
        module('livox_ros_driver2');module('livox_ros_driver2.msg',CustomMsg=object)
        module('nav_msgs');module('nav_msgs.msg',Odometry=object)
        module('std_msgs');module('std_msgs.msg',Header=lambda:NS(),String=lambda data:NS(data=data))
        class Cloud:pass
        class Field:
            FLOAT32=7
            def __init__(self,**kw):self.__dict__.update(kw)
        module('sensor_msgs');module('sensor_msgs.msg',PointCloud2=Cloud,PointField=Field)
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder)/'dense';argv=['dense_node.py','--settings',str(ROOT/'config/lidar_dense_preview.json'),'--output',str(output)]
            with patch.dict(sys.modules,modules),patch.object(sys,'argv',argv):self.assertEqual(dense_node.main(),0)
            messages=pubs['/go2/lidar_dense/registered']
            self.assertEqual(set(pubs),{'/go2/lidar_dense/registered','/go2/lidar_dense/status'})
            self.assertEqual(messages[0].width,3);self.assertEqual(messages[-1].width,0)
            self.assertAlmostEqual(messages[-1].header.stamp,100.08)
            report=json.loads((output/'report.json').read_text());self.assertFalse(report['data_valid'])
            self.assertFalse(report['navigation_usable']);self.assertEqual(report['motion_commands_sent'],0)
