"""Real-data mapping contracts using synthetic data and fake ROS messages only."""
import ast
import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace,ModuleType
import unittest
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from perception.pipeline import MappingEngine,ReceiveMeter,load_settings,expired_snapshot,interpolate_pose
from perception.ros_messages import messages
from core import Pose


class PerceptionTests(unittest.TestCase):
    def setUp(self):
        self.settings,self.cal,self.cfg,_=load_settings(ROOT/'config/perception.json')
        self.settings=copy.deepcopy(self.settings);self.cal=copy.deepcopy(self.cal);self.cfg=copy.deepcopy(self.cfg)
        self.cal.update(lidar_extrinsics_calibrated=True,geometry_calibrated=True,measurement_record='SYNTHETIC',
            reference_frame='lidar_imu',lidar_position_body_m=[.2,0.,.15],lidar_rotation_body=np.eye(3).tolist(),
            body_reference_height_m=.3,geometry_boxes=self.cfg['geometry_boxes'])
        for key in ('lidar_extrinsics_calibrated','geometry_calibrated','lidar_position_body_m','lidar_rotation_body','body_reference_height_m'):self.cfg[key]=self.cal[key]
        self.engine=MappingEngine(self.settings,self.cal,self.cfg)
        yy,xx=np.mgrid[-1.8:1.81:.15,-1.8:1.81:.15]
        self.floor=np.column_stack([xx.ravel()+.02,yy.ravel()+.02,np.zeros(xx.size)])
    def pose(self,stamp=100.,x=0.,mono=0.,frame='camera_init',child='lidar_imu'):
        return self.engine.pose_input(stamp,[x+.2,0.,.45],[0.,0.,0.,1.],frame,child,stamp,mono)
    def cloud(self,stamp=100.,mono=0.,points=None,frame='camera_init'):
        return self.engine.cloud_input(stamp,self.floor if points is None else points,frame,stamp,mono)
    def mapped(self):
        for i in range(4):self.assertTrue(self.pose(100+i*.1,mono=i*.1));self.assertTrue(self.cloud(100+i*.1,mono=i*.1))
        return self.engine.snapshot(100.3,.3)

    def test_default_unmeasured_template_blocks_map_but_preserves_audit(self):
        settings,cal,cfg,_=load_settings(ROOT/'config/perception.json');self.engine=MappingEngine(settings,cal,cfg)
        self.pose();self.assertFalse(self.cloud());s=self.engine.snapshot(100,0.)
        self.assertFalse(s['valid']);self.assertTrue((s['occupancy']==-1).all())
        self.assertIn('extrinsics_unmeasured',s['audit']['calibration_issues'])
        self.assertEqual(s['audit']['topics']['cloud']['processed_count'],1)

    def test_registered_cloud_not_transformed_twice_and_pose_offset_is_applied(self):
        s=self.mapped();self.assertTrue(s['valid'])
        self.assertAlmostEqual(s['body_pose']['position'][0],0.)
        self.assertAlmostEqual(s['body_pose']['position'][2],.3)
        self.assertTrue(np.allclose(s['elevation_points'][:,2],0.))
        self.assertTrue((s['occupancy']<0).any())

    def test_pairing_interpolates_translation_and_quaternion(self):
        poses=[Pose(100.,np.array([0.,0.,.3]),np.array([0.,0.,0.,1.])),
               Pose(100.1,np.array([1.,0.,.3]),np.array([0.,0.,1.,0.]))]
        pose,skew,mode=interpolate_pose(poses,100.05,.06)
        self.assertEqual(mode,'interpolated');self.assertAlmostEqual(pose.position[0],.5)
        self.assertAlmostEqual(pose.yaw,math.pi/2);self.assertAlmostEqual(skew,.05)

    def test_pairing_rejects_excess_skew(self):
        self.pose();self.assertFalse(self.cloud(100.2,mono=.2))
        self.assertIn('skew',self.engine.last_error)

    def test_world_frame_mismatch_blocks_cloud(self):
        self.pose();self.assertFalse(self.cloud(frame='other'))
        self.assertFalse(self.engine.snapshot(100.,0.)['valid'])

    def test_odom_child_reference_must_match_declared_calibration(self):
        self.assertFalse(self.pose(child='some_other_body'))
        self.assertFalse(self.cloud())

    def test_pose_jump_latches_reset_and_does_not_shift_old_map(self):
        self.mapped();self.assertFalse(self.pose(100.4,x=2.,mono=.4))
        self.assertIsNotNone(self.engine.reset_required);self.assertIsNone(self.engine.map)
        self.assertFalse(self.cloud(100.4,mono=.4))

    def test_receipt_order_between_topics_is_not_a_ros_clock_rollback(self):
        self.pose(100.1,mono=.1);self.cloud(100.08,mono=.08)
        self.assertIsNone(self.engine.reset_required)

    def test_actual_ros_clock_rollback_invalidates_map(self):
        self.mapped();s=self.engine.snapshot(99.,.4)
        self.assertFalse(s['valid']);self.assertIn('rollback',s['reason']);self.assertIsNone(self.engine.map)

    def test_duplicate_and_future_clouds_do_not_renew_map_lease(self):
        self.mapped();self.assertFalse(self.cloud(100.3,mono=.31))
        self.assertFalse(self.engine.snapshot(100.31,.31)['valid'])
        self.assertFalse(self.engine.cloud_input(101.,self.floor,'camera_init',100.4,.4))

    def test_nonfinite_points_are_counted_and_not_published(self):
        self.pose();points=np.r_[self.floor,[[float('nan'),0.,0.]]]
        self.assertTrue(self.cloud(points=points))
        self.assertEqual(self.engine.audit.report(0.)['topics']['cloud']['nonfinite_points_dropped'],1)

    def test_invalid_pose_and_oversized_cloud_block_mapping(self):
        self.assertFalse(self.engine.pose_input(100.,[0,0,.3],[0,0,0,0],'camera_init','lidar_imu',100.,0.))
        self.pose(100.1,mono=.1);self.settings['max_cloud_points']=3
        self.assertFalse(self.cloud(100.1,mono=.1))

    def test_stale_worker_output_is_unknown_and_keeps_original_source_stamp(self):
        s=self.mapped();expired=expired_snapshot(s,100.7,.7,self.settings)
        self.assertFalse(expired['valid']);self.assertTrue((expired['occupancy']==-1).all())
        self.assertEqual(len(expired['elevation_points']),0);self.assertEqual(len(expired['obstacle_points']),0)
        self.assertEqual(expired['source_stamp'],s['source_stamp'])
        self.assertTrue((s['occupancy']>=0).any())

    def test_3d_obstacles_keep_height_and_expire(self):
        wall=np.array([[.9,y,z] for y in np.arange(-.3,.31,.1) for z in np.arange(.3,.81,.1)])
        for i in range(4):self.pose(100+i*.1,mono=i*.1);self.cloud(100+i*.1,mono=i*.1,points=np.r_[self.floor,wall])
        s=self.engine.snapshot(100.3,.3)
        self.assertTrue(len(s['obstacle_points'])>0)
        self.assertGreater(s['obstacle_points'][:,2].max(),.6)
        self.assertEqual(len(self.engine.snapshot(104.,4.)['obstacle_points']),0)

    def test_audit_only_never_constructs_map(self):
        self.engine=MappingEngine(self.settings,self.cal,self.cfg,audit_only=True)
        self.pose();self.cloud();self.assertIsNone(self.engine.map)
        self.assertEqual(self.engine.audit.topics['cloud']['rejected'],0)

    def test_solid_self_filter_does_not_use_collision_envelope(self):
        self.cal.update(self_filter_enabled=True,self_filter_boxes=[dict(minimum=[-.1,-.1,-.1],maximum=[.1,.1,.1])])
        self.engine=MappingEngine(self.settings,self.cal,self.cfg);self.pose()
        points=np.r_[self.floor,[[0.,0.,.3],[.4,0.,.6]]]
        self.assertTrue(self.cloud(points=points))
        merged=self.engine.map.frames[-1][1]
        self.assertFalse(np.any(np.all(np.isclose(merged,[0.,0.,.3]),axis=1)))
        self.assertTrue(np.any(np.all(np.isclose(merged,[.4,0.,.6]),axis=1)))

    def test_direction_contract_and_replay_checked_without_affecting_map(self):
        msg=dict(stamp_sec=100.,frame_id='body',kind='direction',valid=True,vector=[1.,0.,0.],confidence=.9,seq=1,source_id='bird',session_id='a')
        self.assertTrue(self.engine.direction_input(msg,100.,0.))
        msg.update(stamp_sec=100.1)
        self.assertFalse(self.engine.direction_input(msg,100.1,.1))
        self.pose(100.1,mono=.1);self.assertTrue(self.cloud(100.1,mono=.1))

    def test_microphone_frame_requires_measured_rotation(self):
        msg=dict(stamp_sec=100.,frame_id='mic_array',kind='direction',valid=True,vector=[1.,0.,0.],confidence=.9,seq=1,source_id='bird',session_id='a')
        self.assertFalse(self.engine.direction_input(msg,100.,0.))
        self.assertIn('microphone',self.engine.audit.topics['direction']['reason'])

    def test_callback_frequency_meter_counts_before_queue_drop(self):
        meter=ReceiveMeter()
        for i in range(21):meter.observe('direction',100+i*.02,i*.02)
        self.assertAlmostEqual(meter.report()['direction']['receive_hz'],50.)
        self.assertEqual(meter.report()['direction']['callback_count'],21)

    def test_fake_ros_messages_keep_source_stamps_and_unknown_cells(self):
        def pose_message():return SimpleNamespace(header=SimpleNamespace(),pose=SimpleNamespace(position=SimpleNamespace(),orientation=SimpleNamespace()))
        modules={}
        def module(name,**attributes):
            m=ModuleType(name)
            for k,v in attributes.items():setattr(m,k,v)
            modules[name]=m
            return m
        module('rospy',Time=SimpleNamespace(from_sec=lambda x:x))
        module('std_msgs');module('std_msgs.msg',Header=lambda:SimpleNamespace())
        module('nav_msgs');module('nav_msgs.msg',OccupancyGrid=lambda:SimpleNamespace(info=SimpleNamespace(origin=SimpleNamespace(position=SimpleNamespace(),orientation=SimpleNamespace()))))
        module('geometry_msgs');module('geometry_msgs.msg',PoseStamped=pose_message)
        sensor=module('sensor_msgs');sensor.point_cloud2=SimpleNamespace(create_cloud_xyz32=lambda h,p:dict(header=h,points=p))
        s=self.mapped()
        with patch.dict(sys.modules,modules):
            outputs=messages(s)
            self.assertEqual(outputs['map'].header.stamp,100.3)
            self.assertIn(-1,outputs['map'].data)
            self.assertEqual(outputs['elevation']['header'].stamp,100.3)
            expired=messages(expired_snapshot(s,101.,1.,self.settings))
            self.assertEqual(expired['obstacles']['points'],[])
            self.assertNotIn('body_pose',expired)

    def test_real_adapter_has_no_actuation_import_or_publisher(self):
        for source in (ROOT/'perception').glob('*.py'):
            tree=ast.parse(source.read_text())
            for node in ast.walk(tree):
                if isinstance(node,ast.Import):names=[n.name for n in node.names]
                elif isinstance(node,ast.ImportFrom):names=[node.module or '']
                else:names=[]
                self.assertFalse(any(n.startswith(('unitree_sdk','cyclonedds')) for n in names))
            self.assertNotIn('/cmd_vel',source.read_text())

    def test_invalid_map_status_keeps_calibration_reason_on_publication(self):
        settings,cal,cfg,_=load_settings(ROOT/'config/perception.json')
        engine=MappingEngine(settings,cal,cfg)
        engine.pose_input(100.,[0.,0.,.45],[0.,0.,0.,1.],'camera_init','lidar_imu',100.,0.)
        engine.cloud_input(100.,self.floor,'camera_init',100.,0.)
        result=expired_snapshot(engine.snapshot(100.,0.),100.,0.,settings)
        self.assertIn('calibration_blocked',result['reason'])

    def test_ros_adapter_runs_with_fake_transport_and_only_publishes_maps(self):
        import time
        from perception import ros_node
        def pose_message():return SimpleNamespace(header=SimpleNamespace(),pose=SimpleNamespace(position=SimpleNamespace(),orientation=SimpleNamespace()))
        modules={}
        def module(name,**attributes):
            m=ModuleType(name)
            for k,v in attributes.items():setattr(m,k,v)
            modules[name]=m
            return m
        start=time.monotonic();published={}
        def publish_factory(topic,kind,**kwargs):
            published[topic]=[]
            return SimpleNamespace(publish=lambda msg:published[topic].append(msg))
        header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda:100.),frame_id='camera_init')
        odom=SimpleNamespace(header=header,child_frame_id='lidar_imu',pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=.2,y=0.,z=.45),orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))))
        cloud=SimpleNamespace(header=header,width=len(self.floor),height=1,points=self.floor.tolist())
        def subscribe(topic,kind,callback,**kwargs):
            if topic=='/aft_mapped_to_init':callback(odom)
            if topic=='/cloud_registered':callback(cloud)
            return SimpleNamespace()
        module('rospy',Time=SimpleNamespace(now=lambda:SimpleNamespace(to_sec=lambda:100.+time.monotonic()-start),from_sec=lambda x:x),
               myargv=lambda:sys.argv,init_node=lambda name:None,Publisher=publish_factory,Subscriber=subscribe,is_shutdown=lambda:False)
        module('std_msgs');module('std_msgs.msg',Header=lambda:SimpleNamespace(),String=lambda data:SimpleNamespace(data=data))
        module('nav_msgs');module('nav_msgs.msg',Odometry=object,OccupancyGrid=lambda:SimpleNamespace(info=SimpleNamespace(origin=SimpleNamespace(position=SimpleNamespace(),orientation=SimpleNamespace()))))
        module('geometry_msgs');module('geometry_msgs.msg',PoseStamped=pose_message)
        sensor=module('sensor_msgs');module('sensor_msgs.msg',PointCloud2=object,Imu=object)
        sensor.point_cloud2=SimpleNamespace(read_points=lambda m,**kwargs:iter(m.points),create_cloud_xyz32=lambda h,p:dict(header=h,points=p))
        module('tf2_msgs');module('tf2_msgs.msg',TFMessage=object)
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder);cal_path=folder/'calibration.json';cal_path.write_text(json.dumps(self.cal))
            argv=['ros_node.py','--settings',str(ROOT/'config/perception.json'),'--calibration',str(cal_path),'--output',str(folder/'output'),'--seconds','.6']
            with patch.dict(sys.modules,modules),patch.object(sys,'argv',argv):self.assertEqual(ros_node.main(),0)
            report=json.loads((folder/'output/report.json').read_text())
            self.assertEqual(report['last_status']['motion_commands_sent'],0)
            self.assertGreater(len(published['/go2/perception/map']),0)
            self.assertEqual(set(published),{'/go2/perception/map','/go2/perception/elevation','/go2/perception/obstacles','/go2/perception/body_pose','/go2/perception/status'})
            self.assertEqual(published['/go2/perception/map'][0].header.stamp,100.)
            self.assertFalse(report['worker_still_running'])
            self.assertFalse(report['last_status']['map_data_valid'])
            self.assertGreaterEqual(len(published['/go2/perception/map']),2)
            self.assertEqual(published['/go2/perception/map'][-1].header.stamp,100.)
            self.assertEqual(report['hardware_acceptance'],'pending')

    def test_declared_calibration_with_reflected_rotation_remains_blocked(self):
        self.cal['lidar_rotation_body']=np.diag([1.,1.,-1.]).tolist()
        self.engine=MappingEngine(self.settings,self.cal,self.cfg)
        self.assertIn('invalid_extrinsics',self.engine.calibration_issues)
        self.pose();self.assertFalse(self.cloud())

    def test_missing_noetic_live_entry_fails_without_enabling_motion(self):
        if Path('/opt/ros/noetic/setup.bash').exists():self.skipTest('Noetic is available')
        with tempfile.TemporaryDirectory() as folder:
            result=subprocess.run([str(ROOT/'bin/go2-nav'),'data-audit','--seconds','.1','--log-root',folder],capture_output=True,text=True)
            self.assertEqual(result.returncode,2)
            meta=json.loads((next(Path(folder).iterdir())/'metadata.json').read_text())
            self.assertFalse(meta['robot_motion_backend_enabled'])
            self.assertIn('Noetic',meta['error'])

    def test_demo_launcher_is_synthetic_and_rolls_map(self):
        with tempfile.TemporaryDirectory() as folder:
            result=subprocess.run([str(ROOT/'bin/go2-nav'),'perception-demo','--log-root',folder],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            run=next(Path(folder).iterdir());report=json.loads((run/'perception/report.json').read_text())
            self.assertTrue(report['passed']);self.assertEqual(report['source'],'synthetic')
            self.assertTrue(report['static_obstacle_world_x_correct']);self.assertGreater(report['rolling_origin'][0],-5.1)
            self.assertFalse(json.loads((run/'metadata.json').read_text())['robot_motion_backend_enabled'])
