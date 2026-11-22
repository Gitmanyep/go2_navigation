"""Rolling return-grid invariants and the actual read-only ROS adapter."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from perception.pipeline import load_settings,expired_snapshot
from perception.reference_preview import ReferencePreviewEngine


class ReferencePreviewTests(unittest.TestCase):
    def setUp(self):
        self.settings,self.cal,self.cfg,_=load_settings(ROOT/'config/lidar_preview.json')
        self.engine=ReferencePreviewEngine(self.settings,self.cal,self.cfg)
        self.points=np.array([[1.,0.,-.7],[1.,0.,.1],[1.,.2,-.6],[1.2,.2,-.6]])
    def pose(self,stamp=100.,mono=0.,x=0.,child='body',frame='camera_init'):
        return self.engine.pose_input(stamp,[x,0.,0.],[0.,0.,0.,1.],frame,child,stamp,mono)
    def cloud(self,stamp=100.,mono=0.,points=None,frame='camera_init',wall=None):
        return self.engine.cloud_input(stamp,self.points if points is None else points,frame,stamp if wall is None else wall,mono)
    def live(self):
        self.assertTrue(self.pose());self.assertTrue(self.cloud());return self.engine.snapshot(100.,0.)
    def test_unmeasured_preview_is_separate_from_formal_terrain_map(self):
        snap=self.live()
        self.assertTrue(snap['valid']);self.assertIsNone(self.engine.map)
        self.assertFalse(self.cal['lidar_extrinsics_calibrated']);self.assertFalse(self.cal['geometry_calibrated'])
        self.assertTrue(snap['audit']['preview_only']);self.assertFalse(snap['audit']['navigation_usable'])
        self.assertFalse(snap['audit']['ground_classification_performed']);self.assertIsNone(snap['body_pose'])
        self.assertTrue((snap['occupancy']==-1).all())
    def test_world_points_remain_unchanged_without_extrinsic(self):
        snap=self.live();np.testing.assert_allclose(snap['obstacle_points'],self.points)
        self.assertEqual(snap['reference_pose']['position'],[0.,0.,0.])
    def test_minimum_and_maximum_returns_do_not_claim_ground(self):
        snap=self.live();self.assertAlmostEqual(snap['elevation_points'][:,2].min(),-.7)
        self.assertAlmostEqual(snap['maximum_points'][:,2].max(),.1)
        self.assertGreater(snap['audit']['unknown_cells'],0);self.assertLess(snap['audit']['observed_cells'],len(self.points))
    def test_rolling_window_preserves_static_obstacle_world_location(self):
        first=self.live()
        for i in range(1,5):
            self.assertTrue(self.pose(100.+i*.1,mono=i*.1,x=i*.5));self.assertTrue(self.cloud(100.+i*.1,mono=i*.1))
        last=self.engine.snapshot(100.4,.4)
        self.assertGreater(last['origin'][0],first['origin'][0]+1.8)
        self.assertTrue(np.all(np.isin(last['obstacle_points'][:,0],[1.,1.2])))
        self.assertGreater(last['audit']['unknown_cells'],2400)
    def test_returns_outside_current_window_are_clipped(self):
        self.live()
        for i in range(1,7):
            self.pose(100.+i*.1,mono=i*.1,x=i*.75)
            self.cloud(100.+i*.1,mono=i*.1,points=self.points+[4.5,0.,0.])
        snap=self.engine.snapshot(100.6,.6)
        self.assertTrue((snap['obstacle_points'][:,0]>=snap['origin'][0]).all())
        self.assertFalse(np.any(np.isclose(snap['obstacle_points'][:,0],1.)))
    def test_old_evidence_expires_even_with_new_source_data(self):
        self.live();self.pose(103.1,mono=3.1)
        self.cloud(103.1,mono=3.1,points=self.points+[1.,0.,0.])
        snap=self.engine.snapshot(103.1,3.1)
        self.assertTrue(snap['valid']);self.assertFalse(np.any(np.isclose(snap['obstacle_points'][:,0],1.)))
    def test_stale_input_clears_points_and_preserves_source_time(self):
        self.live();snap=self.engine.snapshot(100.6,.6)
        self.assertFalse(snap['valid']);self.assertEqual(len(snap['obstacle_points']),0)
        self.assertEqual(snap['source_stamp'],100.);self.assertIsNone(snap['reference_pose'])
        self.assertEqual(snap['audit']['observed_cells'],0)
    def test_publisher_watchdog_invalidates_stalled_worker(self):
        snap=expired_snapshot(self.live(),100.31,.31,self.settings)
        self.assertFalse(snap['valid']);self.assertEqual(len(snap['elevation_points']),0)
        self.assertEqual(snap['source_stamp'],100.)
    def test_duplicate_and_future_data_cannot_refresh_map(self):
        self.live();self.assertFalse(self.cloud());self.assertFalse(self.engine.snapshot(100.01,.01)['valid'])
        self.assertFalse(self.cloud(101.,mono=.1,wall=100.1))
    def test_frame_and_pose_skew_are_enforced(self):
        self.pose();self.assertFalse(self.cloud(frame='other'))
        self.assertFalse(self.cloud(100.2,mono=.2));self.assertIn('skew',self.engine.last_error)
    def test_nonfinite_points_dropped_and_bad_cloud_blocked(self):
        self.pose();self.assertTrue(self.cloud(points=np.r_[self.points,[[np.nan,0.,0.]]]))
        self.assertEqual(self.engine.audit.topics['cloud']['nonfinite_points_dropped'],1)
        self.assertFalse(self.cloud(100.01,mono=.01,points=[[np.nan,0.,0.]]*3))
        self.assertFalse(self.engine.snapshot(100.01,.01)['valid'])
    def test_reference_change_requires_restart(self):
        self.live();self.assertFalse(self.pose(100.1,mono=.1,child='other'))
        snap=self.engine.snapshot(100.1,.1);self.assertFalse(snap['valid']);self.assertEqual(len(self.engine.frames),0)
    def test_odometry_jump_and_clock_rollback_invalidate(self):
        self.live();self.assertFalse(self.pose(100.1,mono=.1,x=2.))
        self.assertFalse(self.engine.snapshot(100.1,.1)['valid'])
        self.setUp();self.live();snap=self.engine.snapshot(99.,.1)
        self.assertFalse(snap['valid']);self.assertIn('rollback',snap['reason'])
    def test_memory_has_strict_point_limit(self):
        self.settings['reference_preview']['max_retained_points']=100
        self.engine=ReferencePreviewEngine(self.settings,self.cal,self.cfg)
        points=np.tile(self.points,(100,1))
        for i in range(5):self.pose(100+i*.1,mono=i*.1);self.cloud(100+i*.1,mono=i*.1,points=points)
        self.assertLessEqual(sum(len(p) for _,p in self.engine.frames),100)
        self.assertLessEqual(len(self.engine.snapshot(100.4,.4)['obstacle_points']),100)
    def test_invalid_preview_dimensions_rejected(self):
        for key,value in [('window_m',float('nan')),('resolution_m',0.),('window_m',10000.),('max_retained_points',1),('retention_s',-1.)]:
            settings=copy.deepcopy(self.settings);settings['reference_preview'][key]=value
            with self.assertRaises(ValueError):ReferencePreviewEngine(settings,self.cal,self.cfg)
    def test_launcher_cannot_sneak_preview_flag_into_mapping(self):
        with tempfile.TemporaryDirectory() as folder:
            p=subprocess.run([str(ROOT/'bin/go2-nav'),'mapping','--reference-preview','--log-root',folder],capture_output=True,text=True)
            self.assertEqual(p.returncode,2);self.assertEqual(list(Path(folder).iterdir()),[])

    def test_actual_ros_adapter_topics_live_and_stale_clear(self):
        from perception import ros_node
        from perception.reference_preview import preview_messages
        modules={};published={};start=time.monotonic()
        def module(name,**kw):
            m=ModuleType(name)
            for k,v in kw.items():setattr(m,k,v)
            modules[name]=m;return m
        def pose():return NS(position=NS(x=0.,y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))
        class Marker:
            LINE_STRIP=4;TEXT_VIEW_FACING=9;CUBE_LIST=6;ARROW=0;ADD=0;DELETE=2
            def __init__(self):self.pose=pose();self.scale=NS();self.color=NS();self.header=NS()
        def pub(topic,kind,**kw):
            published[topic]=[];return NS(publish=lambda msg:published[topic].append(msg))
        header=NS(stamp=NS(to_sec=lambda:100.),frame_id='camera_init')
        odom=NS(header=header,child_frame_id='body',pose=NS(pose=pose()))
        cloud=NS(header=header,width=len(self.points),height=1,points=self.points.tolist())
        def sub(topic,kind,callback,**kw):
            if topic=='/aft_mapped_to_init':callback(odom)
            if topic=='/cloud_registered':callback(cloud)
            return NS()
        module('rospy',Time=NS(now=lambda:NS(to_sec=lambda:100.+time.monotonic()-start),from_sec=lambda x:x),
               myargv=lambda:sys.argv,init_node=lambda name:None,Publisher=pub,Subscriber=sub,is_shutdown=lambda:False)
        module('std_msgs');module('std_msgs.msg',Header=lambda:NS(),String=lambda data:NS(data=data))
        module('nav_msgs');module('nav_msgs.msg',Odometry=object,OccupancyGrid=object)
        module('geometry_msgs');module('geometry_msgs.msg',PoseStamped=object,Point=lambda **kw:NS(**kw))
        class BinaryCloud:
            def __setattr__(self,key,value):
                object.__setattr__(self,key,value)
                if key=='data':object.__setattr__(self,'points',np.frombuffer(value,dtype='<f4').reshape(-1,3).tolist())
        class Field:
            FLOAT32=7
            def __init__(self,**kw):self.__dict__.update(kw)
        sensor=module('sensor_msgs');module('sensor_msgs.msg',PointCloud2=BinaryCloud,PointField=Field,Imu=object)
        sensor.point_cloud2=NS(read_points=lambda msg,**kw:iter(msg.points),create_cloud_xyz32=lambda h,p:NS(header=h,points=p))
        module('tf2_msgs');module('tf2_msgs.msg',TFMessage=object)
        module('visualization_msgs');module('visualization_msgs.msg',Marker=Marker)
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder)/'run'
            argv=['ros_node.py','--settings',str(ROOT/'config/lidar_preview.json'),'--reference-preview','--output',str(output),'--seconds','.65']
            with patch.dict(sys.modules,modules),patch.object(sys,'argv',argv):
                self.assertEqual(ros_node.main(),0)
                expired=expired_snapshot(self.live(),100.31,.31,self.settings)
                msgs=preview_messages(expired)
                self.assertEqual(msgs['points'].points,[]);self.assertEqual(msgs['highest_returns'].points,[])
                self.assertEqual(msgs['reference_pose'].action,Marker.DELETE)
            self.assertEqual(set(published),{'/go2/lidar_preview/'+s for s in ('points','lowest_returns','highest_returns','window','warning','observed_cells','reference_pose','status')})
            self.assertGreater(len(published['/go2/lidar_preview/points'][0].points),0)
            self.assertEqual(published['/go2/lidar_preview/points'][-1].points,[])
            self.assertEqual(published['/go2/lidar_preview/points'][-1].header.stamp,100.)
            self.assertEqual(published['/go2/lidar_preview/reference_pose'][-1].action,Marker.DELETE)
            self.assertIn('STALE',published['/go2/lidar_preview/warning'][-1].text)
            report=json.loads((output/'report.json').read_text());last=report['last_status']
            self.assertFalse(last['map_data_valid']);self.assertEqual(last['displayed_points'],0)
            self.assertFalse(last['navigation_usable']);self.assertEqual(last['motion_commands_sent'],0)
            self.assertFalse(report['worker_still_running'])
