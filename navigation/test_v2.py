"""Regression checks from the perception / acoustic-task review."""
import copy
import math
import unittest
import numpy as np
from acoustic_task import AcousticTask
from core import config,Pose,DirectionManager,Navigator,TerrainMap
from geometry import Geometry,CloudAdapter
from heading_planner import heading_astar
from simulation import World,Simulation,quaternion


class ReviewRegressions(unittest.TestCase):
    def setUp(self):
        self.cfg=config()
        self.pose=Pose(100.,np.array([0.,0.,.3]),np.array([0.,0.,0.,1.]))

    def msg(self,seq,stamp,angle=0.,**extra):
        return dict(seq=seq,stamp_sec=stamp,vector=[math.cos(angle),math.sin(angle),.2],
                    frame_id='body',kind='direction',confidence=.9,valid=True,**extra)

    def floor(self):
        yy,xx=np.mgrid[-1.8:1.81:.15,-1.8:1.81:.15]
        return np.column_stack([xx.ravel()+.02,yy.ravel()+.02,np.zeros(xx.size)])

    def test_one_point_per_cell_per_frame_fuses(self):
        m=TerrainMap(self.cfg,[0,0]);points=self.floor()
        for i in range(10):m.integrate(points,self.pose.position,.1*i)
        terrain=m.analyze(.9)
        self.assertTrue(terrain['support_valid']);self.assertGreater((terrain['grid']==0).sum(),300)
        self.assertFalse(terrain['blocked'][m.cell([0,0])[1],m.cell([0,0])[0]])

    def test_fusion_does_not_keep_expired_evidence_alive(self):
        m=TerrainMap(self.cfg,[0,0]);p=self.floor()
        for i in range(3):m.integrate(p,self.pose.position,i*.1)
        self.assertTrue(m.analyze(2.)['blocked'].all())
        with self.assertRaises(ValueError):m.integrate(p,self.pose.position,.2)

    def test_sparse_old_frames_cannot_supply_current_support(self):
        m=TerrainMap(self.cfg,[0,0]);p=self.floor()
        m.integrate(p,self.pose.position,0)
        m.integrate(p,self.pose.position,2.)
        self.assertTrue(m.analyze(2.)['blocked'].all())

    def test_elevated_plane_is_not_ground(self):
        m=TerrainMap(self.cfg,[0,0]);p=self.floor();p[:,2]+=2.
        for i in range(4):m.integrate(p,self.pose.position,i*.1)
        self.assertFalse(m.support_valid);self.assertTrue(m.analyze(.3)['blocked'].all())

    def test_table_without_ground_below_is_not_support(self):
        m=TerrainMap(self.cfg,[0,0]);p=self.floor();p[:,2]+=.8
        for i in range(4):m.integrate(p,self.pose.position,i*.1)
        self.assertFalse(m.support_valid)

    def test_ground_and_high_overhead_are_separate(self):
        m=TerrainMap(self.cfg,[0,0]);p=self.floor();roof=p.copy();roof[:,2]=1.4
        for i in range(3):m.integrate(np.r_[p,roof],self.pose.position,.1*i)
        t=m.analyze(.2);x,y=m.cell([0,0])
        self.assertEqual(t['grid'][y,x],0)
        self.assertFalse(t['heading_masks'][0][y,x])

    def test_blind_zone_stays_unknown(self):
        w=World('blind_zone',self.cfg);m=TerrainMap(self.cfg,[0,0])
        for i in range(5):m.integrate(w.cloud([0,0],np.random.RandomState(i),frame_index=i+1),self.pose.position,.1*i)
        x,y=m.cell([-.15,0]);self.assertEqual(m.analyze(.4)['grid'][y,x],-1)
        self.assertTrue(m.analyze(.4)['blocked'][y,x])

    def test_filter_response_independent_of_message_rate(self):
        values=[]
        for hz in [10,50]:
            m=DirectionManager(self.cfg);self.assertTrue(m.accept(self.msg(0,100),self.pose,100,0))
            for i in range(1,int(.3*hz)+1):
                stamp=100+i/hz;pose=copy.deepcopy(self.pose);pose.stamp=stamp
                self.assertTrue(m.accept(self.msg(i,stamp,math.pi/4),pose,stamp,i/hz))
            values.append(m.heading)
        self.assertAlmostEqual(values[0],values[1],places=10)
        self.assertAlmostEqual(math.degrees(values[0]),45*(1-math.exp(-.3/.5)),places=8)

    def test_reconnect_requires_new_session_and_expired_old_session(self):
        m=DirectionManager(self.cfg)
        self.assertTrue(m.accept(self.msg(50,100,session_id='a'),self.pose,100,0))
        p=copy.deepcopy(self.pose);p.stamp=100.1
        self.assertFalse(m.accept(self.msg(0,100.1,session_id='b'),p,100.1,.1))
        # The rejected session switch never permits replay of the old source time.
        p.stamp=101.
        self.assertTrue(m.accept(self.msg(0,101.,session_id='b'),p,101.,1.))
        p.stamp=101.1
        self.assertFalse(m.accept(self.msg(51,101.1,session_id='a'),p,101.1,1.1))

    def test_source_switch_requires_listening(self):
        n=Navigator(self.cfg);n.pose_input(self.pose,100,0)
        self.assertTrue(n.direction_input(self.msg(1,100,source_id='bird-a'),100,0))
        n.task.state='APPROACH'
        p=copy.deepcopy(self.pose);p.stamp=100.1;n.pose_input(p,100.1,.1)
        self.assertTrue(n.direction_input(self.msg(2,100.1,source_id='bird-b'),100.1,.1))
        self.assertEqual(n.task.state,'STOP_LISTEN')

    def test_direction_packet_cannot_confirm_completion(self):
        sim=Simulation('gentle_slope',self.cfg)
        for _ in range(25):sim.step()
        p=sim.nav.pose
        msg=self.msg(1000,p.stamp,confirmed=True,target_confirmed=True)
        sim.nav.direction_input(msg,p.stamp,sim.t)
        self.assertFalse(sim.nav.task.confirmed)
        self.assertNotEqual(sim.nav.task.state,'COMPLETE')

    def test_human_confirm_requires_fresh_task_id_and_sequence(self):
        task=AcousticTask(self.cfg)
        base=dict(task_id=task.task_id,seq=1,stamp_sec=100.,action='confirm_found',confirmed=True)
        for change in [dict(task_id='old-task'),dict(stamp_sec=99.),dict(seq=True),dict(confirmed='true')]:
            msg=dict(base,**change);self.assertFalse(task.confirm(msg,100,0))
        self.assertTrue(task.confirm(base,100,0));self.assertFalse(task.confirm(base,100,0))

    def test_completion_waits_for_observed_stop(self):
        task=AcousticTask(self.cfg)
        self.assertTrue(task.confirm(dict(task_id=task.task_id,seq=1,stamp_sec=100.,action='confirm_found',confirmed=True),100,0))
        for i in range(10):
            task.observe_pose(Pose(100+i*.1,np.array([i*.01,0,.3]),self.pose.quaternion))
            self.assertFalse(task.tick(i*.1,True))
        self.assertEqual(task.state,'VERIFY_STOP')
        for i in range(15):
            task.observe_pose(Pose(101+i*.1,np.array([.1,0,.3]),self.pose.quaternion))
            task.tick(1+i*.1,True)
        self.assertEqual(task.state,'COMPLETE')

    def test_stale_odometry_cannot_confirm_stopped(self):
        task=AcousticTask(self.cfg);task.linear_speed=task.yaw_speed=0.
        task.confirm(dict(task_id=task.task_id,seq=1,stamp_sec=100.,action='confirm_found',confirmed=True),100,0)
        for i in range(20):task.tick(i*.1,True,False)
        self.assertNotEqual(task.state,'COMPLETE')

    def test_lost_direction_does_not_trigger_blind_motion(self):
        task=AcousticTask(self.cfg);task.state='APPROACH'
        self.assertFalse(task.tick(1,False));self.assertEqual(task.state,'LOST')
        self.assertFalse(task.tick(10,False));self.assertEqual(task.state,'SEARCH')
        self.assertFalse(task.tick(11,True));self.assertEqual(task.state,'STOP_LISTEN')

    def test_fixed_3d_source_and_back_mount(self):
        sim=Simulation('gentle_slope',self.cfg)
        source=sim.world.source_position.copy()
        for _ in range(5):sim.step()
        np.testing.assert_array_equal(sim.world.source_position,source)
        self.assertLess(self.cfg['lidar_position_body_m'][0],0)
        self.assertGreater(self.cfg['lidar_position_body_m'][2],0)
        self.assertFalse(self.cfg['lidar_extrinsics_calibrated'])

    def test_head_frame_front_overhang(self):
        g=Geometry(self.cfg);point=np.array([[.6,0,.8]])
        self.assertEqual(g.collision(point,self.pose.position,self.pose.rotation),'head_frame')
        self.assertIsNone(g.collision(point,self.pose.position,Pose(100,self.pose.position,quaternion(0,0,math.pi/2)).rotation))

    def test_rotation_can_hit_between_clear_endpoints(self):
        g=Geometry(self.cfg);pole=np.array([[.4,.4,.8]])
        hits=[]
        for a in [0,math.pi/4,math.pi/2]:
            r=Pose(100,self.pose.position,quaternion(0,0,a)).rotation
            hits.append(g.collision(pole,self.pose.position,r))
        self.assertEqual(hits,[None,'head_frame',None])

    def test_raw_cloud_transform_requires_calibration(self):
        adapter=CloudAdapter(self.cfg)
        with self.assertRaises(ValueError):adapter.transform(np.array([[1.,0,0]]),self.pose)
        cfg=dict(self.cfg,lidar_extrinsics_calibrated=True)
        out,removed=CloudAdapter(cfg).transform(np.array([[1.,0,0]]),self.pose)
        np.testing.assert_allclose(out,[[.85,0,.45]])
        self.assertEqual(removed,0)

    def test_self_filter_never_enabled_by_unmeasured_geometry(self):
        cfg=dict(self.cfg,lidar_extrinsics_calibrated=True,self_filter_enabled=True)
        with self.assertRaises(ValueError):CloudAdapter(cfg).transform(np.array([[0,0,0.]]),self.pose)

    def test_heading_planner_checks_mid_rotation(self):
        shape=(7,7);zero=np.zeros(shape);masks=[np.zeros(shape,bool) for _ in range(16)]
        masks[1][3,3]=True
        terrain=dict(blocked=np.zeros(shape,bool),gx=zero,gy=zero,slope=zero,heading_masks=masks)
        route=heading_astar(terrain,(3,3),(4,4),0,self.cfg)
        self.assertTrue(route)
        for a,b in zip(route.states,route.states[1:]):
            if a[:2]==b[:2]:
                difference=((b[2]-a[2]+4)%8)-4
                self.assertFalse(masks[(a[2]*2+difference)%16][a[1],a[0]])

    def test_frame_shadow_is_modeled_without_erasing_the_ground(self):
        from geometry import visible_past_boxes
        g=Geometry(self.cfg)
        endpoints=np.array([[2.,0,.3],[2.,1.5,-.3]])
        visible=visible_past_boxes(endpoints,np.asarray(self.cfg['lidar_position_body_m']),g.boxes)
        self.assertFalse(visible[0]);self.assertTrue(visible[1])

    def test_ground_truth_proximity_cannot_complete_task(self):
        sim=Simulation('gentle_slope',self.cfg);sim.world.goal[:]=0.
        for _ in range(5):sim.step()
        self.assertIsNotNone(sim.first_proximity_time)
        self.assertFalse(sim.done);self.assertNotEqual(sim.nav.task.state,'COMPLETE')

    def test_lidar_origin_is_not_the_body_origin(self):
        from geometry import body_pose_from_lidar
        radar=Pose(100.,np.array([-.15,0,.45]),self.pose.quaternion)
        with self.assertRaises(ValueError):body_pose_from_lidar(radar,self.cfg)
        body=body_pose_from_lidar(radar,self.cfg,allow_provisional=True)
        np.testing.assert_allclose(body.position,self.pose.position)
        cfg=dict(self.cfg,lidar_extrinsics_calibrated=True)
        np.testing.assert_allclose(body_pose_from_lidar(radar,cfg).position,self.pose.position)

    def test_rejected_session_switch_cannot_shortcut_session_expiry(self):
        m=DirectionManager(self.cfg)
        self.assertTrue(m.accept(self.msg(30,100,session_id='a'),self.pose,100,0))
        for i in range(1,4):
            p=copy.deepcopy(self.pose);p.stamp=100+i*.1
            self.assertFalse(m.accept(self.msg(0,p.stamp,session_id='b'),p,p.stamp,i*.1))
        self.assertEqual(m.session_id,'a')


if __name__=='__main__':unittest.main(verbosity=2)
