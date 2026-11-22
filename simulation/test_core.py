"""Behavioral checks for interfaces, 2.5D terrain and fail-closed navigation."""
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np
from core import config,Pose,Navigator,DirectionManager,TerrainMap,astar,directional_slope,integrate_twist,segment_clear
from evidence import pcd_xyz
from simulation import World,Simulation


class FrameworkTests(unittest.TestCase):
    def setUp(self):
        self.cfg = config()
        self.now = 100.
        self.pose = Pose(100.,np.array([0.,0.,.3]),np.array([0.,0.,0.,1.]))

    def message(self,seq=1,stamp=100.,vector=None):
        return dict(seq=seq,stamp_sec=stamp,frame_id='body',kind='direction',
                    vector=[1,0,.3] if vector is None else vector,confidence=.9,valid=True)

    def flat_map(self):
        world = World('gentle_slope',self.cfg)
        world.z[:] = 0.;world.gx[:] = 0.;world.gy[:] = 0.
        terrain = TerrainMap(self.cfg,[0,0])
        terrain.integrate(world.cloud([0,0],np.random.RandomState(1)),[0,0],0.)
        return terrain

    def test_left_right_coordinates(self):
        for sign in [-1,1]:
            manager = DirectionManager(self.cfg)
            self.assertTrue(manager.accept(self.message(vector=[1,sign,.2]),self.pose,100,0))
            self.assertGreater(sign*manager.goal[1],0)
            self.assertAlmostEqual(manager.goal[2],self.pose.position[2])

    def test_yaw_rotation(self):
        self.pose.quaternion = np.array([0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)])
        m = DirectionManager(self.cfg)
        self.assertTrue(m.accept(self.message(),self.pose,100,0))
        self.assertAlmostEqual(m.goal[0],0,places=6);self.assertGreater(m.goal[1],1.9)

    def test_replay_direction_invalidates_previous_goal(self):
        m = DirectionManager(self.cfg)
        self.assertTrue(m.accept(self.message(),self.pose,100,0))
        self.assertFalse(m.accept(self.message(),self.pose,100,0))
        self.assertIsNone(m.goal)

    def test_invalid_directions(self):
        edits = [dict(vector=[0,0,0]),dict(vector=[0,0,1]),dict(vector=[float('nan'),0,1]),
                 dict(vector=[1,2]),dict(stamp_sec=99.),dict(stamp_sec=101.),
                 dict(confidence=.1),dict(confidence=float('nan')),dict(valid=False),
                 dict(seq=True),dict(seq=-1),dict(frame_id='map'),dict(frame_id='mic_array')]
        for edit in edits:
            with self.subTest(edit=edit):
                m = DirectionManager(self.cfg);msg = self.message();msg.update(edit)
                self.assertFalse(m.accept(msg,self.pose,100,0));self.assertIsNone(m.goal)

    def test_mic_rotation_validation(self):
        for r in [np.zeros((3,3)),np.diag([1,1,-1]),np.full((3,3),np.nan)]:
            with self.assertRaises(ValueError):DirectionManager(self.cfg,r)
        m = DirectionManager(self.cfg,np.eye(3));msg = self.message();msg['frame_id']='mic_array'
        self.assertTrue(m.accept(msg,self.pose,100,0))

    def test_clock_rollback_does_not_extend_direction(self):
        m = DirectionManager(self.cfg)
        self.assertTrue(m.accept(self.message(),self.pose,100,0))
        self.assertFalse(m.valid(99.98,.51))

    def test_source_age_consumes_direction_lease(self):
        self.pose.stamp = 99.6
        m = DirectionManager(self.cfg)
        self.assertTrue(m.accept(self.message(stamp=99.6),self.pose,100,0))
        self.assertTrue(m.valid(100.05,.05));self.assertFalse(m.valid(100.15,.15))

    def test_jitter_holds_anchored_goal(self):
        m = DirectionManager(self.cfg)
        for i in range(50):
            t = 100+i*.05;self.pose.stamp=t
            a = math.radians(8 if i%2 else -8)
            self.assertTrue(m.accept(self.message(i+1,t,[math.cos(a),math.sin(a),.2]),self.pose,t,i*.05))
        self.assertEqual(m.updates,1)

    def test_known_floor_and_unknown_are_distinct(self):
        m = self.flat_map();t=m.analyze(0)
        self.assertFalse(t['blocked'][m.cell([0,0])[1],m.cell([0,0])[0]])
        self.assertTrue((t['grid']<0).any());self.assertTrue((t['grid']==0).any())
        self.assertTrue(m.analyze(2.)['blocked'].all())

    def test_missing_floor_is_never_ray_cleared(self):
        world = World('dropoff',self.cfg)
        m = TerrainMap(self.cfg,[0,0]);m.integrate(world.cloud([0,0],np.random.RandomState(1)),[0,0],0)
        x,y=m.cell([1.6,0]);self.assertEqual(m.analyze(0)['grid'][y,x],-1)

    def test_obstacle_evidence_does_not_disappear_on_floor_return(self):
        m=self.flat_map();c=m.cell([1.5,0]);xy=m.world(c)
        points=np.array([[xy[0]-.01,xy[1]-.01,0],[xy[0]+.01,xy[1],.6],[xy[0],xy[1]+.01,.3]])
        m.integrate(points,[0,0],.1)
        m.integrate(points* [1,1,0],[0,0],.2)
        self.assertEqual(m.analyze(.2)['grid'][c[1],c[0]],100)

    def test_slope_analysis(self):
        for name,angle in [('gentle_slope',8),('cross_slope',6),('unsafe_cross_slope',13)]:
            w=World(name,self.cfg);m=TerrainMap(self.cfg,[0,0]);m.integrate(w.cloud([0,0],np.random.RandomState(1)),[0,0],0)
            t=m.analyze(0);c=m.cell([1.5,0]);p,r=directional_slope(t,c,0)
            measured=abs(p) if name=='gentle_slope' else abs(r)
            self.assertAlmostEqual(measured,angle,delta=1.)

    def test_obstacle_occludes_ground(self):
        w=World('flat_detour',self.cfg);points=w.cloud([0,0],np.random.RandomState(1))
        self.assertTrue((points[:,2]>.5).any())
        shadow=(points[:,0]>1.4)&(points[:,0]<2.5)&(np.abs(points[:,1])<.1)
        self.assertFalse(shadow.any())

    def test_recenter_preserves_evidence(self):
        m=self.flat_map();point=np.array([1.2,.3]);c=m.cell(point);old=m.elevation[c[1],c[0]]
        m.recenter([3.1,0]);new=m.cell(point)
        self.assertAlmostEqual(m.elevation[new[1],new[0]],old)

    def test_astar_cannot_cut_diagonal(self):
        blocked=np.array([[False,True],[True,False]])
        zero=np.zeros((2,2));t=dict(blocked=blocked,gx=zero,gy=zero,slope=zero,clearance=zero+1)
        self.assertEqual(astar(t,(0,0),(1,1),self.cfg),[])

    def test_astar_rejects_cross_slope(self):
        zero=np.zeros((8,8));t=dict(blocked=zero.astype(bool),gx=zero,gy=zero+math.tan(math.radians(13)),slope=zero+13,clearance=zero+2)
        t['blocked'][:]=True;t['blocked'][3,:]=False
        self.assertEqual(astar(t,(0,3),(7,3),self.cfg),[])

    def test_no_sensors_zero(self):
        self.assertEqual(Navigator(self.cfg).step(100,0)['candidate'],[0.,0.])

    def navigator_ready(self):
        n=Navigator(self.cfg);n.pose_input(self.pose,100,0)
        w=World('gentle_slope',self.cfg);w.z[:]=0;w.gx[:]=0;w.gy[:]=0
        points=w.cloud([0,0],np.random.RandomState(1))
        self.assertTrue(n.cloud_input(points,100,'camera_init',100,0))
        self.assertTrue(n.direction_input(self.message(),100,0))
        for i in range(16):
            stamp=98.5+i*.1
            pose=copy.deepcopy(self.pose);pose.stamp=stamp
            # Rebuild to provide chronological inputs during the startup listen.
            if i==0:n=Navigator(self.cfg)
            n.pose_input(pose,stamp,stamp-100)
            n.cloud_input(points,stamp,'camera_init',stamp,stamp-100)
            n.direction_input(self.message(i+1,stamp),stamp,stamp-100)
            state=n.step(stamp,stamp-100)
        self.assertGreater(state['candidate'][0],0)
        return n,points

    def test_cloud_replay_stops(self):
        n,p=self.navigator_ready();self.assertFalse(n.cloud_input(p,100,'camera_init',100,.01))
        self.assertEqual(n.step(100,.01)['candidate'],[0.,0.])

    def test_stale_rewrapped_cloud_stops(self):
        n,p=self.navigator_ready();self.assertFalse(n.cloud_input(p,99.,'camera_init',100,.01))
        self.assertEqual(n.step(100,.01)['candidate'],[0.,0.])

    def test_sensor_expiry_and_monotonic_deadline(self):
        n,_=self.navigator_ready();self.assertEqual(n.step(99.99,.6)['candidate'],[0.,0.])

    def test_mismatched_frame_stops(self):
        n,p=self.navigator_ready();self.assertFalse(n.cloud_input(p,100.01,'map',100.01,.01))
        self.assertEqual(n.step(100.01,.01)['candidate'],[0.,0.])

    def test_nonfinite_cloud_stops(self):
        n,p=self.navigator_ready();p[0,0]=float('nan')
        self.assertFalse(n.cloud_input(p,100.01,'camera_init',100.01,.01))
        self.assertEqual(n.step(100.01,.01)['candidate'],[0.,0.])

    def test_invalid_pose_stops(self):
        n,_=self.navigator_ready();p=copy.deepcopy(self.pose);p.stamp=100.01;p.quaternion[:]=0
        self.assertFalse(n.pose_input(p,100.01,.01));self.assertEqual(n.step(100.01,.01)['candidate'],[0.,0.])

    def test_turn_integrator(self):
        xy,a=integrate_twist([0,0],0,1,1,math.pi/2)
        np.testing.assert_allclose(xy,[1,1],atol=1e-7);self.assertAlmostEqual(a,math.pi/2)

    def test_ascii_and_binary_pcd(self):
        with tempfile.TemporaryDirectory() as d:
            for mode in ['ascii','binary']:
                p=Path(d)/'map.pcd';header=f'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA {mode}\n'.encode()
                values=np.array([[1,2,3],[4,5,6]],dtype='<f4')
                p.write_bytes(header+(b'1 2 3\n4 5 6\n' if mode=='ascii' else values.tobytes()))
                np.testing.assert_allclose(pcd_xyz(p),values)


    def test_lookahead_checks_tiny_corner_intervals(self):
        m=TerrainMap(self.cfg,[0,0]);blocked=np.zeros((m.n,m.n),bool)
        a=m.origin+np.array([10.9,10.9])*m.res
        b=m.origin+np.array([12.1,12.09])*m.res
        blocked[10,11]=True
        self.assertFalse(segment_clear(a,b,m,blocked))

    def test_sdk_imports_are_blocked_during_navigation(self):
        import importlib.abc,sys
        class NoRobot(importlib.abc.MetaPathFinder):
            def find_spec(self,fullname,path=None,target=None):
                if fullname.split('.')[0] in ['unitree_sdk2py','cyclonedds']:
                    raise AssertionError('robot SDK import: '+fullname)
        guard=NoRobot();sys.meta_path.insert(0,guard)
        try:
            sim=Simulation('gentle_slope',self.cfg)
            for _ in range(20):sim.step()
            self.assertEqual(sim.state['motion_commands_sent'],0)
            self.assertFalse(sim.state['executable'])
        finally:sys.meta_path.remove(guard)


if __name__=='__main__':
    unittest.main(verbosity=2)
