import copy
import json
from pathlib import Path
import unittest
import numpy as np
from native.obstacle_plan import ObstaclePreview, astar
ROOT=Path(__file__).resolve().parents[1]


class ObstacleOnlyTests(unittest.TestCase):
    def setUp(self):
        self.s=json.loads((ROOT/'config/native_obstacle_plan.json').read_text())
        self.s['floor_z_m']=0.
        self.s['planning_budget_s']=.3
        self.e=ObstaclePreview(self.s)
    def pose(self, stamp=1000., mono=10., position=(0.,0.,.3)):
        return self.e.pose_input(stamp,position,[0.,0.,0.,1.],'odom','base_link',mono)
    def cloud(self, xyz=(), stamp=1000., mono=10., frame='odom', size=None):
        return self.e.cloud_input(stamp,np.asarray(xyz,float).reshape(-1,3),frame,mono,size)
    def ready(self, xyz=()):
        self.assertTrue(self.pose());self.assertTrue(self.cloud(xyz))
        self.e.set_goal([2.5,0.],'odom')
        return self.e.snapshot(10.)

    def test_empty_voxels_can_preview_only_under_explicit_flat_assumption(self):
        s=self.ready()
        self.assertTrue(s['valid']);self.assertGreater(len(s['path']),10)
        self.assertFalse(s['free_space_verified']);self.assertFalse(s['terrain_analysis_enabled'])
        self.assertFalse(s['traversability_suitable_for_control'])
        self.assertEqual(s['motion_commands_sent'],0)
        self.assertTrue(all(abs(p[2]-(self.s['voxel_size_m']*.5+.04))<1e-8 for p in s['path']))

    def test_box_forces_detour_with_body_clearance(self):
        box=np.array([[x,y,.4] for x in np.arange(.9,1.21,.1) for y in np.arange(-.2,.21,.1)])
        s=self.ready(box)
        self.assertEqual(s['reason'],'obstacle_only_path_preview')
        path=np.asarray(s['path'])
        self.assertGreater(np.max(abs(path[:,1])),.7)
        distances=np.linalg.norm(path[:,:2,None]-box[:,:2].T[None,:,:],axis=1)
        self.assertGreater(distances.min(),self.s['robot_radius_m']+self.s['collision_margin_m'])

    def test_floor_and_high_ceiling_are_not_projected_as_body_obstacles(self):
        floor=np.array([[x,y,0.] for x in np.arange(-2.,3.,.1) for y in np.arange(-2.,2.,.1)])
        roof=floor+np.array([0,0,2.])
        s=self.ready(np.r_[floor,roof])
        self.assertGreater(len(s['path']),10)
        self.assertEqual(self.e.filtered_floor_points,len(floor))
        self.assertEqual(self.e.out_of_envelope_points,len(roof))
        self.assertEqual(s['obstacle_cells'],0)

    def test_branch_in_envelope_blocks_direct_route(self):
        s=self.ready([[1.,0.,.65]])
        self.assertGreater(max(abs(p[1]) for p in s['path']),.5)

    def test_narrow_passage_and_goal_in_box_are_rejected(self):
        walls=[[x,y,.4] for x in np.arange(-2.,3.,.1) for y in (-.4,.4)]
        s=self.ready(walls);self.assertEqual(s['path'],[])
        self.assertIn('start_in_collision',s['reason'])
        self.setUp();s=self.ready([[2.5,0.,.4]])
        self.assertEqual(s['path'],[]);self.assertIn('goal_in_collision',s['reason'])

    def test_outside_declared_area_is_not_assumed_usable(self):
        self.ready();self.e.set_goal([7.,0.],'odom');s=self.e.snapshot(10.)
        self.assertFalse(s['path']);self.assertTrue((s['grid']<0).any())

    def test_input_timeout_clears_path_and_candidate_and_recovers(self):
        self.ready();s=self.e.snapshot(11.1)
        self.assertFalse(s['valid']);self.assertEqual(s['path'],[])
        self.assertEqual(s['candidate'],[0.,0.]);self.assertTrue((s['grid']==-1).all())
        self.pose(1001.2,11.2);self.cloud(stamp=1001.2,mono=11.2)
        self.assertTrue(self.e.snapshot(11.2)['path'])

    def test_absent_voxels_do_not_erase_obstacle_evidence(self):
        self.ready([[1.,0.,.4]]);count=len(self.e.obstacles)
        self.pose(1000.1,10.1);self.cloud(stamp=1000.1,mono=10.1)
        self.assertEqual(len(self.e.obstacles),count)
        self.assertGreater(max(abs(p[1]) for p in self.e.snapshot(10.1)['path']),.5)

    def test_new_floor_reset_requires_new_cloud_before_any_route(self):
        self.ready([[1.,0.,.4]])
        self.e.set_floor(0.,'odom');self.e.set_goal([2.5,0.],'odom')
        self.assertFalse(self.e.snapshot(10.)['valid'])
        self.pose(1000.1,10.1);self.cloud(stamp=1000.1,mono=10.1)
        self.assertTrue(self.e.snapshot(10.1)['path'])
        self.assertEqual(self.e.snapshot(10.1)['obstacle_cells'],0)

    def test_source_jump_or_pose_jump_requires_restart(self):
        self.ready();self.assertFalse(self.cloud(stamp=999.,mono=10.1))
        self.assertFalse(self.e.snapshot(10.1)['valid'])
        self.setUp();self.ready();self.assertFalse(self.pose(1000.1,10.1,(2.,0.,.3)))
        self.assertEqual(self.e.snapshot(10.1)['path'],[])

    def test_wrong_frame_invalid_points_and_skew_fail_closed(self):
        self.ready();self.assertFalse(self.cloud([[1.,0.,.4]],1000.1,10.1,'camera_init'))
        self.assertFalse(self.e.snapshot(10.1)['valid'])
        self.assertFalse(self.cloud([[float('nan'),0.,.4]],1000.1,10.1))
        self.setUp();self.ready();self.pose(1000.8,10.8)
        s=self.e.snapshot(10.8);self.assertFalse(s['valid']);self.assertEqual(s['reason'],'voxel_odometry_time_mismatch')

    def test_floor_must_be_selected_or_configured_without_terrain_estimation(self):
        self.s['floor_z_m']=None;self.e=ObstaclePreview(self.s)
        self.pose();self.assertFalse(self.cloud([[1.,0.,.4]]))
        self.assertEqual(self.e.snapshot(10.)['reason'],'click_floor_to_declare_flat_test_area')
        self.e.set_floor(0.,'odom');self.pose(1000.1,10.1);self.cloud([[1.,0.,.4]],1000.1,10.1)
        self.e.set_goal([2.5,0.],'odom')
        self.assertTrue(self.e.snapshot(10.1)['path'])

    def test_direction_transform_and_focus_clear_expiry(self):
        self.ready();self.e.set_direction([0.,1.,0.],10.)
        s=self.e.snapshot(10.);self.assertAlmostEqual(s['goal'][1],2.)
        self.assertEqual(self.e.snapshot(10.6)['candidate'],[0.,0.])
        self.e.set_direction([0,0,0],10.2);self.assertIsNone(self.e.direction)
        with self.assertRaises(ValueError):self.e.set_direction([1,0,1],10.2)

    def test_voxel_volume_and_metadata_can_only_enlarge_retained_clearance(self):
        self.ready([[1.,0.,.4]]);a=self.e.snapshot(10.)['footprint_inflation_m']
        self.pose(1000.1,10.1);self.cloud([[1.,0.,.4]],1000.1,10.1,size=.2)
        self.assertGreater(self.e.snapshot(10.1)['footprint_inflation_m'],a)
        self.pose(1000.2,10.2);self.cloud([[1.,0.,.4]],1000.2,10.2,size=.05)
        self.assertEqual(self.e.voxel_size,.2)

    def test_astar_does_not_cut_diagonal_corners(self):
        blocked=np.array([[False,True],[True,False]])
        route,reason=astar(blocked,(0,0),(1,1),.1)
        self.assertEqual(route,[]);self.assertEqual(reason,'no_collision_free_route')


if __name__=='__main__':unittest.main()
