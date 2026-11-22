"""Sector/near-ring boundaries, moving reference and fail-closed integration."""
import copy
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from perception.spatial_roi import SpatialROI
from perception.pipeline import load_settings, Pose, MappingEngine, expired_snapshot
from perception.reference_preview import ReferencePreviewEngine
from perception.dense_cloud import DenseProjector


def config(frame='pointlio_reference'):
    return dict(frame=frame, front_angle_deg=130., front_range_m=3., near_radius_m=1.5,
                fine_voxel_m=.04, near_voxel_m=.12)


def pose(yaw=0., position=(0., 0., 0.), pitch=0.):
    return Pose(100., np.asarray(position, float),
                Rotation.from_euler('ZY', [yaw, pitch]).as_quat(), 'camera_init')


def polar(degrees, radius=2., z=0.):
    a=np.radians(degrees)
    return np.column_stack([radius*np.cos(a), radius*np.sin(a), np.full(len(a), z)])


class SpatialROITests(unittest.TestCase):
    def setUp(self):
        self.roi=SpatialROI(config())

    def test_130_degree_boundary_and_rear_near_ring(self):
        points=np.r_[polar([-66.,-65.,0.,65.,66.]),polar([90.,180.,-90.],1.5),
                     polar([180.],1.5001),polar([0.],3.0001)]
        selected,report=self.roi.select(points,pose(),downsample=False)
        np.testing.assert_allclose(selected,points[[1,2,3,5,6,7]])
        self.assertEqual(report['fine_points'],3);self.assertEqual(report['near_points'],3)
        self.assertFalse(report['cropped_regions_are_free'])
        self.assertFalse(report['body_alignment_verified'])

    def test_world_heading_and_origin_follow_translation_and_rotation(self):
        local=np.r_[polar([0.,60.,80.,180.]),polar([180.],1.)]
        p=pose(math.pi/2,position=(5.,-2.,.4))
        world=local@p.rotation.T+p.position
        selected,_=self.roi.select(world,p,downsample=False)
        np.testing.assert_allclose(selected,world[[0,1,4]])

    def test_sector_is_horizontal_on_a_slope_and_preserves_overhead_returns(self):
        points=np.array([[2.,0.,-2.],[2.,0.,3.],[-1.,0.,-1.],[-1.,0.,2.],[0.,2.,.5]])
        selected,_=self.roi.select(points,pose(pitch=.4),downsample=False)
        np.testing.assert_allclose(selected,points[:4])

    def test_vertical_reference_heading_fails_instead_of_guessing(self):
        with self.assertRaisesRegex(ValueError,'heading'):
            self.roi.select([[0.,0.,0.]],pose(pitch=math.pi/2))

    def test_near_voxels_keep_original_3d_returns_without_filling_space(self):
        points=np.array([[-1.01,.01,.01],[-1.02,.02,.02],[-1.01,.01,.3],
                         [2.01,.01,.01],[2.06,.01,.01]])
        selected,report=self.roi.select(points,pose())
        self.assertEqual(report['near_points'],2);self.assertEqual(report['fine_points'],2)
        np.testing.assert_allclose(selected,points[[0,2,3,4]])
        self.assertTrue(all(any(np.array_equal(p,q) for q in points) for p in selected))

    def test_empty_and_disabled_selection_do_not_invent_returns(self):
        selected,report=self.roi.select([],pose())
        self.assertEqual(selected.shape,(0,3));self.assertEqual(report['output_points'],0)
        points=polar([0.,180.],20.)
        result,_=SpatialROI().select(points,pose())
        np.testing.assert_array_equal(points,result)

    def test_fast_voxels_preserve_the_same_original_representative_per_cell(self):
        rng=np.random.default_rng(130)
        points=np.r_[rng.uniform(-3.,3.,(2000,3)),np.repeat([[1.01,.01,.01]],20,axis=0)]
        indexes=rng.permutation(len(points))
        for voxel in (.04,.12):
            _,expected=np.unique(np.floor(points[indexes]/voxel).astype(np.int64),axis=0,return_index=True)
            actual=SpatialROI.representatives(points,indexes,voxel)
            np.testing.assert_array_equal(np.sort(actual),np.sort(indexes[expected]))

    def test_large_voxel_key_span_falls_back_without_integer_overflow(self):
        points=np.array([[-1e6,-1e6,-1e6],[1e6,1e6,1e6],[0.,0.,0.],[1e6,1e6,1e6]])
        indexes=np.arange(len(points))
        with np.errstate(over='raise',invalid='raise'):
            result=SpatialROI.representatives(points,indexes,.005)
        np.testing.assert_array_equal(np.sort(result),[0,1,2])

    def test_invalid_schema_values_and_coordinate_scope_are_rejected(self):
        for value in ([],False,'130'):
            with self.assertRaises(ValueError):SpatialROI(value)
        for key,value in [('front_angle_deg',0.),('front_angle_deg',361.),('front_angle_deg',True),
                          ('front_range_m',1.),('near_radius_m',float('nan')),
                          ('near_voxel_m',.5),('fine_voxel_m',.2),('frame','world')]:
            bad=config();bad[key]=value
            with self.assertRaises(ValueError):SpatialROI(bad)
        bad=config();bad['typo']=1
        with self.assertRaises(ValueError):SpatialROI(bad)
        with self.assertRaisesRegex(ValueError,'frame'):
            SpatialROI(config(),expected_frame='body')

    def test_invalid_coordinates_cannot_be_voxelized(self):
        for points in ([[float('nan'),0.,0.]], [[1e30,0.,0.]]):
            with self.assertRaises(ValueError):self.roi.select(points,pose())

    def test_preset_does_not_change_full_circle_or_freshness_defaults(self):
        full,_,_,_=load_settings(ROOT/'config/lidar_dense_preview.json')
        sector,_,_,_=load_settings(ROOT/'config/lidar_sector_preview.json')
        self.assertNotIn('spatial_roi',full)
        for key in ('max_source_age_s','snapshot_lease_s','pair_max_skew_s','dense_visualization'):
            self.assertEqual(full[key],sector[key])
        self.assertEqual(sector['spatial_roi']['front_angle_deg'],130.)
        mapping,cal,_,_=load_settings(ROOT/'config/perception_sector.json')
        self.assertEqual(mapping['spatial_roi']['frame'],'body')
        self.assertFalse(cal['lidar_extrinsics_calibrated']);self.assertFalse(cal['geometry_calibrated'])


class SectorIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.settings,self.cal,self.cfg,_=load_settings(ROOT/'config/lidar_sector_preview.json')
        self.engine=ReferencePreviewEngine(self.settings,self.cal,self.cfg)
        self.points=np.r_[polar([0.,30.,60.,90.,180.,-90.],2.),polar([90.,180.,-90.],1.)]

    def live(self,yaw=0.,stamp=100.,points=None):
        q=Rotation.from_euler('z',yaw).as_quat()
        self.assertTrue(self.engine.pose_input(stamp,[0.,0.,0.],q,'camera_init','body',stamp,stamp-100.))
        self.assertTrue(self.engine.cloud_input(stamp,self.points if points is None else points,'camera_init',stamp,stamp-100.))
        return self.engine.snapshot(stamp,stamp-100.)

    def test_preview_retains_forward_fine_and_near_rear_without_navigation_claim(self):
        s=self.live()
        np.testing.assert_allclose(s['obstacle_points'],self.points[[0,1,2,6,7,8]])
        self.assertTrue((s['occupancy']==-1).all());self.assertIsNone(s['body_pose'])
        self.assertFalse(s['audit']['navigation_usable']);self.assertFalse(s['audit']['ground_classification_performed'])
        self.assertEqual(s['audit']['spatial_roi']['frame'],'pointlio_reference')

    def test_retained_history_is_clipped_again_after_turning(self):
        self.live()
        self.assertTrue(self.engine.pose_input(100.1,[0.,0.,0.],Rotation.from_euler('z',math.pi/2).as_quat(),
                                               'camera_init','body',100.1,.1))
        s=self.engine.snapshot(100.1,.1)
        self.assertTrue(s['valid']);self.assertFalse(np.any(np.all(np.isclose(s['obstacle_points'],[2.,0.,0.]),axis=1)))
        self.assertTrue(np.any(np.all(np.isclose(s['obstacle_points'],[-1.,0.,0.]),axis=1)))
        self.assertTrue(self.engine.roi.coverage(s['obstacle_points'],self.engine.poses[-1]).all())

    def test_sector_does_not_renew_source_time_or_worker_lease(self):
        s=self.live();expired=expired_snapshot(s,100.31,.31,self.settings)
        self.assertFalse(expired['valid']);self.assertEqual(len(expired['obstacle_points']),0)
        self.assertEqual(expired['source_stamp'],100.)
        stale=self.engine.snapshot(100.6,.6)
        self.assertFalse(stale['valid']);self.assertEqual(len(stale['obstacle_points']),0)

    def test_excluded_returns_do_not_keep_previous_preview_alive(self):
        self.live()
        self.assertTrue(self.engine.pose_input(100.1,[0.,0.,0.],[0.,0.,0.,1.],'camera_init','body',100.1,.1))
        points=polar([170.,180.,190.],2.)
        self.assertFalse(self.engine.cloud_input(100.1,points,'camera_init',100.1,.1))
        s=self.engine.snapshot(100.1,.1)
        self.assertFalse(s['valid']);self.assertEqual(len(s['obstacle_points']),0)

    def test_dense_and_preview_coordinate_contracts_cannot_be_swapped(self):
        settings=copy.deepcopy(self.settings);settings['spatial_roi']['frame']='body'
        with self.assertRaisesRegex(ValueError,'frame'):ReferencePreviewEngine(settings,self.cal,self.cfg)
        params=dict(extrinsic_T=[0.,0.,0.],extrinsic_R=np.eye(3).ravel().tolist(),extrinsic_est_en=False,scan_line=4)
        with self.assertRaisesRegex(ValueError,'frame'):DenseProjector(settings,params)
        with self.assertRaisesRegex(ValueError,'frame'):MappingEngine(self.settings,self.cal,self.cfg)

    def test_raw_dense_projection_filters_after_deskew_before_voxels(self):
        params=dict(extrinsic_T=[0.,0.,0.],extrinsic_R=np.eye(3).ravel().tolist(),extrinsic_est_en=False,scan_line=4)
        engine=DenseProjector(self.settings,params)
        for stamp in (99.95,100.1):
            self.assertTrue(engine.pose_input(stamp,[0.,0.,0.],[0.,0.,0.,1.],'camera_init','body',100.12,.12))
        points=np.r_[polar([0.,30.,60.,90.,180.,-90.],2.),polar([90.,180.,-90.],1.)]
        msg=NS(header=NS(stamp=NS(to_sec=lambda:100.),frame_id='livox_frame'),timebase=100000000000,
               point_num=len(points),points=[NS(x=p[0],y=p[1],z=p[2],offset_time=i*1000000,tag=0,line=0) for i,p in enumerate(points)])
        frame=engine.decode(msg,100.12,.12)
        output=engine.project(frame,100.12,.12)
        np.testing.assert_allclose(output,points[[0,1,2,6,7,8]],atol=1e-7)
        self.assertEqual(engine.status(.12)['spatial_roi']['outside_points'],3)
        self.assertEqual(engine.last_source_stamp,100.008)
        with self.assertRaisesRegex(ValueError,'expired'):engine.project(frame,100.6,.6)

    def test_formal_map_still_requires_physical_calibration(self):
        settings,cal,cfg,_=load_settings(ROOT/'config/perception_sector.json')
        engine=MappingEngine(settings,cal,cfg)
        self.assertTrue(engine.pose_input(100.,[0.,0.,.3],[0.,0.,0.,1.],'camera_init','body',100.,0.))
        self.assertFalse(engine.cloud_input(100.,self.points,'camera_init',100.,0.))
        s=engine.snapshot(100.,0.)
        self.assertFalse(s['valid']);self.assertTrue((s['occupancy']==-1).all())
        self.assertIn('extrinsics_unmeasured',s['audit']['calibration_issues'])

    def test_formal_map_masks_old_cells_outside_current_body_sector(self):
        # Reuse the existing measured SYNTHETIC scene, never a real robot calibration.
        import test_perception
        scene=test_perception.PerceptionTests();scene.setUp()
        scene.settings['spatial_roi']=config('body')
        scene.engine=MappingEngine(scene.settings,scene.cal,scene.cfg)
        s=scene.mapped();self.assertTrue(s['valid']);self.assertGreater(len(s['elevation_points']),0)
        selected=scene.engine.map.frames[-1][1]
        self.assertTrue(scene.engine.roi.coverage(selected,pose(position=(0.,0.,.3))).all())
        # Rotate only the body pose; old known cells must not remain valid outside current coverage.
        q=Rotation.from_euler('z',math.pi/2).as_quat()
        self.assertTrue(scene.engine.pose_input(100.4,[0.,.2,.45],q,'camera_init','lidar_imu',100.4,.4))
        s=scene.engine.snapshot(100.4,.4);self.assertTrue(s['valid'])
        p=Pose(100.4,np.array(s['body_pose']['position']),np.array(s['body_pose']['quaternion']),'camera_init')
        yy,xx=np.indices(s['occupancy'].shape)
        centres=np.column_stack([s['origin'][0]+(xx.ravel()+.5)*s['resolution'],
                                 s['origin'][1]+(yy.ravel()+.5)*s['resolution'],np.zeros(xx.size)])
        outside=~scene.engine.roi.coverage(centres,p).reshape(s['occupancy'].shape)
        self.assertTrue((s['occupancy'][outside]==-1).all())
        self.assertTrue(scene.engine.roi.coverage(s['elevation_points'],p).all())
        self.assertTrue(scene.engine.roi.coverage(s['obstacle_points'],p).all())


if __name__=='__main__':
    unittest.main()
