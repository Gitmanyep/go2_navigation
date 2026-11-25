import json
from pathlib import Path
import unittest
import numpy as np
from native.rolling_map import NativeRollingMap, SourceClock, fit_floor
ROOT = Path(__file__).resolve().parents[1]


class NativeRollingTests(unittest.TestCase):
    def setUp(self):
        self.settings = json.loads((ROOT/'config/native_rolling.json').read_text())
        self.engine = NativeRollingMap(self.settings)
        y, x = np.mgrid[-1.8:1.81:.06, -1.8:3.81:.06]
        self.floor = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]

    def sample(self, i, x=0., z=.25, cloud=None):
        stamp, mono = 1000+i*.2, 10+i*.2
        self.engine.pose_input(stamp, [x, 0, z], [0, 0, 0, 1], 'odom', 'base_link', mono)
        return self.engine.cloud_input(stamp, self.floor if cloud is None else cloud, 'odom', mono)

    def mapped(self):
        for i in range(12):
            self.sample(i)
        snap = self.engine.snapshot(12.2)
        self.assertTrue(snap['valid'], snap['reason'])
        self.assertTrue(snap['support_valid'])
        return snap

    def test_native_epoch_offset_is_not_treated_as_latency(self):
        snap = self.mapped()
        self.assertGreater(len(snap['elevation_points']), 100)
        self.assertFalse(snap['audit']['timestamp_alignment_verified'])
        self.assertFalse(snap['audit']['traversability_suitable_for_control'])
        self.assertTrue((snap['traversability'] < 0).any())
        self.assertTrue((snap['traversability'] == 0).any())
        self.assertEqual(snap['source_stamp'], 1002.2)

    def test_clock_rejects_backlog_replays_and_outlier_without_reanchoring(self):
        c = SourceClock()
        c.observe('odom', 1000, 10)
        c.observe('cloud', 1000, 10)
        for stamp, mono in [(1000, 10.1), (1000.1, 11), (99999999, 10.2)]:
            with self.assertRaises(ValueError):
                c.observe('cloud', stamp, mono)
        c.observe('cloud', 1000.3, 10.3)
        self.assertEqual(c.anchor, (1000, 10))

    def test_dropout_clears_all_three_layers(self):
        self.mapped()
        snap = self.engine.snapshot(13.)
        self.assertFalse(snap['valid'])
        self.assertEqual(len(snap['elevation_points']), 0)
        self.assertEqual(len(snap['obstacle_points']), 0)
        self.assertTrue((snap['traversability'] == -1).all())

    def test_stand_crouch_does_not_move_ground_plane(self):
        self.mapped()
        for i in range(12, 18):
            self.sample(i, z=.50)
        snap = self.engine.snapshot(13.4)
        self.assertTrue(snap['valid'])
        self.assertLess(abs(snap['elevation_points'][:, 2]).max(), .001)
        self.assertAlmostEqual(snap['body_pose']['position'][2], .5)

    def test_pose_jump_invalidates_until_restart(self):
        self.mapped()
        self.sample(12, x=2.)
        self.assertFalse(self.engine.snapshot(12.4)['valid'])
        self.assertEqual(self.engine.restart_required, 'odometry_jump_restart_required')

    def test_invalid_frame_and_nonfinite_points_fail_closed(self):
        self.mapped()
        self.assertFalse(self.engine.cloud_input(1002.4, self.floor, 'camera_init', 12.4))
        self.assertFalse(self.engine.snapshot(12.4)['valid'])
        self.assertFalse(self.engine.cloud_input(1002.6, np.full((200, 3), np.nan), 'odom', 12.6))

    def test_history_expires_without_refilling_holes(self):
        self.mapped()
        last = self.engine.map.seen.copy()
        terrain = self.engine.map.analyze(23.)
        self.assertFalse(terrain['known'].any())
        np.testing.assert_array_equal(last, self.engine.map.seen)

    def test_obstacle_and_rolling_window_keep_observation_age(self):
        self.mapped()
        x, y, z = np.meshgrid(np.arange(.75, 1.05, .06), np.arange(-.15, .16, .06), np.arange(.1, .41, .06))
        obstacle = np.c_[x.ravel(), y.ravel(), z.ravel()]
        for i in range(12, 17):
            self.sample(i, cloud=np.r_[self.floor, obstacle])
        snap = self.engine.snapshot(13.2)
        self.assertGreater(len(snap['obstacle_points']), 0)
        self.assertGreater(np.sum(snap['traversability'] == 100), 0)
        old_origin = self.engine.map.origin.copy()
        seen = self.engine.map.seen[np.isfinite(self.engine.map.elevation)].copy()
        self.engine.map.recenter([4., 0., .3])
        self.assertFalse(np.array_equal(old_origin, self.engine.map.origin))
        retained = self.engine.map.seen[np.isfinite(self.engine.map.elevation)]
        self.assertTrue(np.isin(retained, seen).all())
        self.assertTrue(np.isnan(self.engine.map.elevation).any())

    def test_zero_padding_never_creates_ground_at_world_origin(self):
        xyz = np.r_[self.floor+np.array([0., 0., -.15]), np.zeros((9000, 3))]
        for i in range(12):
            self.sample(i, z=.1, cloud=xyz)
        snap = self.engine.snapshot(12.2)
        self.assertTrue(snap['valid'], snap['reason'])
        self.assertGreater(snap['audit']['zero_points_dropped'], 9000)
        self.assertTrue(np.allclose(snap['elevation_points'][:, 2], -.15))

    def test_already_cleaned_small_cloud_uses_same_valid_point_threshold(self):
        self.mapped()
        near = self.floor[np.linalg.norm(self.floor[:, :2], axis=1) < 1.2][:64]
        self.assertEqual(len(near), 64)
        self.assertTrue(self.sample(12, cloud=near), self.engine.errors['cloud'])

    def test_ground_bootstrap_requires_real_support(self):
        with self.assertRaises(ValueError):
            fit_floor(self.floor+np.array([0, 0, .5]), np.array([0, 0, .25]))


if __name__ == '__main__':
    unittest.main()
