"""Native lidar rolling terrain PREVIEW; no robot transport or actuation.

Source stamps remain in the native clock. Freshness is bounded by local monotonic
receipt and source progress relative to a fixed initial anchor, not wall epoch.
The bootstrap plane is a scene estimate, never a body extrinsic calibration.
"""
from collections import deque
import math
from pathlib import Path
import sys
import numpy as np
from scipy.ndimage import distance_transform_edt
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'simulation'))
from core import Pose, TerrainMap, config
from perception.pipeline import interpolate_pose
from perception.spatial_roi import SpatialROI


class SourceClock:
    def __init__(self, max_lag=.4, warmup=.6):
        self.max_lag, self.warmup = max_lag, warmup
        self.anchor = None
        self.last = {}
        self.count = {}
        self.lag = {}

    def observe(self, kind, stamp, mono):
        if not np.isfinite([stamp, mono]).all() or stamp <= 0:
            raise ValueError('invalid_source_timestamp')
        if self.anchor is None:
            self.anchor = (stamp, mono)
        predicted = self.anchor[0] + mono - self.anchor[1]
        lag = predicted - stamp
        if lag < -.2 or lag > self.max_lag:
            raise ValueError('source_clock_jump_or_backlog')
        if stamp <= self.last.get(kind, -math.inf):
            raise ValueError('source_timestamp_not_increasing')
        self.last[kind] = stamp
        self.count[kind] = self.count.get(kind, 0) + 1
        self.lag[kind] = lag
        return max(0., lag)

    def ready(self, mono):
        return (self.anchor is not None and mono-self.anchor[1] >= self.warmup
                and all(self.count.get(k, 0) >= 3 for k in ('odom', 'cloud')))


def fit_floor(points, position):
    """Bootstrap on an observed near-horizontal surface BELOW the native pose."""
    d = points-position
    radius = np.linalg.norm(d[:, :2], axis=1)
    a = points[(radius > .25) & (radius < 2.) & (d[:, 2] < -.03) & (d[:, 2] > -1.2)]
    if len(a) < 40:
        raise ValueError('bootstrap_ground_unobserved')
    a = a[::max(1, len(a)//1800)]
    rng = np.random.default_rng(11)
    best = None
    best_count = 0
    for _ in range(64):
        tri = a[rng.choice(len(a), 3, replace=False)]
        n = np.cross(tri[1]-tri[0], tri[2]-tri[0])
        if np.linalg.norm(n) < 1e-8 or abs(n[2])/np.linalg.norm(n) < math.cos(math.radians(10)):
            continue
        coeff = np.array([-n[0]/n[2], -n[1]/n[2], tri[0]@n/n[2]])
        errors = abs(a[:, 2]-(a[:, :2]@coeff[:2]+coeff[2]))
        count = int(np.sum(errors < .025))
        if count > best_count:
            best_count, best = count, coeff
    if best is None or best_count < max(30, .3*len(a)):
        raise ValueError('bootstrap_plane_ambiguous')
    keep = abs(a[:, 2]-(a[:, :2]@best[:2]+best[2])) < .025
    best = np.linalg.lstsq(np.c_[a[keep, :2], np.ones(keep.sum())], a[keep, 2], rcond=None)[0]
    depth = position[2]-(position[:2]@best[:2]+best[2])
    if not .03 < depth < 1.2 or np.linalg.norm(best[:2]) > math.tan(math.radians(10)):
        raise ValueError('bootstrap_plane_invalid')
    return best


class NativeRollingMap:
    def __init__(self, settings):
        self.settings = settings
        self.cfg = config()
        self.cfg.update(settings['terrain'])
        if not 0 < self.cfg['map_lease_s'] <= 10:
            raise ValueError('ground history must be within 10 seconds')
        self.cfg['heading_planner_enabled'] = False
        # These flags stay FALSE: native frames and nominal clearance are not a
        # measured robot/head-frame calibration.
        self.cfg['geometry_calibrated'] = False
        self.cfg['lidar_extrinsics_calibrated'] = False
        self.roi = SpatialROI(settings['spatial_roi'])
        self.clock = SourceClock(settings['max_relative_lag_s'])
        self.poses = deque(maxlen=400)
        self.map = None
        self.floor = None
        self.floor_candidates = deque(maxlen=3)
        self.deadlines = dict(odom=-math.inf, cloud=-math.inf)
        self.errors = dict(odom='waiting_for_odometry', cloud='waiting_for_cloud')
        self.source_stamp = 0.
        self.accepted = 0
        self.rejected = 0
        self.pair_skew = None
        self.restart_required = None
        self.zero_points_dropped = 0
        self.nonfinite_points_dropped = 0

    def pose_input(self, stamp, position, quaternion, frame, child, mono):
        try:
            if frame != 'odom' or child != 'base_link':
                raise ValueError('native_odometry_frame_mismatch')
            pose = Pose(stamp, np.asarray(position), np.asarray(quaternion), frame).validate()
            lag = self.clock.observe('odom', stamp, mono)
            if self.poses and np.linalg.norm(pose.position-self.poses[-1].position) > 1.:
                self.restart_required = 'odometry_jump_restart_required'
                self.map = None
                raise ValueError(self.restart_required)
            self.poses.append(pose)
            self.deadlines['odom'] = mono+self.settings['receive_timeout_s']-lag
            self.errors['odom'] = None
            return True
        except ValueError as e:
            self.errors['odom'] = str(e)
            self.deadlines['odom'] = -math.inf
            self.rejected += 1
            return False

    def cloud_input(self, stamp, points, frame, mono):
        try:
            if self.restart_required:
                raise ValueError(self.restart_required)
            if frame != 'odom':
                raise ValueError('native_cloud_frame_mismatch')
            points = np.asarray(points, float)
            if points.ndim != 2 or points.shape[1] != 3 or not 100 <= len(points) <= 200000:
                raise ValueError('invalid_cloud_size')
            finite = np.isfinite(points).all(axis=1) & (np.abs(points) < 1e5).all(axis=1)
            self.nonfinite_points_dropped += int((~finite).sum())
            zero = np.all(points == 0., axis=1)
            self.zero_points_dropped += int(zero.sum())
            points = points[finite & ~zero]
            if len(points) < 30:
                raise ValueError('insufficient_finite_points')
            lag = self.clock.observe('cloud', stamp, mono)
            pose, skew, _ = interpolate_pose(self.poses, stamp, self.settings['pair_max_skew_s'])
            self.pair_skew = skew
            if not self.clock.ready(mono):
                raise ValueError('source_clock_warmup')
            if self.floor is None:
                candidate = fit_floor(points, pose.position)
                self.floor_candidates.append(candidate)
                if len(self.floor_candidates) < 3:
                    raise ValueError('bootstrap_ground_confirmation')
                heights = [pose.position[:2]@c[:2]+c[2] for c in self.floor_candidates]
                if np.ptp(heights) > .04:
                    raise ValueError('bootstrap_ground_unstable')
                self.floor = np.median(self.floor_candidates, axis=0)
            selected, _ = self.roi.select(points, pose)
            if len(selected) < 20:
                raise ValueError('insufficient_roi_returns')
            if self.map is None:
                self.map = TerrainMap(self.cfg, pose.position)
            # A scene-ground reference, distinct from the unmeasured native body
            # datum. Stand/crouch changes do not redefine the floor as an obstacle.
            ground_z = pose.position[:2]@self.floor[:2]+self.floor[2]
            c = self.map.cell(pose.position[:2])
            if self.map.inside(c) and mono-self.map.seen[c[1], c[0]] < self.cfg['map_lease_s']:
                ground_z = self.map.elevation[c[1], c[0]]
            virtual = np.r_[pose.position[:2], ground_z+self.cfg['body_reference_height_m']]
            evidence = mono-lag
            self.map.integrate(selected, virtual, evidence, np.eye(3))
            self.deadlines['cloud'] = evidence+self.settings['receive_timeout_s']
            self.source_stamp = stamp
            self.errors['cloud'] = None
            self.accepted += 1
            return True
        except ValueError as e:
            self.errors['cloud'] = str(e)
            self.deadlines['cloud'] = -math.inf
            self.rejected += 1
            return False

    def snapshot(self, mono):
        valid = bool(self.map is not None and not self.restart_required and mono <= min(self.deadlines.values()))
        n = round(self.cfg['map_size_m']/self.cfg['resolution_m'])
        occupancy = np.full((n, n), -1, np.int8)
        cost = occupancy.copy()
        elevation = obstacles = np.empty((0, 3))
        support = False
        if valid:
            t = self.map.analyze(mono)
            support = bool(t['support_valid'])
            occupancy = t['grid'].copy()
            yy, xx = np.nonzero(t['known'])
            elevation = np.c_[self.map.origin[0]+(xx+.5)*self.map.res,
                              self.map.origin[1]+(yy+.5)*self.map.res, self.map.elevation[yy, xx]]
            if support:
                free = occupancy == 0
                clearance = distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1]*self.map.res
                risk = np.maximum.reduce([t[f]/self.cfg[k] for f, k in
                    [('slope', 'max_pitch_deg'), ('step', 'max_step_m'), ('roughness', 'max_roughness_m')]])
                observed = occupancy >= 0
                blocked = clearance <= self.cfg['footprint_radius_m']+self.map.res*math.sqrt(.5)
                cost[observed] = np.where(blocked[observed], 100,
                                         np.clip(99*risk[observed], 0, 99)).astype(np.int8)
            retained = []
            for stamp, (raw, mask) in self.map.obstacle_frames.items():
                if mono-stamp <= self.cfg['obstacle_hold_s']:
                    xyz = raw[mask]
                    c = np.floor((xyz[:, :2]-self.map.origin)/self.map.res).astype(int)
                    inside = (c >= 0).all(axis=1) & (c < self.map.n).all(axis=1)
                    retained.append(xyz[inside])
            if retained:
                obstacles = np.concatenate(retained)
        reason = self.restart_required or next((e for e in self.errors.values() if e), None)
        if not reason:
            reason = 'nominal_terrain_preview' if valid else 'native_input_timeout'
        body = None
        if valid and self.poses:
            p = self.poses[-1]
            body = dict(stamp=p.stamp, position=p.position.tolist(), quaternion=p.quaternion.tolist())
        return dict(valid=valid, reason=reason, frame='odom', source_stamp=self.source_stamp,
                    resolution=self.cfg['resolution_m'], origin=self.map.origin.copy() if self.map else np.array([-self.cfg['map_size_m']/2]*2),
                    occupancy=occupancy, traversability=cost, elevation_points=elevation,
                    obstacle_points=obstacles, historical_elevation_points=np.empty((0, 3)),
                    body_pose=body, support_valid=support,
                    audit=dict(accepted_clouds=self.accepted, rejected_inputs=self.rejected,
                        pair_skew_s=self.pair_skew, relative_source_lag_s=dict(self.clock.lag),
                        zero_points_dropped=self.zero_points_dropped, nonfinite_points_dropped=self.nonfinite_points_dropped,
                        timestamp_alignment_verified=False, initial_transport_latency_verified=False,
                        body_calibration_verified=False, traversability_suitable_for_control=False,
                        floor_plane=self.floor.tolist() if self.floor is not None else None,
                        ground_history_s=self.cfg['map_lease_s'], motion_commands_sent=0))
