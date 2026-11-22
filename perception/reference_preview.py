"""Uncalibrated Point-LIO-reference rolling return grid, visualization only.

No ground inference, body transform, collision model, planning or motion API.
This deliberately does not instantiate TerrainMap or certify its traversability.
"""
from collections import deque
import math
import numpy as np
from perception.pipeline import MappingEngine, interpolate_pose


class ReferencePreviewEngine(MappingEngine):
    def __init__(self, settings, calibration, cfg):
        super().__init__(settings, calibration, cfg)
        self.preview = settings.get('reference_preview', {})
        self.resolution = float(self.preview.get('resolution_m', .12))
        self.window_m = float(self.preview.get('window_m', 6.))
        self.retention = float(self.preview.get('retention_s', 3.))
        self.point_limit = self.preview.get('max_retained_points', 60000)
        if not all(math.isfinite(x) and x > 0 for x in (self.resolution, self.window_m, self.retention)):
            raise ValueError('positive finite reference-preview dimensions required')
        self.n = int(round(self.window_m / self.resolution))
        if not 8 <= self.n <= 256 or not math.isclose(self.n*self.resolution, self.window_m, abs_tol=1e-6):
            raise ValueError('preview window must contain 8..256 whole cells per side')
        if type(self.point_limit) is not int or not 100 <= self.point_limit <= 200000:
            raise ValueError('invalid reference-preview point limit')
        self.frames = deque()
        self.reference_frame = None
        self.last_evidence = -math.inf

    def pose_input(self, stamp, position, quaternion, frame, child, wall, mono):
        if not isinstance(child, str) or not child or (self.reference_frame and child != self.reference_frame):
            self.reset_required = 'preview_reference_changed_or_missing_restart_required'
            self.pose_deadline = -math.inf
        ok = super().pose_input(stamp, position, quaternion, frame, child, wall, mono)
        if ok and not self.reference_frame:
            self.reference_frame = child
        return ok and not self.reset_required

    def cloud_input(self, stamp, points, frame, wall, mono):
        if not self.audit.observe('cloud', stamp, wall, mono, frame):
            self.cloud_deadline = -math.inf
            return False
        try:
            if self.reset_required:
                raise ValueError(self.reset_required)
            if frame != self.settings['fixed_frame']:
                raise ValueError('registered_cloud_frame_mismatch')
            points = np.asarray(points, float)
            if points.ndim != 2 or points.shape[1] != 3 or not 3 <= len(points) <= self.settings['max_cloud_points']:
                raise ValueError('invalid_or_oversize_cloud')
            finite = np.isfinite(points).all(axis=1)
            invalid = int((~finite).sum())
            row = self.audit.topics['cloud']
            row['nonfinite_points_dropped'] = row.get('nonfinite_points_dropped', 0) + invalid
            points = points[finite]
            if len(points) < 3:
                raise ValueError('insufficient_finite_cloud_points')
            pose, skew, pairing = interpolate_pose(self.poses, stamp, self.settings['pair_max_skew_s'])
            self.audit.pairs.append(skew)
            self.pairing = pairing
            evidence = mono-max(0., wall-stamp)
            if evidence <= self.last_evidence:
                raise ValueError('nonincreasing_evidence_time')
            # Source points are already in the world frame; never apply a body extrinsic.
            half = self.window_m/2+self.resolution
            points = points[np.all(np.abs(points[:, :2]-pose.position[:2]) <= half, axis=1)]
            if len(points) == 0:
                raise ValueError('no_returns_in_preview_window')
            if len(points) > self.point_limit:
                points = points[np.linspace(0, len(points)-1, self.point_limit, dtype=int)]
            self.frames.append((evidence, points.copy()))
            self.trim(mono)
            self.last_evidence = evidence
            self.cloud_stamp = stamp
            self.cloud_deadline = evidence+self.settings['max_source_age_s']
            self.accepted_clouds += 1
            self.last_error = None
            return True
        except (ValueError, TypeError) as exc:
            self.audit.reject('cloud', str(exc))
            self.last_error = str(exc)
            self.cloud_deadline = -math.inf
            return False

    def trim(self, mono):
        while self.frames and mono-self.frames[0][0] > self.retention:
            self.frames.popleft()
        count = sum(len(points) for _, points in self.frames)
        while count > self.point_limit and self.frames:
            count -= len(self.frames.popleft()[1])

    def snapshot(self, wall, mono):
        self.clock_check(wall)
        if self.reset_required:
            self.frames.clear()
        self.trim(mono)
        valid = bool(self.frames and not self.reset_required and
                     mono <= min(self.cloud_deadline, self.pose_deadline))
        pose = self.poses[-1] if self.poses else None
        centre = pose.position[:2] if pose is not None else np.zeros(2)
        origin = np.floor(centre/self.resolution)*self.resolution-self.window_m/2
        low = np.full(self.n*self.n, np.inf)
        high = np.full(self.n*self.n, -np.inf)
        points = np.empty((0, 3))
        if valid:
            points = np.concatenate([p for _, p in self.frames])
            cells = np.floor((points[:, :2]-origin)/self.resolution).astype(int)
            inside = np.all((cells >= 0) & (cells < self.n), axis=1)
            points, cells = points[inside], cells[inside]
            index = cells[:, 1]*self.n+cells[:, 0]
            np.minimum.at(low, index, points[:, 2])
            np.maximum.at(high, index, points[:, 2])
        observed = np.isfinite(low)
        indexes = np.flatnonzero(observed)
        xy = origin+(np.column_stack([indexes % self.n, indexes // self.n])+.5)*self.resolution
        minimum = np.column_stack([xy, low[observed]])
        maximum = np.column_stack([xy, high[observed]])
        reference = dict(stamp=pose.stamp, position=pose.position.tolist(), quaternion=pose.quaternion.tolist()) if pose is not None and valid else None
        audit = self.audit.report(mono)
        audit.update(calibration_issues=list(self.calibration_issues), accepted_clouds=self.accepted_clouds,
                     odometry_child_frames=sorted(self.child_frames), pairing_method=self.pairing,
                     reference_frame=self.reference_frame, preview_only=True, navigation_usable=False,
                     robot_geometry_used=False, ground_classification_performed=False,
                     retained_points=len(points), observed_cells=int(observed.sum()),
                     unknown_cells=self.n*self.n-int(observed.sum()), retention_s=self.retention,
                     motion_commands_sent=0, tf_chain_verified=False)
        # Compatibility keys allow the existing independent expiration watchdog to clear data.
        return dict(valid=valid, reason=self.reset_required or self.last_error or ('uncalibrated_reference_preview' if valid else 'inputs_expired'),
                    frame=self.settings['fixed_frame'], source_stamp=self.cloud_stamp, built_wall=wall, built_mono=mono,
                    valid_until_mono=min(self.cloud_deadline, self.pose_deadline), resolution=self.resolution, origin=origin,
                    occupancy=np.full((self.n, self.n), -1, dtype=np.int8),
                    elevation_points=minimum, obstacle_points=points, maximum_points=maximum,
                    reference_pose=reference, body_pose=None, support_valid=False, audit=audit)


def preview_messages(snapshot):
    """Only isolated visualization topics; no occupancy or Go2-body pose output."""
    import rospy
    from std_msgs.msg import Header
    from geometry_msgs.msg import Point
    from sensor_msgs import point_cloud2
    from visualization_msgs.msg import Marker
    header = Header()
    header.frame_id = snapshot['frame']
    header.stamp = rospy.Time.from_sec(snapshot['source_stamp'])
    valid = snapshot['valid']
    reference = snapshot['reference_pose'] if valid else None
    minimum = snapshot['elevation_points'] if valid else np.empty((0, 3))
    maximum = snapshot['maximum_points'] if valid else np.empty((0, 3))
    result = dict(points=point_cloud2.create_cloud_xyz32(header, snapshot['obstacle_points'].tolist() if valid else []),
                  lowest_returns=point_cloud2.create_cloud_xyz32(header, minimum.tolist()),
                  highest_returns=point_cloud2.create_cloud_xyz32(header, maximum.tolist()))
    boundary = Marker()
    boundary.header = header
    boundary.ns = 'uncalibrated_window'
    boundary.id = 0
    boundary.type = Marker.LINE_STRIP
    boundary.action = Marker.ADD
    boundary.pose.orientation.w = 1.
    boundary.scale.x = .035
    boundary.color.r = 1.; boundary.color.g = .8; boundary.color.b = .1; boundary.color.a = 1.
    x, y = snapshot['origin']
    side = snapshot['occupancy'].shape[0]*snapshot['resolution']
    z = reference['position'][2] if reference else 0.
    boundary.points = [Point(x=a, y=b, z=z) for a, b in [(x,y),(x+side,y),(x+side,y+side),(x,y+side),(x,y)]]
    result['window'] = boundary
    text = Marker()
    text.header = header; text.ns = 'uncalibrated_warning'; text.id = 0
    text.type = Marker.TEXT_VIEW_FACING; text.action = Marker.ADD
    text.pose.orientation.w = 1.
    text.pose.position.x = x+side/2; text.pose.position.y = y+side/2; text.pose.position.z = z+1.2
    text.scale.z = .23
    text.color.r = 1.; text.color.g = .8; text.color.b = .1; text.color.a = 1.
    text.text = 'UNCALIBRATED / NO CONTROL\n'+('LIVE: returns only, not ground' if valid else 'STALE / WAITING: points cleared')
    result['warning'] = text
    marker = Marker()
    marker.header = header; marker.ns = 'observed_cells_only'; marker.id = 0
    marker.type = Marker.CUBE_LIST; marker.action = Marker.ADD; marker.pose.orientation.w = 1.
    marker.scale.x = marker.scale.y = snapshot['resolution']*.9; marker.scale.z = .015
    marker.color.r = .2; marker.color.g = .8; marker.color.b = 1.; marker.color.a = .5
    marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in minimum]
    result['observed_cells'] = marker
    pose = Marker(); pose.header = header; pose.ns = 'pointlio_reference_not_body'; pose.id = 0
    pose.type = Marker.ARROW; pose.action = Marker.ADD if reference else Marker.DELETE
    pose.scale.x = .45; pose.scale.y = .07; pose.scale.z = .07
    pose.color.r = 1.; pose.color.g = .3; pose.color.b = .1; pose.color.a = 1.
    pose.pose.orientation.w = 1.
    if reference:
        pose.header.stamp = rospy.Time.from_sec(reference['stamp'])
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = reference['position']
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = reference['quaternion']
    result['reference_pose'] = pose
    return result
