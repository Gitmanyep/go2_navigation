"""Horizontal forward sector plus a short all-round return region.

This selects observed points, never certifies free space or robot clearance.
World coordinates and original samples are preserved, including their height.
"""
import math
import numpy as np


class SpatialROI:
    def __init__(self, settings=None, expected_frame=None):
        if settings is not None and not isinstance(settings, dict):
            raise ValueError('spatial_roi must be an object')
        self.config = dict(settings or {})
        self.enabled = bool(self.config)
        if not self.enabled:
            return
        required = {'frame', 'front_angle_deg', 'front_range_m', 'near_radius_m',
                    'fine_voxel_m', 'near_voxel_m'}
        if set(self.config) != required:
            raise ValueError('spatial_roi requires exactly: '+', '.join(sorted(required)))
        if self.config['frame'] not in ('body', 'pointlio_reference'):
            raise ValueError('unsupported spatial_roi frame')
        if expected_frame and self.config['frame'] != expected_frame:
            raise ValueError('spatial_roi frame does not match this mode: '+expected_frame)
        for key in required-{'frame'}:
            value = self.config[key]
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError('positive finite spatial_roi value required: '+key)
        if not 1 <= self.config['front_angle_deg'] <= 360:
            raise ValueError('spatial_roi angle outside 1..360 degrees')
        if not self.config['near_radius_m'] <= self.config['front_range_m'] <= 100:
            raise ValueError('spatial_roi requires near_radius <= front_range <= 100m')
        if not .005 <= self.config['fine_voxel_m'] <= self.config['near_voxel_m'] <= .25:
            raise ValueError('spatial_roi requires .005 <= fine_voxel <= near_voxel <= .25m')

    def masks(self, points, pose):
        """Azimuth follows reference +X projected on the horizontal world plane."""
        points = np.asarray(points, float).reshape(-1, 3)
        if not np.isfinite(points).all() or np.any(np.abs(points) > 1e6):
            raise ValueError('invalid spatial_roi world points')
        forward = pose.rotation[:2, 0]
        norm = np.linalg.norm(forward)
        if not np.isfinite(norm) or norm < 1e-6:
            raise ValueError('spatial_roi heading is vertical or invalid')
        forward = forward/norm
        delta = points[:, :2]-pose.position[:2]
        radius = np.linalg.norm(delta, axis=1)
        x = delta @ forward
        y = delta @ np.array([-forward[1], forward[0]])
        angle = np.abs(np.arctan2(y, x))
        fine = ((radius <= self.config['front_range_m']+1e-9) &
                (angle <= math.radians(self.config['front_angle_deg']/2)+1e-9))
        near = (radius <= self.config['near_radius_m']+1e-9) & ~fine
        return fine, near

    def coverage(self, points, pose):
        if not self.enabled:
            return np.ones(len(points), dtype=bool)
        fine, near = self.masks(points, pose)
        return fine | near

    @staticmethod
    def representatives(points, indexes, voxel):
        if not len(indexes):
            return indexes
        # A 3D voxel retains an actual return, not a centroid or a filled cell.
        cells = np.floor(points[indexes]/voxel).astype(np.int64)
        _, keep = np.unique(cells, axis=0, return_index=True)
        return indexes[keep]

    def select(self, points, pose, downsample=True):
        points = np.asarray(points, float).reshape(-1, 3)
        if not self.enabled:
            return points, dict(enabled=False)
        fine, near = self.masks(points, pose)
        f, n = np.flatnonzero(fine), np.flatnonzero(near)
        if downsample:
            f = self.representatives(points, f, self.config['fine_voxel_m'])
            n = self.representatives(points, n, self.config['near_voxel_m'])
        selected = np.sort(np.r_[f, n])
        return points[selected], dict(enabled=True, **self.config,
            input_points=len(points), fine_candidates=int(fine.sum()), near_candidates=int(near.sum()),
            fine_points=len(f), near_points=len(n), output_points=len(selected),
            outside_points=int((~(fine | near)).sum()),
            downsampled=downsample, cropped_regions_are_free=False,
            body_alignment_verified=self.config['frame']=='body',
            collision_clearance_verified=False)
