"""Flat-test-area obstacle-only PATH PREVIEW. No terrain analysis or actuation.

Unoccupied cells inside the declared area are ASSUMED usable for visualization;
the occupancy-only input cannot certify observed free space or ground support.
"""
from collections import deque
import heapq
import math
import time
import numpy as np
from scipy.ndimage import distance_transform_edt, binary_dilation, label
from scipy.spatial.transform import Rotation

class StreamLease:
    """Bound receipt age and later backlog without changing the source epoch."""
    def __init__(self, timeout):
        self.timeout=timeout; self.anchor=None; self.last=None
        self.deadline=-math.inf; self.restart_required=False
    def observe(self, stamp, frame, mono):
        if self.restart_required:
            raise ValueError('source_jump_restart_required')
        if not np.isfinite([stamp,mono]).all() or stamp<=0:
            self.invalidate(); raise ValueError('invalid_source_stamp')
        if self.anchor is None:self.anchor=(stamp,mono,frame)
        lag=self.anchor[0]+mono-self.anchor[1]-stamp
        if frame!=self.anchor[2] or lag<-.2 or (self.last is not None and stamp<self.last):
            self.restart_required=True; self.invalidate(); raise ValueError('source_jump_restart_required')
        if (self.last is not None and stamp==self.last) or lag>.6:
            self.invalidate(); raise ValueError('source_repeated_or_backlogged')
        self.last=stamp; self.deadline=mono+self.timeout-max(0.,lag)
    def invalidate(self, reason=None):self.deadline=-math.inf
    def valid(self, mono):return mono<=self.deadline



def astar(blocked, start, goal, budget_s):
    h, w = blocked.shape
    def free(c):
        x, y = c
        return 0 <= x < w and 0 <= y < h and not blocked[y, x]
    if not free(start):
        return [], 'start_in_collision_or_outside_test_area'
    if not free(goal):
        return [], 'goal_in_collision_or_outside_test_area'
    q, scores, parent = [(math.dist(start, goal), 0., start)], {start: 0.}, {}
    deadline = time.monotonic()+budget_s
    while q:
        if time.monotonic() > deadline:
            return [], 'planning_timeout'
        _, cost, node = heapq.heappop(q)
        if cost != scores[node]:
            continue
        if node == goal:
            route = [node]
            while route[-1] != start:
                route.append(parent[route[-1]])
            return route[::-1], 'obstacle_only_path_preview'
        x, y = node
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            child = (x+dx, y+dy)
            # Both side cells must be free: a diagonal must not cut a corner.
            if not (free(child) and free((x+dx,y)) and free((x,y+dy))):
                continue
            score = cost+math.hypot(dx, dy)
            if score < scores.get(child, math.inf):
                scores[child], parent[child] = score, node
                heapq.heappush(q, (score+math.dist(child,goal), score, child))
    return [], 'no_collision_free_route'


def reachable_direction_goal(grid, start, origin, resolution, position, axis, horizon, budget_s):
    """Pick an observed forward cell in the start's four-connected free region."""
    h,w=grid.shape;x,y=start
    if not (0<=x<w and 0<=y<h and grid[y,x]==0):
        return [],None,'start_in_collision_or_outside_test_area'
    started=time.monotonic()
    components,_=label(grid==0)
    yy,xx=np.nonzero(components==components[y,x])
    dx=origin[0]+(xx+.5)*resolution-position[0]
    dy=origin[1]+(yy+.5)*resolution-position[1]
    progress=dx*axis[0]+dy*axis[1]
    lateral=np.abs(dx*axis[1]-dy*axis[0])
    mask=(progress>=.3)&(progress<=horizon)&(np.hypot(dx,dy)<=horizon)&(lateral<=np.maximum(.5,progress*.5))
    if not np.any(mask):return [],None,'no_reachable_forward_goal'
    indices=np.flatnonzero(mask);i=indices[np.argmax(progress[mask]-lateral[mask])]
    goal=(int(xx[i]),int(yy[i]))
    remaining=budget_s-(time.monotonic()-started)
    if remaining<=0:return [],None,'planning_timeout'
    route,reason=astar(grid!=0,start,goal,remaining)
    return route,goal if route else None,reason


def route_diagnostics(grid, start, goal, origin, resolution):
    """Explain a failed search; relaxed searches are diagnostic only, never commands."""
    h,w=grid.shape
    def inside(c):return 0<=c[0]<w and 0<=c[1]<h
    names={-1:'unknown',0:'assumed_usable',50:'clearance',100:'obstacle'}
    def kind(c):return names.get(int(grid[c[1],c[0]]),'invalid') if inside(c) else 'outside'
    reachable=set();todo=deque([start] if inside(start) and grid[start[1],start[0]]==0 else [])
    frontier={};nearest=None
    while todo:
        c=todo.popleft()
        if c in reachable:continue
        reachable.add(c)
        if nearest is None or math.dist(c,goal)<math.dist(nearest,goal):nearest=c
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            n=(c[0]+dx,c[1]+dy)
            if inside(n) and grid[n[1],n[0]]==0:
                if n not in reachable:todo.append(n)
            else:frontier[n]=kind(n)
    counts={name:sum(v==name for v in frontier.values()) for name in ('unknown','clearance','obstacle','outside')}
    samples=sorted(frontier,key=lambda c:math.dist(c,goal))[:8]
    result=dict(start_type=kind(start),goal_type=kind(goal),reachable_cells=len(reachable),frontier_counts=counts,
        nearest_reachable_to_goal_m=math.dist(nearest,goal)*resolution if nearest is not None else None,
        blocking_samples=[dict(world_xy=(origin+(np.asarray(c)+.5)*resolution).tolist(),kind=frontier[c]) for c in samples])
    if inside(start) and inside(goal):
        for name,mask in [('allow_unknown', (grid!=0)&(grid!=-1)),('allow_clearance',(grid!=0)&(grid!=50)),('obstacles_only',grid==100)]:
            path,reason=astar(mask,start,goal,.08)
            result[name]=dict(route_exists=bool(path),reason=reason)
    return result


def settings_check(s):
    for k in ('grid_resolution_m', 'preview_radius_m', 'robot_radius_m', 'collision_margin_m',
              'robot_envelope_height_m', 'floor_exclusion_m', 'voxel_size_m', 'input_timeout_s',
              'pose_timeout_s', 'max_pair_skew_s', 'planning_budget_s', 'goal_distance_m',
              'candidate_max_speed_mps', 'candidate_max_yaw_rps', 'direction_timeout_s'):
        if type(s.get(k)) not in (float, int) or not math.isfinite(s[k]) or s[k] <= 0:
            raise ValueError('invalid_setting:'+k)
    if not .05 <= s['grid_resolution_m'] <= .25 or not 1 <= s['preview_radius_m'] <= 8:
        raise ValueError('unsupported_preview_grid')
    if s['floor_exclusion_m'] >= s['robot_envelope_height_m'] or s['robot_radius_m'] >= s['preview_radius_m']:
        raise ValueError('invalid_preview_geometry')
    if s.get('footprint_shape')=='rectangle':
        for k in ('robot_length_m','robot_width_m'):
            if type(s.get(k)) not in (int,float) or not math.isfinite(s[k]) or not 0<s[k]<2:
                raise ValueError('invalid_rectangle_geometry:'+k)
    # Keep the direction target inside the usable window, away from boundary inflation.
    boundary_margin=s['collision_margin_m']+(s['voxel_size_m']+s['grid_resolution_m'])*math.sqrt(.5)
    body_reach=(math.hypot(s['robot_length_m']/2,s['robot_width_m']/2)
                if s.get('footprint_shape')=='rectangle' else s['robot_radius_m'])
    if s['goal_distance_m']+body_reach+boundary_margin+s['grid_resolution_m'] >= s['preview_radius_m']:
        raise ValueError('direction_horizon_exceeds_usable_window')
    if s['fixed_frame'] != 'odom' or s['body_frame'] != 'base_link':
        raise ValueError('native_frame_contract_required')
    if s.get('floor_z_m') is not None and not math.isfinite(s['floor_z_m']):
        raise ValueError('invalid_fixed_floor')
    return s


class ObstaclePreview:
    def __init__(self, settings):
        self.s = settings_check(dict(settings))
        self.voxel_clock = StreamLease(timeout=self.s['input_timeout_s'])
        self.pose_clock = StreamLease(timeout=self.s['pose_timeout_s'])
        self.pose = None
        self.floor_z = self.s.get('floor_z_m')
        self.area_centre = None
        self.goal = None
        self.direction = None
        self.source_stamp = 0.
        self.obstacles = set()
        self.floor_pose = None
        self.last_error = None
        self.restart_required = False
        self.voxel_size = self.s['voxel_size_m']
        self.voxel_size_verified = False
        self.received_points = self.filtered_floor_points = self.out_of_envelope_points = 0
        self.compute_samples = deque(maxlen=200)
        self.coverage = set()

    def set_floor(self, z, frame, centre=None):
        if frame != self.s['fixed_frame'] or not math.isfinite(z):
            raise ValueError('floor_frame_or_value_invalid')
        xy = self.pose['position'][:2] if self.pose else centre
        if xy is None or np.asarray(xy).shape != (2,) or not np.isfinite(xy).all():
            raise ValueError('waiting_for_odometry_to_declare_test_area')
        self.floor_z = float(z)
        self.floor_pose = self.pose['position'].copy() if self.pose else None
        self.area_centre = np.asarray(xy, float).copy()
        self.goal = self.direction = None
        self.obstacles.clear()
        self.voxel_clock.invalidate()
        self.last_error = 'waiting_for_voxels_after_area_reset'

    def set_goal(self, xy, frame):
        xy = np.asarray(xy, float)
        if frame != self.s['fixed_frame'] or xy.shape != (2,) or not np.isfinite(xy).all():
            raise ValueError('goal_frame_or_value_invalid')
        self.goal, self.direction = xy.copy(), None

    def set_direction(self, vector, receipt_mono):
        v = np.asarray(vector, float)
        if v.shape != (3,) or not np.isfinite(v).all() or abs(v[2]) > 1e-6:
            raise ValueError('horizontal_body_direction_required')
        if np.linalg.norm(v[:2]) < 1e-6:
            if self.direction is not None:
                self.goal = self.direction = None
            return
        self.goal = None
        self.direction = (v[:2]/np.linalg.norm(v[:2]), receipt_mono+self.s['direction_timeout_s'])

    def pose_input(self, stamp, position, quaternion, frame, child, mono):
        try:
            p, q = np.asarray(position, float), np.asarray(quaternion, float)
            if frame != self.s['fixed_frame'] or child != self.s['body_frame']:
                raise ValueError('odometry_frame_mismatch')
            if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1.) > .01:
                raise ValueError('invalid_odometry')
            self.pose_clock.observe(stamp, frame, mono)
            jump = self.pose is not None and np.linalg.norm(p-self.pose['position']) > 1.
            moved = self.floor_pose is not None and np.linalg.norm(p-self.floor_pose) > .25
            if jump or moved:
                self.obstacles.clear(); self.coverage.clear()
                if self.s.get('reset_map_on_pose_change',False):
                    self.area_centre=p[:2].copy();self.floor_pose=p.copy()
                    self.voxel_clock.invalidate()
                else:
                    self.restart_required=True
                    raise ValueError('pose_jump_restart_required')
            self.pose = dict(stamp=stamp, position=p.copy(), quaternion=q.copy())
            if self.floor_z is not None and self.area_centre is None:
                self.area_centre = p[:2].copy()
                self.floor_pose = p.copy()
            return True
        except ValueError as exc:
            self.pose_clock.invalidate(str(exc)); self.last_error = str(exc)
            return False

    def cloud_input(self, stamp, points, frame, mono, voxel_size=None):
        try:
            if frame != self.s['fixed_frame']:
                raise ValueError('voxel_frame_mismatch')
            a = np.asarray(points, float)
            if a.ndim != 2 or a.shape[1] != 3 or len(a) > 1000000:
                raise ValueError('invalid_voxel_array')
            if not np.isfinite(a).all() or np.any(np.abs(a) > 1e5):
                raise ValueError('nonfinite_or_unbounded_voxel')
            if voxel_size is not None:
                if not math.isfinite(voxel_size) or not .005 <= voxel_size <= 1:
                    raise ValueError('invalid_voxel_size')
                # A size change changes the swept volume; keep the larger size
                # for old obstacle cells too until an explicit test-area reset.
                if not self.voxel_size_verified and not self.obstacles:
                    self.voxel_size = float(voxel_size)
                    self.voxel_size_verified = True
                else:
                    self.voxel_size = max(self.voxel_size, float(voxel_size))
            self.voxel_clock.observe(stamp, frame, mono)
            self.source_stamp = stamp
            self.received_points = len(a)
            if self.floor_z is None or self.area_centre is None:
                self.last_error = 'click_floor_to_declare_flat_test_area'
                return False
            half = self.voxel_size*.5
            low, high = self.floor_z+self.s['floor_exclusion_m'], self.floor_z+self.s['robot_envelope_height_m']
            self.filtered_floor_points = int(np.sum(a[:,2]+half < low))
            self.out_of_envelope_points = int(np.sum(a[:,2]-half > high))
            use = (a[:,2]+half >= low) & (a[:,2]-half <= high)
            # This is a human-declared empty footprint in a flat, static demo.
            # It does not prove that the masked volume is collision-free.
            mask_radius = self.s.get('preview_assumed_self_radius_m', 0.)
            if not math.isfinite(mask_radius) or not 0 <= mask_radius <= .6:
                raise ValueError('invalid_preview_self_mask')
            if self.pose is not None and mask_radius:
                use &= np.linalg.norm(a[:,:2]-self.pose['position'][:2],axis=1) > mask_radius
            use &= np.linalg.norm(a[:,:2]-self.area_centre, axis=1) <= self.s['preview_radius_m']+self.s['robot_radius_m']+self.voxel_size
            cells = np.floor(a[use,:2]/self.s['grid_resolution_m']).astype(int)
            # A missing occupied voxel does not prove the volume was cleared.
            # Retain obstacles for this declared flat test until the user resets.
            if self.s.get('use_latest_voxel_snapshot',False):
                self.obstacles = set(map(tuple,cells))
                covered = a[np.linalg.norm(a[:,:2]-self.area_centre,axis=1)<=self.s['preview_radius_m'],:2]
                self.coverage = set(map(tuple,np.floor(covered/self.s['grid_resolution_m']).astype(int)))
            else:
                self.obstacles.update(map(tuple, cells))
            self.last_error = None
            return True
        except ValueError as exc:
            self.voxel_clock.invalidate(str(exc)); self.last_error = str(exc)
            return False

    def snapshot(self, mono):
        started = time.monotonic()
        res, radius = self.s['grid_resolution_m'], self.s['preview_radius_m']
        n = 2*math.ceil(radius/res)+1
        pos = self.pose['position'] if self.pose else np.zeros(3)
        origin = np.floor(pos[:2]/res)*res-(n//2)*res
        grid = np.full((n,n), -1, np.int8)
        out = dict(valid=False, reason=self.last_error or 'waiting_for_native_inputs', frame=self.s['fixed_frame'],
                   source_stamp=self.source_stamp, origin=origin, resolution=res, grid=grid,
                   path=[], candidate=[0.,0.], goal=None, floor_z=self.floor_z,
                   pose=self.pose, obstacle_cells=len(self.obstacles), assumption='flat_test_area_unoccupied_is_assumed_usable',
                   terrain_analysis_enabled=False, free_space_verified=False, traversability_suitable_for_control=False,
                   assumed_self_mask_radius_m=self.s.get('preview_assumed_self_radius_m',0.),
                   motion_commands_sent=0, start_assumed_safe=self.s.get('start_assumed_safe',False),
                   footprint_shape=self.s.get('footprint_shape','circle'), body_reference_verified=False,
                   goal_distance_m=self.s['goal_distance_m'], map_radius_m=radius,
                   candidate_limits=dict(speed_mps=self.s['candidate_max_speed_mps'],yaw_rps=self.s['candidate_max_yaw_rps']))
        if self.restart_required or self.voxel_clock.restart_required or self.pose_clock.restart_required:
            out['reason'] = 'source_or_pose_jump_restart_required'; return out
        if not self.pose_clock.valid(mono) or not self.voxel_clock.valid(mono):
            out['reason'] = self.last_error or 'native_input_timeout'; return out
        if self.floor_z is None or self.area_centre is None:
            out['reason'] = 'click_floor_to_declare_flat_test_area'; return out
        if abs(self.source_stamp-self.pose['stamp']) > self.s['max_pair_skew_s']:
            out['reason'] = 'voxel_odometry_time_mismatch'; return out
        yy, xx = np.indices(grid.shape)
        xy = origin+np.stack((xx+.5,yy+.5),axis=-1)*res
        inside = np.linalg.norm(xy-self.area_centre,axis=-1) <= radius-res*math.sqrt(.5)
        obstacle = np.zeros_like(inside)
        if self.obstacles:
            world_cells = np.asarray(list(self.obstacles), float)
            points = (world_cells+.5)*res
            c = np.floor((points-origin)/res).astype(int)
            keep = (c>=0).all(axis=1) & (c<n).all(axis=1)
            obstacle[c[keep,1], c[keep,0]] = True
        occupied_or_outside = obstacle | ~inside
        clearance = distance_transform_edt(np.pad(~occupied_or_outside,1))[1:-1,1:-1]*res
        margin = self.s['robot_radius_m']+self.s['collision_margin_m']+(self.voxel_size+res)*math.sqrt(.5)
        blocked = clearance <= margin
        if self.s.get('footprint_shape')=='rectangle':
            yaw=Rotation.from_quat(self.pose['quaternion']).as_euler('xyz')[2]
            extra=self.s['collision_margin_m']+(self.voxel_size+res)*math.sqrt(.5)
            length=self.s['robot_length_m']/2+extra; width=self.s['robot_width_m']/2+extra
            k=int(math.ceil(math.hypot(length,width)/res))
            ky,kx=np.mgrid[-k:k+1,-k:k+1]
            bx=(kx*math.cos(yaw)+ky*math.sin(yaw))*res
            by=(-kx*math.sin(yaw)+ky*math.cos(yaw))*res
            kernel=(abs(bx)<=length)&(abs(by)<=width)
            blocked=binary_dilation(occupied_or_outside,structure=kernel,border_value=1)
        grid[inside] = np.where(obstacle[inside],100,np.where(blocked[inside],50,0)).astype(np.int8)
        if self.s.get('use_latest_voxel_snapshot',False):
            observed=np.zeros_like(inside)
            if self.coverage:
                cc=np.floor(((np.asarray(list(self.coverage))+.5)*res-origin)/res).astype(int)
                ok=(cc>=0).all(axis=1)&(cc<n).all(axis=1)
                observed[cc[ok,1],cc[ok,0]]=True
            grid[inside & ~observed & ~blocked]=-1
        if self.s.get('start_assumed_safe',False):
            if self.s.get('footprint_shape')=='rectangle':
                yaw=Rotation.from_quat(self.pose['quaternion']).as_euler('xyz')[2]
                delta=xy-pos[:2]
                bx=delta[:,:,0]*math.cos(yaw)+delta[:,:,1]*math.sin(yaw)
                by=-delta[:,:,0]*math.sin(yaw)+delta[:,:,1]*math.cos(yaw)
                declared=(abs(bx)<=self.s['robot_length_m']/2)&(abs(by)<=self.s['robot_width_m']/2)
                grid[declared & (grid==-1)]=0
            start_cell=np.floor((pos[:2]-origin)/res).astype(int)
            if (start_cell>=0).all() and (start_cell<n).all():grid[start_cell[1],start_cell[0]]=0
        out.update(valid=True, reason='waiting_for_goal', grid=grid,
                   assumed_usable_cells=int(np.sum(grid==0)), blocked_cells=int(np.sum(grid>=50)), observed_obstacle_cells=int(np.sum(grid==100)), inflated_cells=int(np.sum(grid==50)),
                   footprint_inflation_m=margin)
        target = self.goal
        if self.direction is not None:
            vector, expiry = self.direction
            if mono > expiry:
                out['reason'] = 'direction_expired'; return out
            axis = Rotation.from_quat(self.pose['quaternion']).as_matrix()[:2,:2]@vector
            norm = np.linalg.norm(axis)
            if norm < .1:
                out['reason'] = 'invalid_body_heading'; return out
            target = pos[:2]+axis/norm*self.s['goal_distance_m']
        if target is None:
            return out
        out['goal'] = target.tolist()
        start, goal = [tuple(np.floor((p-origin)/res).astype(int)) for p in (pos[:2],target)]
        route, reason = astar((grid!=0), start, goal, self.s['planning_budget_s'])
        if self.direction is not None:
            out['desired_goal'] = target.tolist()
            out['goal_selection'] = 'desired'
            if not route and reason != 'planning_timeout':
                route, local_goal, reason = reachable_direction_goal(
                    grid,start,origin,res,pos[:2],axis/norm,self.s['goal_distance_m'],self.s['planning_budget_s'])
                if local_goal is not None:
                    goal=local_goal
                    out['goal']=(origin+(np.asarray(goal)+.5)*res).tolist()
                    out['goal_selection']='reachable_local'
                else:
                    out['goal']=None
                    out['goal_selection']='none'
        if route:
            path = [list(origin+(np.asarray(c)+.5)*res)+[self.floor_z+self.voxel_size*.5+.04] for c in route]
            out['path'] = path
            if len(path)>1:
                delta = np.asarray(path[1][:2])-pos[:2]
                axis = Rotation.from_quat(self.pose['quaternion']).as_matrix()[:2,0]
                error = math.atan2(delta[1],delta[0])-math.atan2(axis[1],axis[0])
                error = math.atan2(math.sin(error),math.cos(error))
                out['candidate'] = [self.s['candidate_max_speed_mps']*math.cos(error) if abs(error)<.7 else 0.,
                                    float(np.clip(1.6*error,-self.s['candidate_max_yaw_rps'],self.s['candidate_max_yaw_rps']))]
        out['reason'] = reason
        if not route:out['route_diagnostic']=route_diagnostics(grid,start,goal,origin,res)
        self.compute_samples.append((time.monotonic()-started)*1000)
        out['planning_p95_ms'] = float(np.quantile(self.compute_samples,.95))
        return out
