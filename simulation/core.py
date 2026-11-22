"""Navigation-only core: timestamped direction -> 2.5D terrain -> A* -> simulated velocity.
Coordinates: x forward, y left, z up. No robot SDK, DDS, sockets or actuation.
"""
from dataclasses import dataclass
from collections import deque
from pathlib import Path
import heapq
import json
import math
import time
import numpy as np
from scipy import ndimage
from geometry import Geometry
from acoustic_task import AcousticTask


def config(path=None):
    cfg = json.loads(Path(path or Path(__file__).with_name('config.json')).read_text())
    positive = ['resolution_m','map_size_m','footprint_radius_m','max_speed_mps',
                'max_yaw_rate_rps','source_max_age_s','source_max_skew_s','map_lease_s',
                'direction_max_age_s','goal_distance_m','replan_period_s','max_pitch_deg',
                'max_roll_deg','max_step_m','max_roughness_m','simulation_dt_s',
                'braking_deceleration_mps2','simulation_response_tau_s']
    for key in positive:
        if not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError('invalid_configuration:' + key)
    if cfg['max_pitch_deg'] >= 45 or cfg['max_roll_deg'] >= 45:
        raise ValueError('unsupported_slope_configuration')
    for key in ['heading_filter_tau_s','fusion_window_s','body_reference_height_m','support_height_tolerance_m','minimum_cell_samples','minimum_support_points','task_settle_s','obstacle_hold_s','task_observation_gap_s','route_position_tolerance_m','route_yaw_tolerance_rad','planner_budget_s','candidate_lease_s']:
        if not math.isfinite(cfg[key]) or cfg[key]<=0:raise ValueError('invalid_configuration:'+key)
    return cfg


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def fresh(stamp, now, cfg, max_age=None):
    age = now - float(stamp)
    if not math.isfinite(age):
        raise ValueError('nonfinite_timestamp')
    if age < -cfg['future_tolerance_s']:
        raise ValueError('future_source')
    if age > (cfg['source_max_age_s'] if max_age is None else max_age):
        raise ValueError('stale_source')
    return max(0., age)


@dataclass
class Pose:
    stamp: float
    position: np.ndarray
    quaternion: np.ndarray
    frame: str = 'camera_init'

    def validate(self):
        self.position = np.asarray(self.position, float)
        self.quaternion = np.asarray(self.quaternion, float)
        if (not math.isfinite(self.stamp) or self.position.shape != (3,) or
            self.quaternion.shape != (4,) or not np.isfinite(self.position).all() or
            not np.isfinite(self.quaternion).all() or
            abs(np.linalg.norm(self.quaternion) - 1.) > .01 or not isinstance(self.frame,str) or not self.frame):
            raise ValueError('invalid_pose')
        return self

    @property
    def rotation(self):
        x, y, z, w = self.quaternion / np.linalg.norm(self.quaternion)
        return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                         [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                         [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

    @property
    def yaw(self):
        r = self.rotation
        return math.atan2(r[1,0], r[0,0])


class DirectionManager:
    """Hold a world-frame subgoal; update bearing without resetting it on every packet."""
    def __init__(self, cfg, mic_rotation=None):
        self.cfg = cfg
        self.seq = -1
        self.stamp = -math.inf
        self.deadline = -math.inf
        self.goal = None
        self.heading = None
        self.commit_heading = None
        self.commit_time = -math.inf
        self.goal_origin=None
        self.updates = 0
        self.reason = 'waiting_for_direction'
        self.session_id = None
        self.retired_sessions = set()
        self.source_id = None
        self.last_filter_stamp = None
        self.session_deadline=-math.inf
        self.mic_rotation = None
        if mic_rotation is not None:
            r = np.asarray(mic_rotation, float)
            if r.shape != (3,3) or not np.isfinite(r).all() or not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(r), 1., atol=1e-5):
                raise ValueError('invalid_mic_rotation')
            self.mic_rotation = r

    def invalidate(self, reason):
        self.goal = None
        self.heading = None
        self.deadline = -math.inf
        self.reason = reason

    def accept(self, message, pose, now, mono):
        try:
            seq = message['seq']
            session=message.get('session_id','legacy')
            source=message.get('source_id','default')
            if not isinstance(session,str) or not session or not isinstance(source,str) or not source:
                raise ValueError('invalid_source_or_session')
            if session in self.retired_sessions:
                raise ValueError('retired_direction_session')
            new_session=self.session_id is not None and session!=self.session_id
            if new_session and mono<self.session_deadline:
                raise ValueError('session_change_requires_expired_previous_session')
            if type(seq) is not int or seq<0 or (not new_session and seq<=self.seq):
                raise ValueError('replayed_direction')
            if message.get('kind') != 'direction' or message.get('valid') is not True:
                raise ValueError('invalid_direction')
            stamp = float(message['stamp_sec'])
            age = fresh(stamp, now, self.cfg, self.cfg['direction_max_age_s'])
            if stamp <= self.stamp:
                raise ValueError('reordered_direction')
            confidence = float(message['confidence'])
            if not math.isfinite(confidence) or not self.cfg['min_confidence'] <= confidence <= 1:
                raise ValueError('low_confidence')
            pose.validate()
            fresh(pose.stamp, now, self.cfg)
            if abs(pose.stamp - stamp) > self.cfg['source_max_skew_s']:
                raise ValueError('direction_pose_skew')
            v = np.asarray(message['vector'], float)
            if v.shape != (3,) or not np.isfinite(v).all() or np.linalg.norm(v) < 1e-8:
                raise ValueError('invalid_vector')
            v = v / np.linalg.norm(v)
            if message['frame_id'] == 'mic_array':
                if self.mic_rotation is None:
                    raise ValueError('uncalibrated_mic_frame')
                v = self.mic_rotation @ v
            elif message['frame_id'] != 'body':
                raise ValueError('invalid_direction_frame')
            world = pose.rotation @ v
            if np.linalg.norm(world[:2]) < .1:
                raise ValueError('vertical_direction')
            angle = math.atan2(world[1], world[0])
            if new_session:
                self.retired_sessions.add(self.session_id)
                self.invalidate('new_source_session')
                self.last_filter_stamp=None
            if self.source_id is not None and source!=self.source_id:
                self.invalidate('source_identity_changed')
                self.last_filter_stamp=None
            elapsed=0. if self.last_filter_stamp is None else max(0.,stamp-self.last_filter_stamp)
            alpha=1.-math.exp(-elapsed/self.cfg['heading_filter_tau_s'])
            self.heading = angle if self.heading is None else wrap(self.heading + alpha*wrap(angle-self.heading))
            self.last_filter_stamp=stamp
            self.session_id,self.source_id=session,source
            reached = self.goal is not None and np.linalg.norm(self.goal[:2]-pose.position[:2]) < self.cfg['goal_tolerance_m']
            changed = self.commit_heading is not None and abs(wrap(self.heading-self.commit_heading)) > math.radians(self.cfg['heading_hysteresis_deg'])
            passed_anchor=self.goal_origin is not None and self.commit_heading is not None and np.dot(pose.position[:2]-self.goal_origin,np.array([math.cos(self.commit_heading),math.sin(self.commit_heading)]))>=.8*self.cfg['goal_distance_m']
            if self.goal is None or reached or passed_anchor or (changed and mono-self.commit_time > .6):
                self.goal = pose.position.copy()
                self.goal[:2] += self.cfg['goal_distance_m'] * np.array([math.cos(self.heading),math.sin(self.heading)])
                self.goal_origin=pose.position[:2].copy()
                self.commit_heading, self.commit_time = self.heading, mono
                self.updates += 1
            self.seq, self.stamp = seq, stamp
            self.deadline = mono + self.cfg['direction_max_age_s'] - age
            self.session_deadline=self.deadline
            self.reason = 'direction_valid'
            return True
        except (ValueError, TypeError, KeyError, OverflowError) as e:
            self.invalidate(str(e))
            return False

    def valid(self, now, mono):
        try:
            fresh(self.stamp, now, self.cfg, self.cfg['direction_max_age_s'])
            if self.goal is None or mono >= self.deadline:
                raise ValueError('direction_expired')
            return True
        except ValueError as e:
            self.invalidate(str(e))
            return False


class TerrainMap:
    """Rolling elevation grid. Only real point support marks cells observed.

    Each cell stores the lowest return, vertical extent and source-derived age.
    Local gradients represent a single ground surface; multi-level surfaces are rejected.
    Missing returns remain unknown, never inferred free from a ray over a hole.
    """
    def __init__(self, cfg, centre):
        self.cfg = cfg
        self.res = cfg['resolution_m']
        self.n = int(round(cfg['map_size_m']/self.res))
        self.origin = np.floor(np.asarray(centre[:2])/self.res)*self.res-self.n*self.res/2
        self.elevation = np.full((self.n,self.n), np.nan)
        self.extent = np.full_like(self.elevation, np.nan)
        self.seen = np.full_like(self.elevation, -math.inf)
        self.obstacle_seen = self.seen.copy()
        self.overhead_seen = self.seen.copy()
        self.obstacle_cleared = self.seen.copy()
        self.revision=0
        self._analysis=None
        self._analysis_until=-math.inf
        self.obstacle_low=np.full_like(self.elevation,np.inf)
        self.obstacle_high=np.full_like(self.elevation,-np.inf)
        self.frames=deque()
        self.obstacle_frames={}
        self.support_valid=False
        self.support_reason='support_not_observed'
        self.geometry=Geometry(cfg)

    def cell(self, xy):
        q = np.floor((np.asarray(xy[:2])-self.origin)/self.res).astype(int)
        return int(q[0]), int(q[1])

    def world(self, c):
        return self.origin+(np.asarray(c)+.5)*self.res

    def inside(self, c):
        return 0 <= c[0] < self.n and 0 <= c[1] < self.n

    def recenter(self, position):
        if np.max(np.abs(np.asarray(position[:2])-(self.origin+self.n*self.res/2))) < self.n*self.res*.22:
            return
        new_origin = np.floor(np.asarray(position[:2])/self.res)*self.res-self.n*self.res/2
        shift = np.rint((self.origin-new_origin)/self.res).astype(int)
        for name, fill in [('elevation',np.nan),('extent',np.nan),('seen',-math.inf),('obstacle_seen',-math.inf),('overhead_seen',-math.inf),('obstacle_cleared',-math.inf),('obstacle_low',np.inf),('obstacle_high',-np.inf)]:
            old = getattr(self,name)
            new = np.full_like(old,fill)
            dx,dy = shift
            xs,xe = max(0,dx),min(self.n,self.n+dx)
            ys,ye = max(0,dy),min(self.n,self.n+dy)
            if xe > xs and ye > ys:
                new[ys:ye,xs:xe] = old[ys-dy:ye-dy,xs-dx:xe-dx]
            setattr(self,name,new)
        self.origin = new_origin
        self.revision+=1;self._analysis=None

    def integrate(self, points, position, evidence_mono, rotation=None):
        self.recenter(position)
        self.revision+=1;self._analysis=None
        position=np.asarray(position,float)
        if position.shape==(2,):position=np.r_[position,self.cfg['body_reference_height_m']]
        rotation=np.eye(3) if rotation is None else rotation
        if self.frames and evidence_mono<=self.frames[-1][0]:
            raise ValueError('replayed_fusion_frame')
        self.frames.append((evidence_mono,np.asarray(points,float).copy()))
        while self.frames and evidence_mono-self.frames[0][0]>self.cfg['fusion_window_s']:
            self.frames.popleft()
        merged=np.concatenate([frame[1] for frame in self.frames])
        times=np.concatenate([np.full(len(frame[1]),frame[0]) for frame in self.frames])
        point_ids=np.concatenate([np.arange(len(frame[1])) for frame in self.frames])
        # A support surface must agree with the body datum, not just be planar.
        expected=position-rotation[:,2]*self.cfg['body_reference_height_m']
        delta=merged-expected
        local=(np.linalg.norm(merged[:,:2]-position[:2],axis=1)<=self.cfg['support_seed_radius_m'])
        seed=local&(np.abs(delta@rotation[:,2])<=self.cfg['support_height_tolerance_m'])
        self.support_valid=int(seed.sum())>=self.cfg['minimum_support_points']
        if not self.support_valid:
            self.support_reason='ground_body_height_inconsistent_or_unobserved'
            return
        floor=merged[seed]
        # Refine a local plane, robustly clipping clutter against the body datum.
        design=np.column_stack([floor[:,:2]-position[:2],np.ones(len(floor))])
        coeff=np.linalg.lstsq(design,floor[:,2],rcond=None)[0]
        for _ in range(2):
            keep=np.abs(design@coeff-floor[:,2])<self.cfg['terrain_ground_band_m']
            if keep.sum()<self.cfg['minimum_support_points']:
                self.support_valid=False;self.support_reason='unsupported_ground_plane';return
            coeff=np.linalg.lstsq(design[keep],floor[keep,2],rcond=None)[0]
        self.support_reason='support_plane_observed'
        c=np.floor((merged[:,:2]-self.origin)/self.res).astype(int)
        use=(c[:,0]>=0)&(c[:,0]<self.n)&(c[:,1]>=0)&(c[:,1]<self.n)
        points,z,ts,c,point_ids=merged[use],merged[use,2],times[use],c[use],point_ids[use]
        index=c[:,1]*self.n+c[:,0];size=self.n*self.n
        low=np.full(size,np.inf);high=np.full(size,-np.inf);count=np.zeros(size,int)
        oldest=np.full(size,np.inf)
        np.minimum.at(low,index,z);np.maximum.at(high,index,z)
        np.add.at(count,index,1);np.minimum.at(oldest,index,ts)
        support=count>=self.cfg['minimum_cell_samples']
        yy,xx=np.indices((self.n,self.n))
        xy=self.origin+(np.stack([xx,yy],axis=-1)+.5)*self.res
        predicted=(xy-position[:2])@coeff[:2]+coeff[2]
        reach=np.linalg.norm(xy-position[:2],axis=-1)*math.tan(math.radians(self.cfg['max_pitch_deg']))
        eligible=support.reshape(self.n,self.n)&(np.abs(low.reshape(self.n,self.n)-predicted)<=self.cfg['support_height_tolerance_m']+reach)
        seed_cells=eligible&(np.linalg.norm(xy-position[:2],axis=-1)<self.cfg['support_seed_radius_m'])&(np.abs(low.reshape(self.n,self.n)-predicted)<self.cfg['support_height_tolerance_m'])
        # A disconnected roof/table cannot become ground solely from its flatness.
        connected=np.zeros_like(eligible)
        queue=deque(zip(*np.where(seed_cells)))
        connected[seed_cells]=True
        lows=low.reshape(self.n,self.n)
        while queue:
            y,x=queue.popleft()
            for dy,dx in [(-1,0),(1,0),(0,-1),(0,1)]:
                ny,nx=y+dy,x+dx
                if 0<=ny<self.n and 0<=nx<self.n and eligible[ny,nx] and not connected[ny,nx] and abs(lows[ny,nx]-lows[y,x])<=self.cfg['max_step_m']:
                    connected[ny,nx]=True;queue.append((ny,nx))
        # Conservative age: support cannot be kept alive by one new sample plus old data.
        capture=oldest.reshape(self.n,self.n)
        self.elevation[connected]=lows[connected]
        self.extent[connected]=(high-low).reshape(self.n,self.n)[connected]
        self.seen[connected]=capture[connected]
        supported_height=lows.ravel()[index]
        relative=z-supported_height
        raw_nonground=relative>self.cfg['terrain_ground_band_m']
        nonground=raw_nonground&(ts>self.obstacle_cleared.ravel()[index])
        overlow=np.full(size,np.inf);overhigh=np.full(size,-np.inf)
        np.minimum.at(overlow,index[nonground],relative[nonground])
        np.maximum.at(overhigh,index[nonground],relative[nonground])
        overlow,overhigh=overlow.reshape(self.n,self.n),overhigh.reshape(self.n,self.n)
        # Obstacle evidence is independent of ground fusion. Store absolute heights;
        # floor-only returns cannot prove the formerly occupied volume is empty.
        obstacle_times=np.full(size,-np.inf)
        np.maximum.at(obstacle_times,index[nonground],ts[nonground])
        observed=connected&np.isfinite(overlow)
        abs_low=np.full_like(lows,np.inf);abs_high=np.full_like(lows,-np.inf)
        abs_low[observed]=overlow[observed]+lows[observed]
        abs_high[observed]=overhigh[observed]+lows[observed]
        self.obstacle_low[observed]=np.minimum(self.obstacle_low[observed],abs_low[observed])
        self.obstacle_high[observed]=np.maximum(self.obstacle_high[observed],abs_high[observed])
        self.overhead_seen[observed]=obstacle_times.reshape(self.n,self.n)[observed]
        obstacle=observed&(overlow<=self.cfg['body_reference_height_m']+.15)
        self.obstacle_seen[obstacle]=self.overhead_seen[obstacle]
        # Retain actual 3D returns for full-attitude swept checks too. The 2.5D
        # heading masks alone do not capture a pitched head frame on a slope.
        recognized=nonground&connected.ravel()[index]
        # A sparse first-frame branch may only be recognized once later ground
        # samples establish support. Backfill its ORIGINAL frame and point IDs.
        # Per-frame masks accumulate monotonically without duplicating points or
        # renewing source ages when old points are reclassified.
        for stamp,raw in self.frames:
            selected=recognized&(ts==stamp)
            if not selected.any():continue
            if stamp not in self.obstacle_frames:
                self.obstacle_frames[stamp]=(raw,np.zeros(len(raw),bool))
            self.obstacle_frames[stamp][1][point_ids[selected]]=True
        for stamp in list(self.obstacle_frames):
            if evidence_mono-stamp>self.cfg['obstacle_hold_s']:del self.obstacle_frames[stamp]
        retained=[]
        for stamp,(raw,mask) in self.obstacle_frames.items():
            cloud=raw[mask]
            c=np.floor((cloud[:,:2]-self.origin)/self.res).astype(int)
            inside=(c[:,0]>=0)&(c[:,0]<self.n)&(c[:,1]>=0)&(c[:,1]<self.n)
            cloud,c=cloud[inside],c[inside]
            retained.append(cloud[stamp>self.obstacle_cleared[c[:,1],c[:,0]]])
        self.recent_points=np.concatenate([points[~raw_nonground|nonground]]+retained)

    def clear_obstacle_volume(self, cells, z_min, z_max, evidence_mono, *, certified=False):
        """Explicit trusted free-volume evidence, NOT inferred from missing cloud returns.

        Caller must certify visibility/coverage over the full stored absolute height
        interval. No ROS point-cloud callback calls this API without such evidence.
        """
        if not certified or not np.isfinite([z_min,z_max,evidence_mono]).all() or z_min>=z_max:
            raise ValueError('uncertified_free_volume')
        for x,y in cells:
            if not self.inside((x,y)):raise ValueError('free_volume_outside_map')
            if evidence_mono<=max(self.overhead_seen[y,x],self.obstacle_cleared[y,x]):raise ValueError('stale_free_volume')
            if z_min>self.obstacle_low[y,x] or z_max<self.obstacle_high[y,x]:
                raise ValueError('incomplete_free_volume')
        for x,y in cells:
            self.obstacle_cleared[y,x]=evidence_mono
            self.obstacle_low[y,x]=np.inf;self.obstacle_high[y,x]=-np.inf
            self.overhead_seen[y,x]=self.obstacle_seen[y,x]=-math.inf
        if hasattr(self,'recent_points'):
            c=np.floor((self.recent_points[:,:2]-self.origin)/self.res).astype(int)
            cleared=np.zeros(len(c),bool)
            for x,y in cells:
                cleared|=(c[:,0]==x)&(c[:,1]==y)&(self.recent_points[:,2]>=z_min)&(self.recent_points[:,2]<=z_max)
            self.recent_points=self.recent_points[~cleared]
        self.revision+=1;self._analysis=None

    def analyze(self, mono):
        # Valid until the next evidence-age boundary; duplicate ROS/UI reads reuse
        # the same immutable-by-convention arrays, never refresh the evidence age.
        if self._analysis is not None and self._analysis_at<=mono<self._analysis_until:
            return self._analysis
        expired_obstacle=np.isfinite(self.overhead_seen)&(mono-self.overhead_seen>self.cfg['obstacle_hold_s'])
        known = (mono-self.seen <= self.cfg['map_lease_s']) & np.isfinite(self.elevation)&~expired_obstacle
        occupied = mono-self.obstacle_seen <= self.cfg['obstacle_hold_s']
        # Nearest filling is only for numerical derivatives; it never changes known.
        if known.any():
            indices = ndimage.distance_transform_edt(~known, return_distances=False, return_indices=True)
            z = self.elevation[tuple(indices)]
        else:
            z = np.zeros_like(self.elevation)
        smooth = ndimage.median_filter(z,size=3)
        gy,gx = np.gradient(smooth,self.res)
        rough = np.abs(z-smooth)
        step = np.zeros_like(z)
        for axis in [0,1]:
            diff = np.abs(np.diff(z,axis=axis))
            if axis == 0:
                pair = known[1:]&known[:-1]
                diff = np.where(pair,diff,0)
                step[1:] = np.maximum(step[1:],diff)
                step[:-1] = np.maximum(step[:-1],diff)
            else:
                pair = known[:,1:]&known[:,:-1]
                diff = np.where(pair,diff,0)
                step[:,1:] = np.maximum(step[:,1:],diff)
                step[:,:-1] = np.maximum(step[:,:-1],diff)
        slope = np.degrees(np.arctan(np.hypot(gx,gy)))
        bad = (~known)|occupied|(slope>self.cfg['max_pitch_deg'])|(step>self.cfg['max_step_m'])|(rough>self.cfg['max_roughness_m'])
        # Pad boundary as unknown and include cell-corner margin in inflation.
        padded = np.pad(~bad,1,constant_values=False)
        clearance = ndimage.distance_transform_edt(padded)[1:-1,1:-1]*self.res
        blocked = clearance <= self.cfg['footprint_radius_m']+self.res*math.sqrt(.5)
        grid = np.where(~known,-1,np.where(bad,100,0)).astype(np.int8)
        result=dict(known=known,grid=grid,blocked=blocked,elevation=z,gx=gx,gy=gy,
                    slope=slope,roughness=rough,step=step,clearance=clearance,
                    obstacle_low=np.where(known,self.obstacle_low-self.elevation,np.inf),
                    obstacle_high=np.where(known,self.obstacle_high-self.elevation,-np.inf),
                    support_valid=self.support_valid,support_reason=self.support_reason)
        if self.cfg['heading_planner_enabled']:
            result['heading_masks']=self.geometry.heading_masks(result,self.res)
        result['obstacle_unknown']=expired_obstacle
        self._analysis=result;self._analysis_at=mono
        expiries=np.r_[self.seen.ravel()+self.cfg['map_lease_s'],
                       self.overhead_seen.ravel()+self.cfg['obstacle_hold_s']]
        future=expiries[expiries>=mono]
        self._analysis_until=np.nextafter(future.min(),math.inf) if len(future) else math.inf
        return result


def directional_slope(terrain, cell, heading):
    x,y = cell
    gx,gy = terrain['gx'][y,x],terrain['gy'][y,x]
    return (math.degrees(math.atan(gx*math.cos(heading)+gy*math.sin(heading))),
            math.degrees(math.atan(-gx*math.sin(heading)+gy*math.cos(heading))))


def astar(terrain, start, goal, cfg):
    blocked = terrain['blocked']
    h,w = blocked.shape
    def valid(c):
        return 0 <= c[0] < w and 0 <= c[1] < h and not blocked[c[1],c[0]]
    if not valid(start) or not valid(goal):
        return []
    frontier = [(math.dist(start,goal),0.,start)]
    costs,parents = {start:0.},{}
    while frontier:
        _,g,c = heapq.heappop(frontier)
        if g > costs[c]+1e-8:
            continue
        if c == goal:
            route = [c]
            while route[-1] != start:
                route.append(parents[route[-1]])
            return route[::-1]
        for dx,dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            nxt = (c[0]+dx,c[1]+dy)
            if not valid(nxt) or (dx and dy and (not valid((c[0]+dx,c[1])) or not valid((c[0],c[1]+dy)))):
                continue
            angle = math.atan2(dy,dx)
            if any(abs(p)>cfg['max_pitch_deg'] or abs(r)>cfg['max_roll_deg']
                   for p,r in [directional_slope(terrain,c,angle),directional_slope(terrain,nxt,angle)]):
                continue
            x,y = nxt
            cost = g+math.hypot(dx,dy)*(1+.6*terrain['slope'][y,x]/cfg['max_pitch_deg']+.08/max(.1,terrain['clearance'][y,x]))
            if cost < costs.get(nxt,math.inf):
                costs[nxt],parents[nxt] = cost,c
                heapq.heappush(frontier,(cost+math.dist(nxt,goal),cost,nxt))
    return []


def integrate_twist(xy, yaw, v, w, dt):
    if abs(w) < 1e-8:
        return np.asarray(xy)+v*dt*np.array([math.cos(yaw),math.sin(yaw)]),wrap(yaw+w*dt)
    return np.asarray(xy)+v/w*np.array([math.sin(yaw+w*dt)-math.sin(yaw),math.cos(yaw)-math.cos(yaw+w*dt)]),wrap(yaw+w*dt)


def segment_clear(a, b, terrain_map, blocked):
    """Visit every crossed grid interval, including sub-centimetre corner clips."""
    a,b = np.asarray(a),np.asarray(b)
    delta = b-a
    ts = [0.,1.]
    for axis in [0,1]:
        if abs(delta[axis]) < 1e-12:
            continue
        lower,upper = sorted([(a[axis]-terrain_map.origin[axis])/terrain_map.res,
                              (b[axis]-terrain_map.origin[axis])/terrain_map.res])
        for boundary in range(math.floor(lower)+1,math.ceil(upper)):
            t = (terrain_map.origin[axis]+boundary*terrain_map.res-a[axis])/delta[axis]
            if 0 < t < 1:
                ts.append(t)
    ts = sorted(ts)
    samples = [0.,1.]+[(l+r)/2 for l,r in zip(ts,ts[1:])]
    for t in samples:
        c = terrain_map.cell(a+t*delta)
        if not terrain_map.inside(c) or blocked[c[1],c[0]]:
            return False
    return True


class Navigator:
    def __init__(self, cfg=None, mic_rotation=None):
        self.cfg = config() if cfg is None else cfg
        self.direction = DirectionManager(self.cfg,mic_rotation)
        self.task=AcousticTask(self.cfg)
        self.geometry=Geometry(self.cfg)
        self.visited={}
        self.recovery_since=None
        self.exploring=False
        self.intermediate_goal=None
        self.recovery_anchor=None
        self.exploration_started=None
        self.exploration_exhausted=False
        self.poses = deque(maxlen=300)
        self.pose = None
        self.map = None
        self.cloud_stamp = -math.inf
        self.cloud_deadline = -math.inf
        self.pose_deadline = -math.inf
        self.fault = 'waiting_for_sensors'
        self.route = []
        self.route_headings=[]
        self.route_cursor=0
        self.last_plan = -math.inf
        self.planned_goal = None
        self.last_velocity = 0.
        self.terrain = None

    def pose_input(self, pose, now, mono):
        try:
            pose.validate()
            age = fresh(pose.stamp,now,self.cfg)
            if self.pose is not None and (pose.stamp <= self.pose.stamp or pose.frame != self.pose.frame):
                raise ValueError('pose_reordered_or_frame_changed')
            self.pose = pose
            self.task.observe_pose(pose,mono)
            key=tuple(np.floor(pose.position[:2]/.3).astype(int))
            self.visited[key]=self.visited.get(key,0)+1
            self.poses.append(pose)
            self.pose_deadline = mono+self.cfg['source_max_age_s']-age
            return True
        except (ValueError,TypeError,OverflowError) as e:
            self.fault = str(e)
            self.pose_deadline = -math.inf
            self.task.invalidate_observation()
            return False

    def cloud_input(self, points, stamp, frame, now, mono):
        try:
            age = fresh(stamp,now,self.cfg)
            if stamp <= self.cloud_stamp:
                raise ValueError('replayed_cloud')
            points = np.asarray(points,float)
            if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3 or not np.isfinite(points).all():
                raise ValueError('invalid_cloud')
            if not self.poses:
                raise ValueError('waiting_for_pose')
            pose = min(self.poses,key=lambda p:abs(p.stamp-stamp))
            if pose.frame != frame or abs(pose.stamp-stamp)>self.cfg['source_max_skew_s']:
                raise ValueError('cloud_pose_mismatch')
            age = max(age,fresh(pose.stamp,now,self.cfg))
            if self.map is None:
                self.map = TerrainMap(self.cfg,pose.position)
            self.map.integrate(points,pose.position,mono-age,pose.rotation)
            self.cloud_stamp = stamp
            self.cloud_deadline = mono+self.cfg['source_max_age_s']-age
            self.fault = None
            return True
        except (ValueError,TypeError,OverflowError) as e:
            self.fault = str(e)
            self.cloud_deadline = -math.inf
            return False

    def direction_input(self, message, now, mono):
        try:
            if not self.poses:
                raise ValueError('waiting_for_pose')
            stamp = float(message['stamp_sec'])
            pose = min(self.poses,key=lambda p:abs(p.stamp-stamp))
            old_source=self.direction.source_id
            accepted=self.direction.accept(message,pose,now,mono)
            if accepted and old_source is not None and old_source!=self.direction.source_id:
                self.task.transition('STOP_LISTEN',mono,'source_identity_changed')
            return accepted
        except (ValueError,KeyError,TypeError,OverflowError) as e:
            self.direction.invalidate(str(e))
            return False

    def plan(self,terrain,start,goal):
        if self.cfg['heading_planner_enabled']:
            from heading_planner import heading_astar
            return heading_astar(terrain,start,goal,self.pose.yaw,self.cfg)
        return astar(terrain,start,goal,self.cfg)

    def rotation_pending(self):
        if not self.route_headings or not self.route:return False
        i=self.route_cursor
        if i>=len(self.route):return False
        if i==0:return False
        return np.linalg.norm(self.route[i]-self.route[i-1])<1e-8

    def track_route(self):
        """Consume ordered (position, heading) states; never jump via nearest XY.

        Consecutive same-position nodes are individual signed 45-degree turns.
        Replanning waits until such a turn has completed.
        """
        tol=self.cfg['route_position_tolerance_m'];yaw_tol=self.cfg['route_yaw_tolerance_rad']
        while self.route_cursor<len(self.route):
            i=self.route_cursor;target=self.route[i];heading=self.route_headings[i]
            delta=target-self.pose.position[:2];distance=float(np.linalg.norm(delta))
            if i==0:
                # Initial lattice state is a cell/yaw quantization anchor, not an
                # extra physical turn. Re-aligning here at every replan causes
                # stop-turn-stop oscillations even on a straight route.
                self.route_cursor+=1;continue
            rotation=np.linalg.norm(target-self.route[i-1])<1e-8
            error=wrap(heading-self.pose.yaw)
            if rotation:
                if distance<self.map.res*.76 and abs(error)<yaw_tol:
                    self.route_cursor+=1;continue
                signed=wrap(heading-self.route_headings[i-1])
                if signed*error<0 and abs(error)>math.pi/2:
                    error+=math.copysign(2*math.pi,signed)
                return 0.,float(np.clip(1.6*error,-self.cfg['max_yaw_rate_rps'],self.cfg['max_yaw_rate_rps'])),'rotate'
            along=np.array([math.cos(heading),math.sin(heading)])
            relative=self.pose.position[:2]-target
            reached=(np.dot(relative,along)>=-tol and abs(relative[0]*along[1]-relative[1]*along[0])<self.map.res*.51)
            if reached:
                self.route_cursor+=1;continue
            # Look ahead only within this uninterrupted straight-heading segment.
            # The final translation before a repeated-XY turn is never skipped.
            aim=target
            for j in range(i,len(self.route)):
                if j>i and (np.linalg.norm(self.route[j]-self.route[j-1])<1e-8 or abs(wrap(self.route_headings[j]-heading))>1e-8):break
                point=self.route[j]
                if self.terrain is not None and not segment_clear(self.pose.position[:2],point,self.map,self.terrain['blocked']):break
                aim=point
                if np.linalg.norm(point-self.pose.position[:2])>=.4:break
            diff=aim-self.pose.position[:2];remaining=float(np.linalg.norm(diff))
            error=wrap(math.atan2(diff[1],diff[0])-self.pose.yaw)
            w=float(np.clip(1.6*error,-self.cfg['max_yaw_rate_rps'],self.cfg['max_yaw_rate_rps']))
            v=min(self.cfg['max_speed_mps'],1.2*remaining)*max(0.,math.cos(error)) if abs(error)<.7 else 0.
            return v,w,'translate'
        return 0.,0.,'route_finished'

    def step(self, now, mono, dt=.1):
        result = dict(reason='waiting',candidate=[0.,0.],path=[],goal=None,task=self.task.status(),
                      source_age_s=None,pitch_deg=None,roll_deg=None,
                      executable=False,motion_commands_sent=0)
        # Task continuity is updated even when the cloud/map cannot be used.
        observations_fresh=(self.pose is not None and self.map is not None and not self.fault and
                            mono<min(self.pose_deadline,self.cloud_deadline))
        if observations_fresh:
            try:
                fresh(self.pose.stamp,now,self.cfg)
                fresh(self.cloud_stamp,now,self.cfg)
            except ValueError:observations_fresh=False
        direction_ok=self.direction.valid(now,mono)
        may_move=self.task.tick(mono,direction_ok,observations_fresh)
        result['task']=self.task.status()
        try:
            if self.pose is None or self.map is None:
                raise ValueError('waiting_for_sensors')
            fresh(self.pose.stamp,now,self.cfg)
            result['source_age_s'] = fresh(self.cloud_stamp,now,self.cfg)
            if self.fault:
                raise ValueError(self.fault)
            if mono >= min(self.cloud_deadline,self.pose_deadline):
                raise ValueError('sensor_deadline_expired')
            if not may_move:
                raise ValueError('task_'+self.task.state.lower())
            if not direction_ok:
                raise ValueError(self.direction.reason)
            terrain = self.map.analyze(mono)
            self.terrain = terrain
            if not self.map.support_valid:
                raise ValueError(self.map.support_reason)
            start = self.map.cell(self.pose.position)
            if not self.map.inside(start) or terrain['blocked'][start[1],start[0]]:
                raise ValueError('nearfield_ground_unobserved' if self.map.inside(start) and terrain['grid'][start[1],start[0]]<0 else 'footprint_blocked_or_unknown')
            if self.geometry.collision(self.map.recent_points,self.pose.position,self.pose.rotation):
                raise ValueError('body_or_head_frame_collision')
            pitch,roll = directional_slope(terrain,start,self.pose.yaw)
            result.update(pitch_deg=pitch,roll_deg=roll)
            if abs(pitch)>self.cfg['max_pitch_deg'] or abs(roll)>self.cfg['max_roll_deg']:
                raise ValueError('body_slope_limit')
            goal = self.direction.goal
            result['goal'] = goal.tolist()
            if self.exploration_exhausted:
                raise ValueError('bounded_exploration_exhausted')
            if self.exploring and self.exploration_started is not None and mono-self.exploration_started>=self.cfg['exploration_total_timeout_s']:
                self.exploration_exhausted=True
                raise ValueError('bounded_exploration_exhausted')
            changed = self.planned_goal is None or np.linalg.norm(goal-self.planned_goal)>.01
            if (mono-self.last_plan >= self.cfg['replan_period_s'] or changed) and not self.rotation_pending():
                terrain.pop('_plan_budget_exhausted',None)
                terrain['_plan_deadline']=time.perf_counter()+self.cfg['planner_budget_s']
                cells = self.plan(terrain,start,self.map.cell(goal))
                if not cells and self.intermediate_goal is not None:
                    if np.linalg.norm(self.pose.position[:2]-self.intermediate_goal)>.22:
                        cells=self.plan(terrain,start,self.map.cell(self.intermediate_goal))
                    else:
                        key=tuple(np.floor(self.intermediate_goal/.3).astype(int))
                        for dx in range(-1,2):
                            for dy in range(-1,2):
                                self.visited[(key[0]+dx,key[1]+dy)]=max(80,self.visited.get((key[0]+dx,key[1]+dy),0))
                        self.intermediate_goal=None
                if not cells:
                    # A direction is not a known destination. Advance to an observed
                    # reachable frontier when the anchored point is occluded.
                    yy,xx = np.where(~terrain['blocked'])
                    coords = self.map.origin+(np.column_stack([xx,yy])+.5)*self.map.res
                    remaining = np.linalg.norm(coords-goal[:2],axis=1)
                    progress = np.linalg.norm(goal[:2]-self.pose.position[:2])-remaining
                    distance = np.linalg.norm(coords-self.pose.position[:2],axis=1)
                    if self.recovery_since is None or self.recovery_anchor is None:
                        self.recovery_since=mono;self.recovery_anchor=self.pose.position[:2].copy()
                    if np.linalg.norm(self.pose.position[:2]-self.recovery_anchor)>self.map.res:
                        self.recovery_since=mono;self.recovery_anchor=self.pose.position[:2].copy()
                    self.exploring=self.exploring or (self.cfg['exploration_enabled'] and mono-self.recovery_since>2.)
                    if self.exploring and self.exploration_started is None:self.exploration_started=mono
                    if self.exploring and (mono-self.recovery_since>self.cfg['exploration_timeout_s'] or mono-self.exploration_started>self.cfg['exploration_total_timeout_s']):
                        self.exploration_exhausted=True
                        raise ValueError('bounded_exploration_exhausted')
                    permitted=progress> (-1.5 if self.exploring else .25)
                    candidate_clearance=terrain['clearance'][yy,xx]
                    robust=candidate_clearance>self.cfg['footprint_radius_m']+2*self.map.res
                    candidates = np.where(permitted&robust&(distance>.5)&(distance<self.cfg['goal_distance_m']+.8))[0]
                    visits=np.array([self.visited.get(tuple(np.floor(xy/.3).astype(int)),0) for xy in coords])
                    ranking=remaining+.06*np.minimum(visits,100)+.15/np.maximum(.05,candidate_clearance-self.cfg['footprint_radius_m'])
                    # Spatially subsample endpoints so a wall does not exhaust the budget.
                    chosen = []
                    for j in candidates[np.argsort(ranking[candidates])]:
                        if all(np.linalg.norm(coords[j]-coords[k])>.3 for k in chosen):
                            chosen.append(j)
                        if len(chosen)>=8:
                            break
                    if chosen and self.cfg['heading_planner_enabled']:
                        from heading_planner import heading_astar_many
                        goals=[(int(xx[j]),int(yy[j])) for j in chosen]
                        cells=heading_astar_many(terrain,start,goals,self.pose.yaw,self.cfg)
                        if cells:self.intermediate_goal=self.map.world(cells[-1])
                    else:
                        for j in chosen:
                            cells=self.plan(terrain,start,(int(xx[j]),int(yy[j])))
                            if cells:
                                self.intermediate_goal=coords[j].copy();break
                if cells and self.map.cell(goal)==cells[-1]:
                    self.recovery_since=None;self.exploring=False;self.intermediate_goal=None;self.recovery_anchor=None;self.exploration_started=None
                self.route_headings=[state[2]*math.pi/4 for state in getattr(cells,"states",[])]
                self.route = [self.map.world(c) for c in cells]
                self.route_cursor=0
                self.planned_goal = goal.copy()
                self.last_plan = mono
            if not self.route:
                raise ValueError('planning_budget_exhausted' if terrain.get('_plan_budget_exhausted') else 'no_traversable_route')
            if self.route_headings:
                v,w,tracking_mode=self.track_route()
                result['route_cursor']=self.route_cursor
                result['tracking_mode']=tracking_mode
            else:
                # Compatibility for the optional XY-only planner.
                target=self.route[min(self.route_cursor+1,len(self.route)-1)]
                error=wrap(math.atan2(*(target-self.pose.position[:2])[::-1])-self.pose.yaw)
                w=float(np.clip(1.6*error,-self.cfg['max_yaw_rate_rps'],self.cfg['max_yaw_rate_rps']))
                v=self.cfg['max_speed_mps']*max(0.,math.cos(error)) if abs(error)<.7 else 0.
            v *= max(.25,1-.6*abs(pitch)/self.cfg['max_pitch_deg']-.4*abs(roll)/self.cfg['max_roll_deg'])
            # Clearance also bounds source-delay + response + braking distance.
            clearance = terrain['clearance'][start[1],start[0]]-self.cfg['footprint_radius_m']-self.map.res*math.sqrt(.5)
            b = self.cfg['braking_deceleration_mps2']
            reaction = result['source_age_s']+self.cfg['simulation_response_tau_s']
            v = min(v,max(0.,-b*reaction+math.sqrt((b*reaction)**2+2*b*max(0.,clearance))))
            v = min(v,self.last_velocity+self.cfg['acceleration_mps2']*dt)
            horizon = max(self.cfg['prediction_horizon_s'],reaction+v/b)
            uy,ux=np.where(~terrain['known'])
            unknown_xy=self.map.origin+(np.column_stack([ux,uy])+.5)*self.map.res
            unknown_xy=unknown_xy[np.linalg.norm(unknown_xy-self.pose.position[:2],axis=1)<1.3]
            def sweep_clear(speed,rate):
                previous = self.pose.position[:2]
                for t in np.linspace(0,horizon,max(12,int(horizon*20))):
                    xy,a = integrate_twist(self.pose.position[:2],self.pose.yaw,speed,rate,t)
                    cell = self.map.cell(xy)
                    if not segment_clear(previous,xy,self.map,terrain['blocked']):
                        return False
                    previous = xy
                    base=np.array([*xy,terrain['elevation'][cell[1],cell[0]]+self.cfg['body_reference_height_m']])
                    yaw_delta=a-self.pose.yaw
                    rz=np.array([[math.cos(yaw_delta),-math.sin(yaw_delta),0],[math.sin(yaw_delta),math.cos(yaw_delta),0],[0,0,1]])
                    if self.geometry.footprint_unknown(unknown_xy,xy,a,self.map.res):
                        return False
                    if self.geometry.collision(self.map.recent_points,base,rz@self.pose.rotation):
                        return False
                    p,r = directional_slope(terrain,cell,a)
                    if abs(p)>self.cfg['max_pitch_deg'] or abs(r)>self.cfg['max_roll_deg']:
                        return False
                return True
            if not sweep_clear(v,w):
                # Turn in place before attempting the bend. A circular footprint
                # covers the full rotation; attitude is checked throughout it.
                v = 0.
                if not sweep_clear(v,w):
                    raise ValueError('predicted_sweep_blocked')
            result.update(reason='exploring' if self.exploring else 'tracking',candidate=[float(v),float(w)],path=[p.tolist() for p in self.route])
            self.last_velocity = v
        except (ValueError,TypeError,OverflowError) as e:
            if self.pose is None or mono>=self.pose_deadline:
                self.task.tick(mono,False,False)
            result['task']=self.task.status()
            result['reason'] = str(e)
            self.last_velocity = 0.
            # Failed plans respect the replan period too; retain heading states on
            # a sweep rejection so the next tick cannot silently reverse the turn.
            if str(e)!='predicted_sweep_blocked':
                self.route=[];self.route_headings=[];self.route_cursor=0
            if str(e) not in ('no_traversable_route','planning_budget_exhausted','predicted_sweep_blocked'):
                self.last_plan=-math.inf
        return result
