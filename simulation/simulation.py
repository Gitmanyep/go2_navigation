"""Repeatable closed-loop navigation simulator; ground truth is isolated from the planner."""
from collections import Counter,deque
from pathlib import Path
import argparse
import json
import math
import time
import numpy as np
from scipy import ndimage
from core import Navigator,Pose,config,integrate_twist,wrap

SCENARIOS = ['flat_detour','gentle_slope','cross_slope','narrow_passage',
             'goal_jitter','sensor_delay','cloud_dropout','odom_dropout',
             'direction_dropout','backlog_20s','steep_slope','unsafe_cross_slope',
             'too_narrow','dropoff','dead_end','sparse_lidar','blind_zone','elevated_plane',
             'overhead_branch','intermittent_branch','high_beam','u_obstacle','manual_confirmation','localization_drift','random_obstacles']


def quaternion(roll,pitch,yaw):
    # ZYX orientation for body on a height field.
    cr,sr = math.cos(roll/2),math.sin(roll/2)
    cp,sp = math.cos(pitch/2),math.sin(pitch/2)
    cy,sy = math.cos(yaw/2),math.sin(yaw/2)
    return np.array([sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,
                     cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy])


class World:
    def __init__(self,name,cfg):
        self.name,self.cfg = name,cfg
        self.res = cfg['resolution_m']
        self.origin = np.array([-1.5,-3.3])
        self.nx,self.ny = 50,44
        if name=='u_obstacle':
            self.origin=np.array([-3.,-3.3]);self.nx=60
        yy,xx = np.indices((self.ny,self.nx))
        self.xy = self.origin+np.stack([xx+.5,yy+.5],axis=-1)*self.res
        x,y = self.xy[:,:,0],self.xy[:,:,1]
        self.z = np.zeros_like(x)
        self.obstacle = np.zeros_like(x,bool)
        self.ground = np.ones_like(x,bool)
        self.goal = np.array([4.2,0.])
        self.source_position=np.array([4.2,0.,2.3])
        self.overhang_points=np.empty((0,3))
        if name=='manual_confirmation':
            self.goal=np.array([1.,0.]);self.source_position[:2]=self.goal
        if name in ['overhead_branch','intermittent_branch','high_beam']:
            self.goal=np.array([3.,0.]);self.source_position[:2]=self.goal
            beam_z=1.4 if name=='high_beam' else .85
            by,bx=np.mgrid[-3.3:3.31:.06,1.20:1.36:.06]
            self.overhang_points=np.column_stack([bx.ravel(),by.ravel(),np.full(bx.size,beam_z)])
        if name=='elevated_plane':self.z[:]=2.
        if name=='u_obstacle':
            self.obstacle|=((x>-.8)&(x<2.)&(np.abs(np.abs(y)-1.0)<.13))|((x>1.8)&(x<2.1)&(np.abs(y)<1.1))
        if name=='random_obstacles':
            layout_rng=np.random.RandomState(cfg.get('layout_seed',21))
            for ox in [1.2,2.6]:
                oy=layout_rng.uniform(-.55,.55)
                self.obstacle|=(np.abs(x-ox)<.20)&(np.abs(y-oy)<.32)
        if name in ['flat_detour','goal_jitter','sensor_delay','cloud_dropout','odom_dropout','direction_dropout','backlog_20s']:
            self.obstacle |= (x>.9)&(x<1.35)&(np.abs(y)<.38)
        if name in ['gentle_slope','steep_slope']:
            angle = 8 if name=='gentle_slope' else 23
            self.z = np.clip(x-.5,0,3)*math.tan(math.radians(angle))
        if name in ['cross_slope','unsafe_cross_slope']:
            angle = 6 if name=='cross_slope' else 13
            self.z = y*math.tan(math.radians(angle))
        if name in ['narrow_passage','too_narrow']:
            half_width = .78 if name=='narrow_passage' else .43
            self.obstacle |= (x>.8)&(x<3.2)&(np.abs(y)>half_width)
        if name=='dropoff':
            self.ground &= ~((x>1.2)&(x<2.1))
        if name=='dead_end':
            self.obstacle |= ((x>.8)&(x<3.0)&(np.abs(y)>.8))|((x>2.6)&(np.abs(y)<1.))
        padded = np.pad(~self.obstacle & self.ground,1,constant_values=False)
        self.clearance = ndimage.distance_transform_edt(padded)[1:-1,1:-1]*self.res
        self.gy,self.gx = np.gradient(self.z,self.res)

    def cell(self,xy):
        q = np.floor((np.asarray(xy)-self.origin)/self.res).astype(int)
        return int(q[0]),int(q[1])

    def inside(self,c):
        return 0<=c[0]<self.nx and 0<=c[1]<self.ny

    def height_angles(self,xy,yaw):
        c = self.cell(xy)
        if not self.inside(c):
            return 0.,0.,0.
        x,y = c
        gx,gy = self.gx[y,x],self.gy[y,x]
        pitch = -math.atan(gx*math.cos(yaw)+gy*math.sin(yaw))
        roll = math.atan(-gx*math.sin(yaw)+gy*math.cos(yaw))
        return self.z[y,x],pitch,roll

    def valid_footprint(self,xy):
        c = self.cell(xy)
        return self.inside(c) and self.clearance[c[1],c[0]]>self.cfg['footprint_radius_m']+self.res*math.sqrt(.5)

    def cloud(self,xy,rng,yaw=0.,frame_index=0):
        # Back-mounted external lidar origin: distinct from the body reference.
        body_xy=np.asarray(xy).copy()
        mount=np.asarray(self.cfg['lidar_position_body_m'])
        xy=np.asarray(xy)+np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])@mount[:2]
        # Grid-supported point samples are an idealized registered LiDAR, not Livox optics.
        endpoints = self.xy.reshape(-1,2)
        delta = endpoints-np.asarray(xy)
        radius=np.linalg.norm(delta,axis=1)
        blind=.5 if self.name=='blind_zone' else self.cfg['near_blind_radius_m']
        selected=(radius<self.cfg['sensor_range_m'])&(radius>=blind)
        ids = np.where(selected)[0]
        d = delta[ids]
        visible = np.ones(len(ids),bool)
        endpoint_cells = np.floor((endpoints[ids]-self.origin)/self.res).astype(int)
        # Occlusion checks stop before endpoint, so wall surfaces themselves remain visible.
        for fraction in np.linspace(0,.96,25):
            ray = np.asarray(xy)+fraction*d
            cells = np.floor((ray-self.origin)/self.res).astype(int)
            inside = (cells[:,0]>=0)&(cells[:,0]<self.nx)&(cells[:,1]>=0)&(cells[:,1]<self.ny)
            obstructed = np.ones(len(ids),bool)
            obstructed[inside] = self.obstacle[cells[inside,1],cells[inside,0]]
            obstructed &= np.any(cells != endpoint_cells,axis=1)
            visible &= ~obstructed
        ids = ids[visible]
        centres = endpoints[ids]
        zz = self.z.reshape(-1)[ids]
        wall = self.obstacle.reshape(-1)[ids]
        ground = self.ground.reshape(-1)[ids]
        points = []
        gx,gy = self.gx.reshape(-1)[ids],self.gy.reshape(-1)[ids]
        offsets=[(-.3,-.3),(-.3,.3),(.3,-.3),(.3,.3)]
        # One sample/cell/frame by default; coverage is earned through fusion.
        sample_offsets=[offsets[frame_index%4]] if frame_index else offsets
        for ox,oy in sample_offsets:
            offset = self.res*np.array([ox,oy])
            samples = np.column_stack([centres+offset,zz+gx*offset[0]+gy*offset[1]])
            samples[:,2] += rng.normal(0,.001,len(ids))
            points.append(samples[ground|wall])
            top = samples[wall].copy()
            top[:,2] += .6
            points.append(top)
            for wall_height in [.15,.30,.45]:
                side=samples[wall].copy();side[:,2]+=wall_height;points.append(side)
        if len(self.overhang_points) and not (self.name=='intermittent_branch' and frame_index%40>=5):
            near=np.linalg.norm(self.overhang_points[:,:2]-xy,axis=1)<self.cfg['sensor_range_m']
            points.append(self.overhang_points[near])
        result=np.concatenate(points)
        if self.cfg['simulate_frame_occlusion']:
            from geometry import Geometry,visible_past_boxes
            ground_z,pitch,roll=self.height_angles(body_xy,yaw)
            r=Pose(0,np.zeros(3),quaternion(roll,pitch,yaw)).rotation
            body=(result-np.array([*body_xy,ground_z+self.cfg['body_reference_height_m']]))@r
            result=result[visible_past_boxes(body,mount,Geometry(self.cfg).boxes)]
        return result


class Simulation:
    def __init__(self,name='flat_detour',cfg=None,seed=21):
        self.cfg = config() if cfg is None else cfg
        self.cfg=dict(self.cfg,layout_seed=seed)
        self.world = World(name,self.cfg)
        self.nav = Navigator(self.cfg)
        self.rng = np.random.RandomState(seed)
        self.name,self.seed = name,seed
        self.xy,self.yaw = np.array([0.,0.]),0.
        self.v,self.w = 0.,0.
        self.t,self.seq = 0.,0
        self.pending = deque()
        self.trace = []
        self.collisions,self.unsafe_slopes = 0,0
        self.max_pitch,self.max_roll = 0.,0.
        self.done = False
        self.state = None
        self.stop_distance_after_dropout = None
        self.dropout_xy = None
        self.wall0 = 1800000000.
        self.first_proximity_time=None
        self.compute_times=[]
        self.stop_started=None
        self.stop_position=None
        self.stopping_distance=None
        self.human_confirmation_sent=False

    def step(self):
        dt = self.cfg['simulation_dt_s']
        self.seq += 1
        now = self.wall0+self.t
        tick_started=time.perf_counter()
        z,pitch,roll = self.world.height_angles(self.xy,self.yaw)
        if self.name=='elevated_plane':z=0. # Incorrect floating surface with unchanged body datum.
        true_pose=Pose(now,np.array([*self.xy,z+self.cfg['body_reference_height_m']]),quaternion(roll,pitch,self.yaw))
        drift=.003 if self.name=='localization_drift' else self.cfg['simulated_drift_mps']
        yaw_drift=.002 if self.name=='localization_drift' else self.cfg['simulated_yaw_drift_rps']
        pose=Pose(now,true_pose.position+np.array([0,drift*self.t,0]),quaternion(roll,pitch,self.yaw+yaw_drift*self.t))
        cloud = self.world.cloud(self.xy,self.rng,self.yaw,self.seq)
        # Registered points share the same simulated odometry drift; truth stays in World.
        c,sn=math.cos(yaw_drift*self.t),math.sin(yaw_drift*self.t)
        cloud=cloud@np.array([[c,sn,0],[-sn,c,0],[0,0,1]])+np.array([0,drift*self.t,0])
        delay = .12
        if self.name=='sensor_delay':
            delay = .30+.1*math.sin(self.t)
        if self.name=='backlog_20s' and 5<=self.t<8:
            delay = 20.
        # Queue delivery preserves original source stamps, including delayed bursts.
        self.pending.append((self.t+delay,self.t,pose,cloud,true_pose))
        due = [p for p in self.pending if p[0]<=self.t+1e-8]
        self.pending = deque(p for p in self.pending if p[0]>self.t+1e-8)
        due.sort(key=lambda p:p[0])
        dropout = 5<=self.t<8
        if dropout and self.dropout_xy is None and ('dropout' in self.name or self.name=='backlog_20s'):
            self.dropout_xy = self.xy.copy()
        for _,source_t,p,points,true_observation_pose in due:
            if not (self.name=='odom_dropout' and dropout):
                self.nav.pose_input(p,now,self.t)
            if not (self.name=='cloud_dropout' and dropout):
                self.nav.cloud_input(points,p.stamp,p.frame,now,self.t)
            if not (self.name=='direction_dropout' and dropout):
                # Mic origin and a fixed 3D source generate the simulated observation.
                # Use the actual body pose, not the drifted pose, for acoustic physics.
                acoustic_origin=true_observation_pose.position+true_observation_pose.rotation@np.asarray(self.cfg['microphone_position_body_m'])
                vector=true_observation_pose.rotation.T@(self.world.source_position-acoustic_origin)
                if self.name=='goal_jitter':
                    a = self.rng.normal(0,math.radians(10))
                    vector[:2] = np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])@vector[:2]
                self.nav.direction_input(dict(seq=int(round(source_t/dt))+1,stamp_sec=p.stamp,
                    frame_id='body',kind='direction',vector=vector.tolist(),confidence=.9,valid=True,
                    session_id='simulation-session',source_id='bird-1'),now,self.t)
        if self.name=='manual_confirmation' and self.t>=12 and not self.human_confirmation_sent:
            self.human_confirmation_sent=self.nav.task.confirm(dict(task_id=self.nav.task.task_id,seq=1,stamp_sec=now,action='confirm_found',confirmed=True),now,self.t)
        plan_started=time.perf_counter()
        state = self.nav.step(now,self.t,dt)
        self.compute_times.append(dict(total_s=time.perf_counter()-tick_started,navigation_s=time.perf_counter()-plan_started))
        command_v,command_w = state['candidate']
        # A first-order approximation of the high-level velocity response.
        alpha = 1.-math.exp(-dt/self.cfg['simulation_response_tau_s'])
        if abs(command_v)<1e-10 and abs(self.v)>.001 and self.stop_started is None:
            self.stop_started=self.t;self.stop_position=self.xy.copy()
        self.v += np.clip(alpha*(command_v-self.v),-self.cfg['braking_deceleration_mps2']*dt,self.cfg['acceleration_mps2']*dt)
        if self.stop_started is not None and abs(self.v)<.005 and self.stopping_distance is None:
            self.stopping_distance=float(np.linalg.norm(self.xy-self.stop_position))
        self.w += alpha*(command_w-self.w)
        old = self.xy.copy()
        self.xy,self.yaw = integrate_twist(self.xy,self.yaw,self.v,self.w,dt)
        # Ground truth checks are recorded; never clamp a collision to manufacture a pass.
        from geometry import Geometry
        true_z,true_pitch,true_roll=self.world.height_angles(self.xy,self.yaw)
        physical_pose=Pose(now,np.array([*self.xy,true_z+self.cfg['body_reference_height_m']]),quaternion(true_roll,true_pitch,self.yaw))
        head_hit=Geometry(self.cfg).collision(self.world.overhang_points,physical_pose.position,physical_pose.rotation)
        if not self.world.valid_footprint(self.xy) or head_hit:
            self.collisions += 1
        _,true_pitch,true_roll = self.world.height_angles(self.xy,self.yaw)
        pd,rd = abs(math.degrees(true_pitch)),abs(math.degrees(true_roll))
        if abs(self.v)>.005:
            self.max_pitch,self.max_roll = max(self.max_pitch,pd),max(self.max_roll,rd)
            if pd>self.cfg['max_pitch_deg']+.3 or rd>self.cfg['max_roll_deg']+.3:
                self.unsafe_slopes += 1
        row = dict(t=round(self.t,4),x=float(self.xy[0]),y=float(self.xy[1]),yaw=self.yaw,
                   v=self.v,w=self.w,command=state['candidate'],reason=state['reason'],
                   source_age_s=state['source_age_s'],goal=state['goal'],
                   path=state['path'] if self.seq%4==0 else [],
                   route_headings=self.nav.route_headings if self.seq%4==0 else [],
                   route_cursor=state.get('route_cursor'),tracking_mode=state.get('tracking_mode'),
                   pitch_deg=pd,roll_deg=rd,task=state['task'])
        self.trace.append(row)
        self.state = state
        self.t += dt
        if self.dropout_xy is not None and self.stop_distance_after_dropout is None and self.t>=5.8:
            self.stop_distance_after_dropout = float(np.linalg.norm(self.xy-self.dropout_xy))
        # Truth is for evaluation only. It never terminates the controller.
        if np.linalg.norm(self.xy-self.world.goal)<.65 and self.first_proximity_time is None:
            self.first_proximity_time=self.t
        self.done=bool(self.nav.task.state=='COMPLETE' and abs(self.v)<.005 and abs(self.w)<.01)
        return state

    def run(self,duration=100.):
        while self.t<duration and not self.done:
            self.step()
            if self.first_proximity_time is not None and self.t-self.first_proximity_time>=5 and self.name!='manual_confirmation':
                break
            if self.name in ['steep_slope','unsafe_cross_slope','too_narrow','dropoff','dead_end','blind_zone','elevated_plane','overhead_branch','intermittent_branch'] and self.t>=18 and all(np.linalg.norm(r['command'])<1e-9 for r in self.trace[-30:]):
                break
        return self.summary()

    def summary(self):
        fault_case = self.name in ['cloud_dropout','odom_dropout','direction_dropout','backlog_20s']
        window = [r for r in self.trace if 5.7<=r['t']<8.]
        zero_on_fault = bool(window) and all(np.linalg.norm(r['command'])<1e-9 for r in window)
        recovered = any(r['t']>8.7 and r['command'][0]>.01 for r in self.trace)
        blocked_case = self.name in ['steep_slope','unsafe_cross_slope','too_narrow','dropoff','dead_end']
        stopped = len(self.trace)>5 and all(np.linalg.norm(r['command'])<1e-9 for r in self.trace[-5:])
        passed = self.collisions==0 and self.unsafe_slopes==0 and (stopped if blocked_case else self.done)
        if fault_case:
            passed = passed and zero_on_fault and recovered
        proximity=self.first_proximity_time is not None
        blocked_case=blocked_case or self.name in ['blind_zone','elevated_plane','overhead_branch','intermittent_branch']
        safe=self.collisions==0 and self.unsafe_slopes==0
        expected='safe_rejection' if blocked_case else 'directional_approach'
        passed=safe and (stopped if blocked_case else proximity)
        if self.name=='manual_confirmation':
            expected='operator_confirmed_completion';passed=safe and self.done and proximity and stopped
        if fault_case:passed=passed and zero_on_fault and recovered
        # Direction-only approach success and completed search are intentionally distinct.
        times=np.array([r['navigation_s'] for r in self.compute_times])
        return dict(scenario=self.name,seed=self.seed,passed=bool(passed),reached=proximity,
                    expected_behavior=expected,proximity_reached=proximity,
                    task_completed=self.nav.task.state=='COMPLETE',task_success=bool(self.done and proximity and safe),
                    controller_state=self.nav.task.state,arrival_stopped=bool(self.done and abs(self.v)<.005 and abs(self.w)<.01),
                    proximity_and_stopped=bool(proximity and stopped and abs(self.v)<.005 and abs(self.w)<.01),
                    evaluation_proximity_radius_m=.65,evaluation_distance='horizontal distance from body origin to source ground projection',
                    navigation_runtime_ms=dict(p50=float(np.quantile(times,.5)*1000),p95=float(np.quantile(times,.95)*1000),maximum=float(times.max()*1000)),
                    p95_tick_ms=float(np.quantile([r['total_s'] for r in self.compute_times],.95)*1000),
                    real_time_10hz_verified=False,first_proximity_time_s=self.first_proximity_time,
                    measured_simulated_stop_distance_m=self.stopping_distance,
                    duration_s=round(self.t,3),distance_to_source_m=float(np.linalg.norm(self.xy-self.world.goal)),
                    path_length_m=float(sum(math.dist([a['x'],a['y']],[b['x'],b['y']]) for a,b in zip(self.trace,self.trace[1:]))),
                    collision_samples=self.collisions,unsafe_slope_samples=self.unsafe_slopes,
                    max_moving_pitch_deg=self.max_pitch,max_moving_roll_deg=self.max_roll,
                    stopped_at_end=stopped,zero_during_fault=zero_on_fault if fault_case else None,
                    recovered_after_fault=recovered if fault_case else None,
                    travel_before_fault_stop_m=self.stop_distance_after_dropout,
                    subgoal_updates=self.nav.direction.updates,reasons=dict(Counter(r['reason'] for r in self.trace)),
                    executable=False,motion_commands_sent=0)

    def save(self,directory):
        directory = Path(directory)
        directory.mkdir(parents=True,exist_ok=True)
        (directory/'summary.json').write_text(json.dumps(self.summary(),indent=2,allow_nan=False))
        with (directory/'trace.jsonl').open('w') as f:
            for r in self.trace:
                f.write(json.dumps(r,allow_nan=False)+'\n')
        data = dict(world_z=self.world.z,world_obstacle=self.world.obstacle,world_ground=self.world.ground,
                    world_origin=self.world.origin,resolution=self.world.res,source_position=self.world.source_position,
                    microphone_position_body_m=self.cfg['microphone_position_body_m'],lidar_position_body_m=self.cfg['lidar_position_body_m'],
                    xy=np.array([[r['x'],r['y']] for r in self.trace]))
        if self.nav.map is not None:
            terrain = self.nav.map.analyze(self.t)
            data.update(grid=terrain['grid'],blocked=terrain['blocked'],slope=terrain['slope'],
                        origin=self.nav.map.origin,elevation=terrain['elevation'])
        np.savez_compressed(directory/'maps.npz',**data)
        (directory/'task_transitions.json').write_text(json.dumps(self.nav.task.transitions,indent=2))


def plot_results(simulations,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    cols = 3
    rows = math.ceil(len(simulations)/cols)
    fig,axes = plt.subplots(rows,cols,figsize=(15,4*rows),squeeze=False)
    for ax,sim in zip(axes.flat,simulations):
        w = sim.world
        extent = [w.origin[0],w.origin[0]+w.nx*w.res,w.origin[1],w.origin[1]+w.ny*w.res]
        ax.imshow(w.z,origin='lower',extent=extent,cmap='terrain',alpha=.55,aspect='equal')
        overlay = np.ma.masked_where(~w.obstacle,np.ones_like(w.z))
        ax.imshow(overlay,origin='lower',extent=extent,cmap='Reds',vmin=0,vmax=1)
        ax.imshow(np.ma.masked_where(w.ground,np.ones_like(w.z)),origin='lower',extent=extent,cmap='Greys',vmin=0,vmax=1)
        xy = np.array([[r['x'],r['y']] for r in sim.trace])
        ax.plot(xy[:,0],xy[:,1],color='#1769d2',lw=2,label='simulated trajectory')
        ax.scatter([0,w.goal[0]],[0,w.goal[1]],c=['green','purple'],s=45,zorder=5)
        ax.set_title(sim.name+' / '+('PASS' if sim.summary()['passed'] else 'FAIL'))
        ax.set_xlabel('x (m)');ax.set_ylabel('y (m)');ax.grid(alpha=.2)
    for ax in list(axes.flat)[len(simulations):]:
        ax.axis('off')
    fig.suptitle('Go2 navigation framework | synthetic terrain | kinematic simulation only',fontsize=14)
    fig.tight_layout(rect=[0,0,1,.97]);fig.savefig(path,dpi=130);plt.close(fig)


def gui(name,cfg):
    import tkinter as tk
    from tkinter import ttk
    root=tk.Tk();root.title('Go2 寻声仿真 v3：场景真值 / 机器人感知')
    current=[Simulation(name,cfg)];running=[True]
    panel=ttk.Frame(root);panel.pack(fill='x')
    selected=tk.StringVar(value=name)
    ttk.Combobox(panel,textvariable=selected,values=SCENARIOS,state='readonly',width=24).pack(side='left')
    def reset():current[0]=Simulation(selected.get(),cfg);running[0]=True
    ttk.Button(panel,text='重置',command=reset).pack(side='left')
    ttk.Button(panel,text='暂停 / 继续仿真',command=lambda:running.__setitem__(0,not running[0])).pack(side='left')
    def confirm():
        sim=current[0];task=sim.nav.task
        task.confirm(dict(task_id=task.task_id,seq=task.confirm_seq+1,stamp_sec=sim.wall0+sim.t,
                          action='confirm_found',confirmed=True),sim.wall0+sim.t,sim.t)
    ttk.Button(panel,text='人工确认找到 → 停稳后完成',command=confirm).pack(side='left')
    view=tk.StringVar(value='occupancy')
    ttk.Combobox(panel,textvariable=view,values=['occupancy','slope','clearance'],state='readonly',width=12).pack(side='left')
    speed=tk.IntVar(value=1)
    ttk.Spinbox(panel,from_=1,to=10,textvariable=speed,width=3).pack(side='left')
    canvas=tk.Canvas(root,width=1260,height=620,bg='#eef2f6');canvas.pack(fill='both',expand=True)
    status=tk.StringVar();ttk.Label(root,textvariable=status,wraplength=1240).pack(fill='x')
    def draw():
        sim=current[0]
        if running[0] and not sim.done:
            for _ in range(max(1,min(10,speed.get()))):
                sim.step()
                if sim.done:break
        canvas.delete('all')
        width,height=canvas.winfo_width(),canvas.winfo_height();pane=width/2
        span=cfg['map_size_m'];scale=min((pane-40)/span,(height-70)/span)
        origin=sim.nav.map.origin if sim.nav.map else np.array([-span/2,-span/2])
        def pixel(xy,side):return (side*pane+20+(xy[0]-origin[0])*scale,height-30-(xy[1]-origin[1])*scale)
        canvas.create_text(18,12,anchor='nw',text='场景真值（只供评分）',fill='#203348')
        canvas.create_text(pane+18,12,anchor='nw',text='机器人实际感知：'+view.get()+'（深灰=未知）',fill='#203348')
        w=sim.world
        for iy,ix in np.ndindex(w.z.shape):
            x,y=pixel(w.xy[iy,ix],0)
            if not 0<=x<pane:continue
            if w.obstacle[iy,ix]:color='#d45a60'
            elif not w.ground[iy,ix]:color='#303744'
            else:
                level=int(np.clip(230-w.z[iy,ix]*50,135,245));color=f'#{level:02x}e6cf'
            r=scale*w.res/2;canvas.create_rectangle(x-r,y-r,x+r,y+r,fill=color,outline='')
        if len(w.overhang_points):
            a=pixel(w.overhang_points[0,:2],0);b=pixel(w.overhang_points[-1,:2],0)
            canvas.create_line(*a,*b,fill='#8b3fc0',width=4)
            canvas.create_text(a[0]+8,a[1],anchor='w',text=f'横梁 z={w.overhang_points[0,2]:.2f}m',fill='#663399')
        if sim.nav.map:
            m=sim.nav.map;t=m.analyze(sim.t)
            for iy,ix in np.ndindex(t['grid'].shape):
                x,y=pixel(m.world((ix,iy)),1)
                if t['grid'][iy,ix]<0:color='#505d6e'
                elif t['grid'][iy,ix]>0:color='#d45a60'
                elif view.get()=='slope':
                    value=int(np.clip(t['slope'][iy,ix]/25*220,0,220));color=f'#{min(255,80+value):02x}{max(60,210-value//2):02x}80'
                elif view.get()=='clearance':
                    value=int(np.clip(t['clearance'][iy,ix]/2*200,0,200));color=f'#60{80+value//2:02x}{55+value:02x}'
                elif t['blocked'][iy,ix]:color='#e8bc72'
                else:color='#eff4ef'
                r=scale*m.res/2;canvas.create_rectangle(x-r,y-r,x+r,y+r,fill=color,outline='')
        for side in [0,1]:
            trace=sim.trace[::3]
            if len(trace)>1:canvas.create_line(*[v for row in trace for v in pixel([row['x'],row['y']],side)],fill='#236bd1',width=2)
            if sim.state and len(sim.state['path'])>1:
                canvas.create_line(*[v for point in sim.state['path'] for v in pixel(point,side)],fill='#08a6a6',width=2)
            position=sim.xy if side==0 or sim.nav.pose is None else sim.nav.pose.position[:2]
            yaw=sim.yaw if side==0 or sim.nav.pose is None else sim.nav.pose.yaw
            rotation=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
            for box in cfg['geometry_boxes']:
                lo,hi=box['minimum'],box['maximum']
                corners=np.array([[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]]])@rotation.T+position
                canvas.create_polygon(*[v for point in corners for v in pixel(point,side)],fill='',outline='#1769d2' if box['name']!='head_frame' else '#d08814',width=2)
            for offset,color in [(cfg['lidar_position_body_m'],'#e86800'),(cfg['microphone_position_body_m'],'#b43699')]:
                x,y=pixel(position+rotation@np.asarray(offset[:2]),side)
                canvas.create_oval(x-3,y-3,x+3,y+3,fill=color)
        # The true source is shown only on the evaluation pane.
        x,y=pixel(w.source_position[:2],0);canvas.create_oval(x-5,y-5,x+5,y+5,fill='#7b36bd')
        s=sim.state or {};cmd=s.get('candidate',[0,0]);age=s.get('source_age_s')
        status.set(f"{sim.name} | t={sim.t:.1f}s | {sim.nav.task.state} | {s.get('reason','waiting')} | v={cmd[0]:.3f}m/s w={cmd[1]:.3f}rad/s | 源年龄={age if age is not None else '未知'}\n橙点=后背雷达；紫点=麦克风；黄色轮廓=未测头架示例。只有方向，需人工确认完成；无真实运动输出。")
        root.after(60,draw)
    root.after(100,draw);root.mainloop()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario',choices=SCENARIOS,default='flat_detour')
    p.add_argument('--suite',action='store_true')
    p.add_argument('--gui',action='store_true')
    p.add_argument('--duration',type=float,default=220.)
    p.add_argument('--seed',type=int,default=21)
    p.add_argument('--seeds',help='Comma-separated seed sweep')
    p.add_argument('--scenarios',help='Comma-separated subset of scenarios')
    p.add_argument('--config')
    p.add_argument('--output',default=None)
    a = p.parse_args();cfg = config(a.config)
    if a.gui:return gui(a.scenario,cfg)
    # Unique run folders preserve previous evidence instead of overwriting it.
    output = Path(a.output or Path(__file__).parent/'runs'/time.strftime('%Y%m%d_%H%M%S'))
    output.mkdir(parents=True,exist_ok=False)
    (output/'config.json').write_text(json.dumps(cfg,indent=2))
    sims = []
    names=a.scenarios.split(',') if a.scenarios else SCENARIOS if a.suite else [a.scenario]
    if not all(name in SCENARIOS for name in names):p.error('unknown scenario')
    seeds=[int(seed) for seed in a.seeds.split(',')] if a.seeds else [a.seed]
    for seed in seeds:
        for name in names:
            sim = Simulation(name,cfg,seed);summary = sim.run(a.duration)
            label=f'{name}_seed{seed}' if len(seeds)>1 else name
            sim.save(output/label);sims.append(sim)
            print(json.dumps(summary),flush=True)
    results=[s.summary() for s in sims]
    approach=[r for r in results if r['expected_behavior']=='directional_approach']
    reject=[r for r in results if r['expected_behavior']=='safe_rejection']
    manual=[r for r in results if r['expected_behavior']=='operator_confirmed_completion']
    report = dict(all_passed=all(r['passed'] for r in results),version=3,
                  model='direction-only acoustic navigation; configurable head frame; sparse synthetic lidar; kinematics',
                  statistics=dict(approach_proximity_rate=sum(r['proximity_reached'] for r in approach)/len(approach) if approach else None,
                                  safe_rejection_rate=sum(r['passed'] for r in reject)/len(reject) if reject else None,
                                  operator_confirmed_task_success_rate=sum(r['task_success'] for r in manual)/len(manual) if manual else None,
                                  direction_only_completed_tasks=sum(r['task_completed'] for r in approach),
                                  completed_and_stopped=sum(r['arrival_stopped'] for r in results)),
                  results=results,motion_commands_sent=0)
    (output/'report.json').write_text(json.dumps(report,indent=2))
    plot_results(sims,output/'overview.png')
    print('OUTPUT',output)
    return 0 if report['all_passed'] else 1

if __name__=='__main__':
    raise SystemExit(main())
