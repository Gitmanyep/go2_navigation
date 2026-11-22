"""Eight-heading search with cached constraints and shared failed-search reachability."""
import heapq
import math
import time
import numpy as np


class HeadingRoute(list):
    def __init__(self,states):
        super().__init__((x,y) for x,y,h in states)
        self.states=states


def _search(terrain,start,goals,yaw,cfg):
    deadline=terrain.get('_plan_deadline',time.perf_counter()+cfg['planner_budget_s'])
    if time.perf_counter()>=deadline:
        terrain['_plan_budget_exhausted']=True;return HeadingRoute([])
    masks=terrain['heading_masks'];height,width=terrain['blocked'].shape
    if '_heading_valid' not in terrain:
        angles=np.arange(16)*math.pi/8
        gx,gy=terrain['gx'][None],terrain['gy'][None]
        c,s=np.cos(angles)[:,None,None],np.sin(angles)[:,None,None]
        terrain['_heading_valid']=(~np.asarray(masks))&(np.abs(gx*c+gy*s)<=math.tan(math.radians(cfg['max_pitch_deg'])))&(np.abs(-gx*s+gy*c)<=math.tan(math.radians(cfg['max_roll_deg'])))
    valid_grid=terrain['_heading_valid']
    def valid(x,y,h):return 0<=x<width and 0<=y<height and valid_grid[h%16,y,x]
    heading=int(round(yaw/(math.pi/4)))%8;initial=(*start,heading)
    if not valid(*start,heading*2):return HeadingRoute([])
    goals=[g for g in goals if 0<=g[0]<width and 0<=g[1]<height and valid_grid[::2,g[1],g[0]].any()]
    if not goals:return HeadingRoute([])
    def route(state,parents):
        states=[state]
        while states[-1]!=initial:states.append(parents[states[-1]])
        return HeadingRoute(states[::-1])
    cached=terrain.get('_reachability')
    if cached and cached[0]==initial:
        _,scores,parents=cached
        for goal in goals:
            states=[(*goal,h) for h in range(8) if (*goal,h) in scores]
            if states:return route(min(states,key=scores.get),parents)
        return HeadingRoute([])
    # Multi-goal fallback uses a single Dijkstra search. Single-goal A* returns
    # its full reachability tree when exhausted; fallback endpoints reuse it.
    goalset=set(goals);primary=goals[0]
    heuristic=(lambda st:math.dist(st[:2],primary)) if len(goals)==1 else (lambda st:0.)
    frontier=[(heuristic(initial),0.,initial)];scores={initial:0.};parents={}
    directions=[(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1)]
    deadline=terrain.get('_plan_deadline',time.perf_counter()+cfg['planner_budget_s'])
    expansions=0;found={}
    while frontier:
        _,cost,state=heapq.heappop(frontier)
        if cost>scores[state]+1e-9:continue
        x,y,h=state
        if (x,y) in goalset:
            if time.perf_counter()>=deadline:
                terrain['_plan_budget_exhausted']=True;return HeadingRoute([])
            if (x,y)==goals[0]:return route(state,parents)
            found.setdefault((x,y),state)
        expansions+=1
        if expansions%64==0 and time.perf_counter()>=deadline:
            terrain['_plan_budget_exhausted']=True
            return HeadingRoute([])
        candidates=[]
        for sign in [-1,1]:
            nh=(h+sign)%8
            if valid(x,y,2*h+sign) and valid(x,y,nh*2):candidates.append(((x,y,nh),.32))
        dx,dy=directions[h];nx,ny=x+dx,y+dy
        if valid(nx,ny,h*2) and (not dx or not dy or (valid(x+dx,y,h*2) and valid(x,y+dy,h*2))):
            candidates.append(((nx,ny,h),math.hypot(dx,dy)*(1+.4*terrain['slope'][ny,nx]/cfg['max_pitch_deg'])))
        for nxt,edge in candidates:
            new=cost+edge
            if new<scores.get(nxt,math.inf):
                scores[nxt]=new;parents[nxt]=state
                heapq.heappush(frontier,(new+heuristic(nxt),new,nxt))
    terrain['_reachability']=(initial,scores,parents)
    for goal in goals:
        if goal in found:return route(found[goal],parents)
    return HeadingRoute([])


def heading_astar(terrain,start,goal,yaw,cfg):return _search(terrain,start,[goal],yaw,cfg)


def heading_astar_many(terrain,start,goals,yaw,cfg):return _search(terrain,start,goals,yaw,cfg)
