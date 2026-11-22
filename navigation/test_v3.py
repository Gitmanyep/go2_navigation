"""Regressions for v3: evidence, ordered heading control, watchdog and settle continuity."""
import math
import unittest
import numpy as np
from core import config, TerrainMap, Pose, Navigator
from acoustic_task import AcousticTask


def floor():
    yy,xx=np.mgrid[-1.8:1.81:.15,-1.8:1.81:.15]
    p=np.column_stack([xx.ravel()+.02,yy.ravel()+.02,np.zeros(xx.size)])
    return np.repeat(p,3,axis=0)


def pose(stamp,x=0.,yaw=0.):
    return Pose(stamp,np.array([x,0.,.3]),np.array([0.,0.,math.sin(yaw/2),math.cos(yaw/2)]))


class V3Regressions(unittest.TestCase):
    def setUp(self):self.cfg=config()

    def branch_map(self):
        m=TerrainMap(self.cfg,[0,0]);p=floor()
        m.integrate(np.r_[p,[[.47,.02,.85]]],pose(100).position,0.)
        return m,p

    def test_overhead_survives_missing_returns(self):
        m,p=self.branch_map()
        for i in range(1,11):m.integrate(p,pose(100).position,i*.1)
        t=m.analyze(1.);x,y=m.cell([0,0])
        self.assertTrue(t['heading_masks'][0][y,x])

    def test_exact_3d_sweep_retains_missing_branch_returns(self):
        from geometry import Geometry
        m,p=self.branch_map()
        for i in range(1,11):m.integrate(p,pose(100).position,i*.1)
        self.assertEqual(Geometry(self.cfg).collision(m.recent_points,pose(100).position,np.eye(3)),'head_frame')
        m.clear_obstacle_volume([m.cell([.47,.02])],.7,1.,1.1,certified=True)
        self.assertIsNone(Geometry(self.cfg).collision(m.recent_points,pose(100).position,np.eye(3)))
        m.integrate(p,pose(100).position,1.2)
        self.assertIsNone(Geometry(self.cfg).collision(m.recent_points,pose(100).position,np.eye(3)))

    def test_sparse_branch_is_backfilled_after_ground_fusion(self):
        from geometry import Geometry
        m=TerrainMap(self.cfg,[0,0]);p=floor()[::3]
        m.integrate(np.r_[p,[[.47,.02,.85]]],pose(100).position,0.)
        for i in range(1,11):m.integrate(p,pose(100).position,i*.1)
        self.assertEqual(Geometry(self.cfg).collision(m.recent_points,pose(100).position,np.eye(3)),'head_frame')
        for i in range(11,41):m.integrate(p,pose(100).position,i*.1)
        x,y=m.cell([.47,.02]);self.assertEqual(m.analyze(4.)['grid'][y,x],-1)
        self.assertFalse(m.obstacle_frames)

    def test_expired_obstacle_is_unknown_even_with_fresh_floor(self):
        m,p=self.branch_map()
        for i in range(1,41):m.integrate(p,pose(100).position,i*.1)
        t=m.analyze(4.);x,y=m.cell([.47,.02])
        self.assertEqual(t['grid'][y,x],-1)

    def test_observed_movement_resets_settle_without_tick(self):
        task=AcousticTask(self.cfg)
        task.confirm(dict(task_id=task.task_id,seq=1,stamp_sec=100.,action='confirm_found',confirmed=True),100.,0.)
        for i in range(4):
            task.observe_pose(pose(100+i*.1));task.tick(i*.1,True)
        for i in range(4,14):task.observe_pose(pose(100+i*.1,(i-3)*.01))
        task.observe_pose(pose(101.4,.10));task.tick(1.4,True)
        task.observe_pose(pose(101.5,.10));task.tick(1.5,True)
        self.assertNotEqual(task.state,'COMPLETE')

    def test_missing_odometry_resets_settle(self):
        task=AcousticTask(self.cfg)
        task.confirm(dict(task_id=task.task_id,seq=1,stamp_sec=100.,action='confirm_found',confirmed=True),100.,0.)
        for i in range(4):task.observe_pose(pose(100+i*.1));task.tick(i*.1,True)
        task.observe_pose(pose(102.));task.tick(2.,True)
        self.assertNotEqual(task.state,'COMPLETE')

    def test_obstacle_clear_requires_complete_certified_volume(self):
        m,p=self.branch_map();c=m.cell([.47,.02])
        for kwargs in [dict(certified=False),dict(certified=True)]:
            with self.assertRaises(ValueError):m.clear_obstacle_volume([c],.9,1.,.2,**kwargs)
        with self.assertRaises(ValueError):m.clear_obstacle_volume([c],.7,1.,0.,certified=True)
        m.clear_obstacle_volume([c],.7,1.,.2,certified=True)
        # Old branch points still in fusion must not undo a newer certified clear.
        m.integrate(p,pose(100).position,.3)
        x,y=m.cell([0,0]);self.assertFalse(m.analyze(.3)['heading_masks'][0][y,x])
        with self.assertRaises(ValueError):m.clear_obstacle_volume([c],.7,1.,.1,certified=True)
        # A later actual branch return wins over clearance.
        m.integrate(np.r_[p,[[.47,.02,.85]]],pose(100).position,.4)
        self.assertTrue(m.analyze(.4)['heading_masks'][0][y,x])

    def test_obstacle_memory_rolls_with_map(self):
        m,p=self.branch_map();old=m.cell([.47,.02]);m.recenter([3.,0])
        c=m.cell([.47,.02]);self.assertNotEqual(c,old)
        self.assertTrue(math.isfinite(m.overhead_seen[c[1],c[0]]))

    def test_analysis_cache_expires_at_evidence_boundary(self):
        m,p=self.branch_map();a=m.analyze(.1)
        self.assertIs(a,m.analyze(.2))
        self.assertIsNot(a,m.analyze(2.))
        m.integrate(p,pose(100).position,2.1)
        self.assertIsNot(a,m.analyze(2.1))

    def forced_clockwise_route(self):
        from heading_planner import heading_astar
        size=11;blocked=np.ones((size,size),bool);blocked[5:8,5]=False
        masks=[blocked.copy() for _ in range(16)]
        masks[1][5,5]=True # CCW half-step is obstructed; clockwise is clear.
        terrain=dict(blocked=blocked,heading_masks=masks,gx=np.zeros((size,size)),
                     gy=np.zeros((size,size)),slope=np.zeros((size,size)))
        route=heading_astar(terrain,(5,5),(5,7),0.,self.cfg)
        self.assertTrue(route);self.assertEqual(route.states[1][2],7)
        return route

    def test_tracker_executes_clockwise_plan_not_goal_shortcut(self):
        route=self.forced_clockwise_route();n=Navigator(self.cfg)
        n.map=TerrainMap(self.cfg,[0,0]);n.pose=pose(100)
        n.route=[(np.array(c)-[5,5])*.15 for c in route]
        n.route_headings=[h*math.pi/4 for x,y,h in route.states]
        v,w,mode=n.track_route()
        self.assertEqual(mode,'rotate');self.assertEqual(v,0.);self.assertLess(w,0.)
        self.assertEqual(n.route_cursor,1);self.assertTrue(n.rotation_pending())
        # Execute each turn physically; do not skip six repeated XY nodes.
        visited=[]
        for _ in range(250):
            v,w,mode=n.track_route();visited.append(n.route_cursor)
            if mode=='translate':break
            n.pose=pose(100,n.pose.position[0],n.pose.yaw+w*.1)
        self.assertEqual(mode,'translate');self.assertTrue(set(range(1,7)).issubset(visited))
        self.assertAlmostEqual(n.pose.yaw,math.pi/2,delta=.08)

    def test_tracker_rotation_across_pi_uses_adjacent_signed_step(self):
        n=Navigator(self.cfg);n.map=TerrainMap(self.cfg,[0,0]);n.pose=pose(100,yaw=math.radians(179))
        n.route=[np.zeros(2),np.zeros(2)];n.route_headings=[math.pi,5*math.pi/4]
        v,w,mode=n.track_route();self.assertEqual(v,0.);self.assertGreater(w,0.)

    def test_audit_interpolates_short_arc_both_directions(self):
        from audit_truth import interpolate_pose
        from geometry import Geometry
        for a,b in [(179,-179),(-179,179)]:
            previous=np.array([-.1,0.,math.radians(a)]);current=np.array([.1,0.,math.radians(b)])
            samples=[interpolate_pose(previous,current,t) for t in np.linspace(0,1,21)]
            self.assertTrue(all(abs(s[2])>math.radians(178) for s in samples))
            mid=samples[10];body=pose(0,yaw=mid[2])
            self.assertIsNone(Geometry(self.cfg).collision(np.array([[.6,0,.8]]),body.position,body.rotation))
            self.assertAlmostEqual(mid[0],0.)

    def test_confirm_cannot_complete_from_timer_without_new_poses(self):
        task=AcousticTask(self.cfg)
        task.confirm(dict(task_id=task.task_id,seq=1,stamp_sec=100.,action='confirm_found',confirmed=True),100.,0.)
        task.observe_pose(pose(100.));task.observe_pose(pose(100.1))
        for i in range(20):task.tick(.1+i*.1,True)
        self.assertNotEqual(task.state,'COMPLETE')

    def test_clock_rollback_cannot_complete_before_freshness_rejection(self):
        n=Navigator(self.cfg);p=floor()
        n.task.confirm(dict(task_id=n.task.task_id,seq=1,stamp_sec=100.,action='confirm_found',confirmed=True),100.,0.)
        for i in range(9):
            t=i*.1;n.pose_input(pose(100+t),100+t,t);n.cloud_input(p,100+t,'camera_init',100+t,t);n.step(100+t,t)
        n.pose_input(pose(100.9),100.9,.9);n.cloud_input(p,100.9,'camera_init',100.9,.9)
        result=n.step(99.9,.9)
        self.assertNotEqual(n.task.state,'COMPLETE');self.assertEqual(result['candidate'],[0.,0.])
        self.assertIsNone(n.task.stop_since)

    def test_cloud_outage_resets_completion_in_navigator(self):
        n=Navigator(self.cfg);p=floor()
        n.task.confirm(dict(task_id=n.task.task_id,seq=1,stamp_sec=100.,action='confirm_found',confirmed=True),100.,0.)
        for i in range(7):
            t=i*.1;n.pose_input(pose(100+t),100+t,t);n.cloud_input(p,100+t,'camera_init',100+t,t);n.step(100+t,t)
        for i in range(7,18):
            t=i*.1;n.pose_input(pose(100+t,(i-6)*.01),100+t,t);n.step(100+t,t)
        for i in range(18,21):
            t=i*.1;n.pose_input(pose(100+t,.11),100+t,t);n.cloud_input(p,100+t,'camera_init',100+t,t);n.step(100+t,t)
        self.assertNotEqual(n.task.state,'COMPLETE')
        for i in range(21,31):
            t=i*.1;n.pose_input(pose(100+t,.11),100+t,t);n.cloud_input(p,100+t,'camera_init',100+t,t);n.step(100+t,t)
        self.assertEqual(n.task.state,'COMPLETE')

    def test_navigator_holds_planned_rotation_past_replan_period(self):
        from test_core import FrameworkTests
        helper=FrameworkTests();helper.setUp();n,points=helper.navigator_ready()
        route=self.forced_clockwise_route()
        n.route=[(np.array(c)-[5,5])*.15 for c in route]
        n.route_headings=[h*math.pi/4 for x,y,h in route.states]
        n.route_cursor=1;n.last_plan=-1.;n.planned_goal=n.direction.goal.copy()
        # A replacement XY-biased replan must not interrupt a committed turn.
        n.plan=lambda *args:(_ for _ in ()).throw(AssertionError('rotation replanned'))
        state=n.step(100.,0.)
        self.assertEqual(state['candidate'][0],0.)
        self.assertLess(state['candidate'][1],0.)
        self.assertEqual(state['tracking_mode'],'rotate')

    def test_straight_lookahead_preserves_next_explicit_turn(self):
        n=Navigator(self.cfg);n.map=TerrainMap(self.cfg,[0,0]);n.pose=pose(100,yaw=.1)
        n.route=[np.array([x,0.]) for x in [0.,.15,.3,.3]]
        n.route_headings=[0.,0.,0.,math.pi/4]
        v,w,mode=n.track_route();self.assertEqual(mode,'translate');self.assertGreater(v,0.)
        # Passing collinear nodes consumes them in order, then rotates at the end.
        n.pose=pose(101,x=.30);v,w,mode=n.track_route()
        self.assertEqual(n.route_cursor,3);self.assertEqual(mode,'rotate')
        self.assertEqual(v,0.);self.assertGreater(w,0.)

    def test_unknown_region_checks_protruding_head_frame(self):
        from geometry import Geometry
        g=Geometry(self.cfg);unknown=np.array([[.6,0.]])
        self.assertTrue(g.footprint_unknown(unknown,[0.,0.],0.,.15))
        self.assertFalse(g.footprint_unknown(unknown,[0.,0.],math.pi,.15))

    def test_shared_search_preserves_ranked_frontier_priority(self):
        from heading_planner import heading_astar_many
        size=9;free=np.zeros((size,size),bool);z=np.zeros((size,size))
        terrain=dict(blocked=free,heading_masks=[free.copy() for _ in range(16)],gx=z,gy=z,slope=z)
        route=heading_astar_many(terrain,(1,1),[(7,1),(2,1)],0.,self.cfg)
        self.assertTrue(route);self.assertEqual(route[-1],(7,1))

    def test_exploration_deadline_is_checked_on_committed_routes_and_latched(self):
        from test_core import FrameworkTests
        helper=FrameworkTests();helper.setUp();n,points=helper.navigator_ready()
        n.exploring=True;n.exploration_started=-self.cfg['exploration_total_timeout_s']
        state=n.step(100.,0.);self.assertEqual(state['candidate'],[0.,0.])
        self.assertEqual(state['reason'],'bounded_exploration_exhausted')
        # A later planner opportunity must not resume an exhausted task.
        n.exploring=False;n.exploration_started=None
        self.assertEqual(n.step(100.,.1)['reason'],'bounded_exploration_exhausted')

    def test_planner_enforces_budget_and_marks_exhaustion(self):
        from heading_planner import heading_astar
        size=40;free=np.zeros((size,size),bool);z=np.zeros((size,size))
        terrain=dict(blocked=free,heading_masks=[free.copy() for _ in range(16)],gx=z,gy=z,slope=z,_plan_deadline=0.)
        route=heading_astar(terrain,(1,1),(38,38),0.,self.cfg)
        self.assertFalse(route);self.assertTrue(terrain['_plan_budget_exhausted'])


class WatchdogTests(unittest.TestCase):
    def test_planning_block_does_not_block_expiry_publication(self):
        import threading,time
        from runtime import PreviewRuntime
        entered=threading.Event();release=threading.Event();blocked=[False]
        def processor(events):
            if blocked[0]:entered.set();release.wait(2.)
            return dict(state=dict(candidate=[.1,0.],reason='tracking')),time.monotonic()+5.
        runtime=PreviewRuntime(processor,lease=.08)
        self.assertTrue(runtime.compute_once());blocked[0]=True
        worker=threading.Thread(target=runtime.compute_once);worker.start()
        try:
            self.assertTrue(entered.wait(.5));time.sleep(.1)
            start=time.perf_counter();result=runtime.read();elapsed=time.perf_counter()-start
            self.assertLess(elapsed,.03);self.assertEqual(result['state']['candidate'],[0.,0.])
        finally:release.set();worker.join(1.)
        # The completed late result cannot gain a fresh lease.
        self.assertEqual(runtime.read()['state']['candidate'],[0.,0.])

    def test_late_commit_and_new_operator_command_cannot_revive_velocity(self):
        from runtime import PreviewRuntime
        clock=[0.];runtime=None
        def processor(events):
            runtime.submit('operator','confirm');clock[0]=.2
            return dict(state=dict(candidate=[.1,0.],reason='tracking')),5.
        runtime=PreviewRuntime(processor,clock=lambda:clock[0],lease=.15)
        self.assertFalse(runtime.compute_once())
        self.assertEqual(runtime.read()['state']['candidate'],[0.,0.])

    def test_new_inputs_do_not_extend_old_result_or_source_lease(self):
        from runtime import PreviewRuntime
        clock=[0.]
        runtime=PreviewRuntime(lambda events:(dict(state=dict(candidate=[.1,0.])),.05),clock=lambda:clock[0])
        self.assertTrue(runtime.compute_once());clock[0]=.06;runtime.submit('cloud',object())
        self.assertEqual(runtime.read()['state']['candidate'],[0.,0.])

    def test_high_rate_sensor_ingress_does_not_starve_valid_results(self):
        from runtime import PreviewRuntime
        clock=[0.];runtime=None
        def processor(events):
            for i in range(20):runtime.submit('pose',i);runtime.submit('cloud',i)
            clock[0]=.02
            return dict(state=dict(candidate=[.1,0.])),.1
        runtime=PreviewRuntime(processor,clock=lambda:clock[0])
        self.assertTrue(runtime.compute_once());self.assertEqual(runtime.read()['state']['candidate'],[.1,0.])
        clock[0]=.11;self.assertEqual(runtime.read()['state']['candidate'],[0.,0.])

    def test_ingress_is_bounded(self):
        from runtime import PreviewRuntime
        runtime=PreviewRuntime(lambda events:(dict(state=dict(candidate=[0.,0.])),0.))
        for i in range(1000):runtime.submit('pose',i);runtime.submit('cloud',i)
        self.assertEqual(len(runtime.poses),300);self.assertEqual(runtime.latest['cloud'],999)


if __name__=='__main__':unittest.main()
