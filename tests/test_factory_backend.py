"""Factory API ownership regressions using fake clients; no SDK or DDS imports."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'control'))
from factory_backend import FactoryVelocityClient, StateFeedback
from manual_panel import PulseController


class FakeSport:
    def __init__(self, events):
        self.events, self.stop_code = events, 0

    def Move(self, *values):
        self.events.append(('sport_move', values))
        return 0

    def StopMove(self):
        self.events.append(('sport_stop',))
        return self.stop_code


class FakeAvoid:
    def __init__(self, events):
        self.events = events
        self.enabled = True
        self.acquire_code = self.release_code = self.move_code = 0

    def SwitchGet(self):
        self.events.append(('avoid_get',))
        return 0, self.enabled

    def Move(self, *values):
        self.events.append(('avoid_move', values))
        return self.move_code

    def UseRemoteCommandFromApi(self, enabled):
        self.events.append(('source', enabled))
        return self.acquire_code if enabled else self.release_code


class FactoryBackendTests(unittest.TestCase):
    def setUp(self):
        self.t = [0.]
        self.events = []
        self.sport, self.avoid = FakeSport(self.events), FakeAvoid(self.events)
        self.feedback = StateFeedback(lambda:self.t[0])
        self.observe()
        self.client = FactoryVelocityClient(self.sport, self.avoid, True, self.feedback,
            lambda:(0, {'name':'mcf'}), clock=lambda:self.t[0])

    def observe(self):
        self.feedback.observe(SimpleNamespace(mode=0,error_code=100,body_height=.32,velocity=[0.,0.,0.]))

    def test_constructing_does_not_acquire_or_send(self):
        self.assertEqual(self.events, [])
        self.assertFalse(self.client.armed)

    def test_nonzero_requires_operator_arm(self):
        with self.assertRaises(RuntimeError):self.client.Move(.1,0.,0.)
        self.assertEqual(self.events, [])

    def test_arm_keeps_avoidance_and_seeds_zero_before_ownership(self):
        self.client.arm()
        self.assertEqual(self.events, [('avoid_get',),('avoid_move',(0.,0.,0.)),
                                      ('source',True),('avoid_move',(0.,0.,0.))])
        self.assertTrue(self.client.api_owned)
        self.assertTrue(self.avoid.enabled)

    def test_velocity_routes_through_avoidance(self):
        self.client.arm();self.events.clear();self.client.Move(.1,0.,0.)
        self.assertEqual(self.events, [('avoid_move',(.1,0.,0.))])

    def test_disabled_avoidance_uses_sport_without_toggling_source(self):
        self.client.avoidance_enabled = self.avoid.enabled = False
        self.client.arm();self.events.clear();self.client.Move(.1,0.,0.)
        self.assertEqual(self.events, [('sport_move',(.1,0.,0.))])

    def test_stop_releases_source_and_requires_new_arm(self):
        self.client.arm();self.events.clear()
        self.assertEqual(self.client.StopMove(),0)
        self.assertEqual(self.events,[('sport_stop',),('source',False)])
        self.assertFalse(self.client.armed)
        self.assertFalse(self.client.api_owned)
        with self.assertRaises(RuntimeError):self.client.Move(.1,0.,0.)

    def test_rejected_sport_stop_is_not_hidden_by_successful_release(self):
        self.client.arm();self.sport.stop_code=-1
        self.assertEqual(self.client.StopMove(),-1)
        self.assertFalse(self.client.api_owned)

    def test_release_failure_latches_pulse_controller(self):
        self.client.arm()
        controller=PulseController(self.client,clock=lambda:self.t[0])
        controller.request((.1,0.,0.));controller.tick()
        self.avoid.release_code=77;self.t[0]=.31;self.observe();controller.tick()
        self.assertIsNotNone(controller.fault)
        self.assertTrue(self.client.api_owned)
        with self.assertRaises(RuntimeError):controller.request((.1,0.,0.))

    def test_sport_stop_exception_still_releases(self):
        self.client.arm()
        def fail():raise OSError('stop transport error')
        self.sport.StopMove=fail
        with self.assertRaises(OSError):self.client.StopMove()
        self.assertFalse(self.client.api_owned)
        self.assertIn(('source',False),self.events)

    def test_failed_acquire_attempts_release_without_nonzero_move(self):
        self.avoid.acquire_code=-1
        with self.assertRaises(RuntimeError):self.client.arm()
        self.assertIn(('source',False),self.events)
        self.assertFalse(self.client.api_owned)
        self.assertFalse(self.client.armed)
        self.assertFalse(any(any(e[1]) for e in self.events if e[0]=='avoid_move'))

    def test_zero_send_failure_prevents_ownership(self):
        self.avoid.move_code=3102
        with self.assertRaises(RuntimeError):self.client.arm()
        self.assertFalse(any(e[0]=='source' for e in self.events))

    def test_stale_feedback_prevents_arm(self):
        self.t[0]=.31
        with self.assertRaises(RuntimeError):self.client.arm()
        self.assertEqual(self.events,[])

    def test_stale_feedback_during_pulse_stops_and_releases(self):
        self.client.arm()
        controller=PulseController(self.client,clock=lambda:self.t[0])
        controller.request((.1,0.,0.),duration=.5);controller.tick()
        self.t[0]=.31;controller.tick()
        self.assertIsNotNone(controller.fault)
        self.assertFalse(self.client.api_owned)
        self.assertIn(('source',False),self.events)

    def test_arm_expiry_cannot_be_extended_by_move(self):
        self.client.arm();deadline=self.client.arm_deadline
        self.t[0]=5.01;self.observe()
        with self.assertRaises(RuntimeError):self.client.Move(.1,0.,0.)
        self.assertEqual(self.client.arm_deadline,deadline)

    def test_changed_avoidance_state_prevents_arm(self):
        self.avoid.enabled=False
        with self.assertRaises(RuntimeError):self.client.arm()
        self.assertFalse(self.client.armed)
        self.assertFalse(any(e[0]=='source' for e in self.events))

    def test_feedback_expiring_during_arm_releases_source(self):
        def delayed_mode():
            self.t[0]=.5
            return 0,{'name':'mcf'}
        self.client.check_mode=delayed_mode
        with self.assertRaises(RuntimeError):self.client.arm()
        self.assertFalse(self.client.armed)
        self.assertFalse(self.client.api_owned)
        self.assertIn(('source',False),self.events)

    def test_mode_mismatch_prevents_arm(self):
        self.client.check_mode=lambda:(0,{'name':'other'})
        with self.assertRaises(RuntimeError):self.client.arm()
        self.assertEqual(self.events,[])


if __name__=='__main__':unittest.main()
