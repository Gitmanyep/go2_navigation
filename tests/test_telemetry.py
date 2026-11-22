"""Passive measurement tests; no real DDS, control clients or robot connection."""
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace,ModuleType
import unittest
from unittest.mock import MagicMock,patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'control'))
from telemetry import TelemetryRecorder
from telemetry_monitor import connect


class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.t=[0.]
        self.recorder=TelemetryRecorder(Path(self.tmp.name)/'telemetry',clock=lambda:self.t[0])
    def tearDown(self):
        self.recorder.close();self.tmp.cleanup()
    def observe(self,velocity=(.3,.4,0.),position=(1.,2.,0.),yaw=0.,rate=.2):
        return self.recorder.observe(SimpleNamespace(velocity=velocity,position=position,
            yaw_speed=rate,imu_state=SimpleNamespace(rpy=[0.,0.,yaw])))

    def test_speed_uses_planar_norm_and_displacement_has_baseline(self):
        self.observe();self.t[0]=.1;self.observe(position=(1.3,2.4,0.))
        s=self.recorder.snapshot()
        self.assertAlmostEqual(s['peak_speed_mps'],.5)
        self.assertAlmostEqual(s['displacement_m'],.5)
        self.assertEqual(s['latest']['velocity'],[.3,.4,0.])

    def test_missing_optional_fields_are_unknown_instead_of_zero(self):
        self.recorder.observe(SimpleNamespace(velocity=[0.,0.,0.]))
        s=self.recorder.snapshot()
        self.assertIsNone(s['latest']['position']);self.assertIsNone(s['latest']['yaw_rate'])
        self.assertIsNone(s['displacement_m']);self.assertIsNone(s['yaw_change_rad'])

    def test_invalid_feedback_does_not_refresh_valid_sample(self):
        self.observe();self.t[0]=.31
        self.assertFalse(self.observe(velocity=(float('nan'),0.,0.)))
        s=self.recorder.snapshot()
        self.assertFalse(s['fresh']);self.assertEqual(s['invalid_samples'],1)
        self.assertEqual(s['latest']['received'],0.)

    def test_yaw_wrap_is_unwrapped_without_large_turn(self):
        self.observe(yaw=math.radians(179));self.t[0]=.1;self.observe(yaw=math.radians(-179))
        self.assertAlmostEqual(math.degrees(self.recorder.snapshot()['yaw_change_rad']),2.)

    def test_gap_does_not_invent_turn_across_missing_data(self):
        self.observe(yaw=0.);self.t[0]=1.;self.observe(yaw=2.)
        self.assertIsNone(self.recorder.snapshot()['yaw_change_rad'])

    def test_reset_preserves_csv_and_creates_new_measurement_baseline(self):
        self.observe();self.t[0]=1.;self.recorder.reset()
        self.observe(position=(3.,5.,0.),yaw=1.);self.t[0]=1.1;self.observe(position=(3.,5.,0.),yaw=1.2)
        s=self.recorder.snapshot()
        self.assertEqual(s['segment'],2);self.assertEqual(s['displacement_m'],0.)
        self.assertAlmostEqual(s['yaw_change_rad'],.2)
        self.recorder.stream.flush()
        with (self.recorder.output/'feedback.csv').open() as f:rows=list(csv.DictReader(f))
        self.assertEqual([r['segment'] for r in rows],['1','2','2'])

    def test_recording_is_throttled_to_twenty_hz(self):
        self.assertTrue(self.observe());self.t[0]=.01;self.assertFalse(self.observe())
        self.t[0]=.05;self.assertTrue(self.observe());self.assertEqual(self.recorder.snapshot()['samples'],2)

    def test_disk_write_failure_is_visible_and_does_not_claim_recording_success(self):
        class BadWriter:
            def writerow(self,_):raise OSError('disk full')
        self.recorder.writer=BadWriter();self.observe()
        self.assertIn('disk full',self.recorder.snapshot()['write_error'])
        self.assertIsNotNone(self.recorder.snapshot()['latest'])

    def test_summary_never_claims_independent_physical_verification(self):
        self.observe();summary=self.recorder.save_summary()
        self.assertFalse(summary['physical_motion_independently_verified'])
        self.assertEqual(summary['source'],'robot_internal_estimate')
        self.recorder.close();self.assertFalse(self.observe())

    def test_live_connection_only_constructs_a_subscriber(self):
        channel=ModuleType('unitree_sdk2py.core.channel')
        channel.ChannelFactoryInitialize=MagicMock();channel.ChannelSubscriber=MagicMock()
        state=ModuleType('unitree_sdk2py.idl.unitree_go.msg.dds_');state.SportModeState_=object()
        modules={'unitree_sdk2py.core.channel':channel,'unitree_sdk2py.idl.unitree_go.msg.dds_':state}
        callback=MagicMock()
        with patch.dict(sys.modules,modules):sub=connect(callback,'eth0')
        channel.ChannelFactoryInitialize.assert_called_once_with(0,'eth0')
        channel.ChannelSubscriber.assert_called_once_with('rt/sportmodestate',state.SportModeState_)
        sub.Init.assert_called_once_with(callback,1)

    def test_offline_launcher_records_demo_without_motion_backend(self):
        with tempfile.TemporaryDirectory() as folder:
            p=subprocess.run([str(ROOT/'bin/go2-nav'),'monitor-demo','--headless','--seconds','.15',
                              '--log-root',folder],capture_output=True,text=True)
            self.assertEqual(p.returncode,0,p.stdout+p.stderr)
            run=next(Path(folder).iterdir())
            self.assertFalse(json.loads((run/'metadata.json').read_text())['robot_motion_backend_enabled'])
            meta=json.loads((run/'telemetry/metadata.json').read_text())
            self.assertEqual(meta['source'],'synthetic');self.assertEqual(meta['control_apis_used'],[])
            summary=json.loads((run/'telemetry/segments.jsonl').read_text().splitlines()[0])
            self.assertEqual(summary['source'],'synthetic')
            with (run/'telemetry/feedback.csv').open() as f:self.assertGreaterEqual(len(list(csv.DictReader(f))),2)

    def test_demo_cannot_be_switched_to_live_and_output_cannot_override_provenance(self):
        for extra in (['--live'],['--output','/tmp/should-not-exist-go2-monitor']):
            with tempfile.TemporaryDirectory() as folder:
                p=subprocess.run([str(ROOT/'bin/go2-nav'),'monitor-demo','--log-root',folder,*extra],capture_output=True,text=True)
                self.assertEqual(p.returncode,2,p.stdout+p.stderr)
