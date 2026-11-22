"""Verify isolation and provenance across the real entry point, without ROS or SDK."""
import ast
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


class DeploymentTests(unittest.TestCase):
    def test_offline_dependency_lock_is_shipped(self):
        lock = ROOT/'config/requirements-py38.lock'
        self.assertTrue(lock.is_file())
        self.assertIn('numpy==1.24.4', lock.read_text())
        ignored = subprocess.run(['git', '-C', str(ROOT), 'check-ignore', str(lock)], capture_output=True)
        self.assertNotEqual(ignored.returncode, 0)

    def test_doctor_logs_exact_source_and_config(self):
        with tempfile.TemporaryDirectory() as folder:
            p = subprocess.run([str(ROOT/'bin/go2-nav'), 'doctor', '--log-root', folder],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            self.assertEqual(p.returncode, 0, p.stdout)
            runs = list(Path(folder).iterdir())
            self.assertEqual(len(runs), 1)
            meta = json.loads((runs[0]/'metadata.json').read_text())
            self.assertEqual(meta['root'], str(ROOT))
            self.assertEqual(meta['mode'], 'doctor')
            self.assertFalse(meta['robot_motion_backend_enabled'])
            self.assertEqual(meta['exit_code'], 0)
            self.assertEqual((runs[0]/'config.json').read_bytes(), (ROOT/'simulation/config.json').read_bytes())
            self.assertIn('preview_only', (runs[0]/'terminal.log').read_text())

    def test_preview_rejects_mode_override(self):
        with tempfile.TemporaryDirectory() as folder:
            p = subprocess.run([str(ROOT/'bin/go2-nav'), 'preview', '--log-root', folder,
                                '--mode', 'synthetic'], capture_output=True, text=True)
            self.assertEqual(p.returncode, 2)
            self.assertEqual(list(Path(folder).iterdir()), [])

    def test_manual_preview_cannot_enable_live(self):
        p = subprocess.run([str(ROOT/'bin/go2-nav'), 'manual-preview', '--live'], capture_output=True, text=True)
        self.assertEqual(p.returncode, 2)

    def test_navigation_has_no_hardware_import(self):
        for source in (ROOT/'simulation').glob('*.py'):
            tree = ast.parse(source.read_text())
            for node in ast.walk(tree):
                names = [a.name for a in node.names] if isinstance(node, ast.Import) else [node.module or ''] if isinstance(node, ast.ImportFrom) else []
                for name in names:
                    self.assertFalse(name.startswith(('unitree_sdk2', 'cyclonedds')), (source.name, name))
        text = (ROOT/'simulation/ros_node.py').read_text()
        self.assertNotIn('/cmd_vel', text)
        self.assertIn("prefix='/go2/framework_v3'", text)

    def test_calibration_stays_unmeasured(self):
        cfg = json.loads((ROOT/'simulation/config.json').read_text())
        self.assertFalse(cfg['geometry_calibrated'])
        self.assertFalse(cfg['lidar_extrinsics_calibrated'])

    def test_ros_package_points_to_unique_entry(self):
        package = ET.parse(ROOT/'package.xml').getroot()
        self.assertEqual(package.find('name').text, 'go2_navigation')
        launch = ET.parse(ROOT/'launch/pointlio_preview.launch').getroot().find('node')
        self.assertEqual(launch.attrib['pkg'], 'go2_navigation')
        self.assertEqual(launch.attrib['type'], 'go2-navigation-node')


if __name__ == '__main__':
    unittest.main()
