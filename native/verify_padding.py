"""Read-only same-stamp byte-preservation check of native cloud cleanup.

Run while bin/go2-native is active, with ROS Noetic sourced.
"""
import json
import time
import threading
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from perception.ros_messages import pointcloud_xyz

rospy.init_node('native_padding_fix_acceptance', anonymous=True)
lock = threading.Lock()
samples, bridge = {}, {}
mapstates = []
def receive(msg, kind):
    key = (msg.header.stamp.secs, msg.header.stamp.nsecs)
    with lock:
        if key not in samples and len(samples) >= 200:
            samples.pop(next(iter(samples)))
        samples.setdefault(key, {})[kind] = msg
def bridge_state(msg):
    with lock:
        bridge.update(json.loads(msg.data))
def map_state(msg):
    with lock:
        mapstates.append(json.loads(msg.data))
subs = [
    rospy.Subscriber('/go2/native/cloud_deskewed_unfiltered', PointCloud2, lambda m: receive(m, 'input'), queue_size=2, buff_size=2**22),
    rospy.Subscriber('/go2/native/cloud_deskewed', PointCloud2, lambda m: receive(m, 'output'), queue_size=2, buff_size=2**22),
    rospy.Subscriber('/go2/native/status', String, bridge_state, queue_size=1),
    rospy.Subscriber('/go2/native_rolling/status', String, map_state, queue_size=1)]
time.sleep(5)
for sub in subs:
    sub.unregister()
with lock:
    paired = list(samples.items())
rows = []
for stamp, pair in paired:
    if set(pair) != {'input', 'output'}:
        continue
    a, b = pair['input'], pair['output']
    if a.width*a.height == 0:
        continue
    xyz = pointcloud_xyz(a)
    finite = np.isfinite(xyz).all(axis=1)
    zero = np.all(xyz == 0, axis=1)
    keep = np.flatnonzero(finite & ~zero)
    blob = bytes(a.data)
    parts = []
    for i in keep:
        start = (int(i)//a.width)*a.row_step+(int(i)%a.width)*a.point_step
        parts.append(blob[start:start+a.point_step])
    expected = b''.join(parts)
    outxyz = pointcloud_xyz(b)
    fields = lambda msg: [(f.name, f.offset, f.datatype, f.count) for f in msg.fields]
    passed = (
        bytes(b.data) == expected and b.width == len(keep) and b.height == 1 and
        b.row_step == len(keep)*b.point_step and a.header.frame_id == b.header.frame_id and
        a.is_bigendian == b.is_bigendian and a.point_step == b.point_step and fields(a) == fields(b) and
        not np.any(np.all(outxyz == 0, axis=1)) and np.isfinite(outxyz).all())
    rows.append({
        'stamp': list(stamp), 'input_points': len(xyz), 'input_zeros': int(zero.sum()),
        'output_points': len(outxyz), 'output_zeros': int(np.sum(np.all(outxyz == 0, axis=1))),
        'input_bytes': len(a.data), 'output_bytes': len(b.data),
        'valid_records_byte_identical': bool(passed)})
report = {
    'time': time.strftime('%Y-%m-%d %H:%M:%S'),
    'paired_frames': len(rows),
    'all_passed': bool(rows) and all(r['valid_records_byte_identical'] for r in rows),
    'output_zeros': sum(r['output_zeros'] for r in rows),
    'input_zeros': sum(r['input_zeros'] for r in rows),
    'output_points_min': min((r['output_points'] for r in rows), default=0),
    'output_points_max': max((r['output_points'] for r in rows), default=0),
    'payload_reduction_fraction': 1-sum(r['output_bytes'] for r in rows)/max(1, sum(r['input_bytes'] for r in rows)),
    'bridge_cleanup': bridge.get('details', {}).get('cloud_deskewed', {}),
    'bridge_errors': bridge.get('errors'),
    'rolling_map_valid_samples': sum(bool(s.get('valid')) for s in mapstates),
    'rolling_map_samples': len(mapstates),
    'last_map_state': mapstates[-1] if mapstates else None,
    'motion_commands_sent': 0, 'frames': rows}
f = ROOT/'logs'/('native_zero_cleanup_acceptance_'+time.strftime('%Y%m%d_%H%M%S')+'.json')
f.write_text(json.dumps(report, indent=2))
print(json.dumps({k: v for k, v in report.items() if k != 'frames'}, indent=2))
print('REPORT', f)
if not report['all_passed'] or len(rows) < 10:
    raise SystemExit(1)
