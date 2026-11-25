"""Bounded ROS-only native terrain viewer. Publishes visualization, never commands."""
import argparse
from collections import deque
import json
from pathlib import Path
import sys
import threading
import time
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from native.rolling_map import NativeRollingMap
from perception.ros_messages import messages, pointcloud_xyz, terrain_overlay


def main():
    import rospy
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry, OccupancyGrid
    from geometry_msgs.msg import PoseStamped
    from visualization_msgs.msg import Marker
    from std_msgs.msg import Header, String
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    settings = json.loads((ROOT/'config/native_rolling.json').read_text())
    engine = NativeRollingMap(settings)
    rospy.init_node('go2_native_rolling_preview')
    prefix = '/go2/native_rolling/'
    pubs = {k: rospy.Publisher(prefix+k, cls, queue_size=1) for k, cls in [
        ('map', OccupancyGrid), ('traversability', OccupancyGrid), ('elevation', PointCloud2),
        ('obstacles', PointCloud2), ('body_pose', PoseStamped), ('terrain_overlay', Marker),
        ('warning', Marker), ('status', String)]}
    lock = threading.Lock()
    poses = deque(maxlen=100)
    latest = {}
    shared = dict(snapshot=None, error=None)
    finished = threading.Event()
    durations = deque(maxlen=300)
    drops = dict(odom=0, cloud=0)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'rolling_settings.json').write_text(json.dumps(settings, indent=2)+'\n')
    log = (args.output/'rolling_status.jsonl').open('w', buffering=1)
    input_log = (args.output/'rolling_inputs.jsonl').open('w', buffering=1)
    def receive(kind, msg):
        item = (msg, time.monotonic())
        with lock:
            if kind == 'odom':
                if len(poses) == poses.maxlen:
                    drops[kind] += 1
                poses.append(item)
            else:
                if kind in latest:
                    drops[kind] += 1
                latest[kind] = item
    subscribers = [
        rospy.Subscriber('/go2/native/odom', Odometry, lambda m: receive('odom', m), queue_size=100),
        rospy.Subscriber('/go2/native/cloud_deskewed', PointCloud2, lambda m: receive('cloud', m),
                         queue_size=1, buff_size=2**22)]
    def worker():
        while not finished.is_set():
            start = time.monotonic()
            with lock:
                batch = list(poses)
                poses.clear()
                cloud = latest.pop('cloud', None)
            try:
                for msg, mono in batch:
                    p, q = msg.pose.pose.position, msg.pose.pose.orientation
                    ok = engine.pose_input(msg.header.stamp.to_sec(), [p.x, p.y, p.z],
                        [q.x, q.y, q.z, q.w], msg.header.frame_id, msg.child_frame_id, mono)
                    input_log.write(json.dumps(dict(kind='odom', source_stamp=msg.header.stamp.to_sec(),
                        receipt_monotonic=mono, accepted=ok, reason=engine.errors['odom']))+'\n')
                if cloud:
                    msg, mono = cloud
                    if len(msg.data) > 32000000:
                        raise ValueError('oversized_cloud_buffer')
                    ok = engine.cloud_input(msg.header.stamp.to_sec(), pointcloud_xyz(msg), msg.header.frame_id, mono)
                    input_log.write(json.dumps(dict(kind='cloud', source_stamp=msg.header.stamp.to_sec(),
                        receipt_monotonic=mono, accepted=ok, reason=engine.errors['cloud']))+'\n')
                snap = engine.snapshot(time.monotonic())
                snap['valid_until_mono'] = min(engine.deadlines.values())
                snap['built_mono'] = time.monotonic()
                with lock:
                    durations.append(time.monotonic()-start)
                    shared.update(snapshot=snap, error=None)
            except Exception as exc:
                with lock:
                    shared['error'] = type(exc).__name__+':'+str(exc)
            finished.wait(max(0., settings['period_s']-(time.monotonic()-start)))
    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    next_visual = next_log = 0.
    try:
        while not rospy.is_shutdown():
            mono = time.monotonic()
            with lock:
                snap, error = shared['snapshot'], shared['error']
                timing, dropped = list(durations), dict(drops)
            if snap:
                snap = dict(snap)
                if error or mono > snap['valid_until_mono'] or mono-snap['built_mono'] > .4:
                    snap.update(valid=False, reason=error or 'native_input_or_worker_timeout',
                        occupancy=np.full_like(snap['occupancy'], -1), traversability=np.full_like(snap['traversability'], -1),
                        elevation_points=np.empty((0, 3)), obstacle_points=np.empty((0, 3)))
                status = dict(snap['audit'], valid=snap['valid'], reason=snap['reason'], support_valid=snap['support_valid'],
                    source_stamp=snap['source_stamp'], wall_time=time.time(),
                    map_origin=snap['origin'].tolist(), ground_cells=len(snap['elevation_points']),
                    obstacle_points=len(snap['obstacle_points']), nominal_free_cells=int(np.sum((snap['traversability'] >= 0) & (snap['traversability'] < 100))),
                    nominal_blocked_cells=int(np.sum(snap['traversability'] == 100)),
                    worker_p95_ms=float(np.quantile(timing, .95)*1000) if timing else None,
                    queue_drops=dropped, settings_source=str(ROOT/'config/native_rolling.json'))
                pubs['status'].publish(String(data=json.dumps(status, allow_nan=False)))
                if mono >= next_visual:
                    out = messages(snap)
                    header = Header(frame_id='odom', stamp=rospy.Time.from_sec(snap['source_stamp']))
                    out['terrain_overlay'] = terrain_overlay(snap, header)
                    warning = Marker(header=header, ns='native_rolling_warning', type=Marker.TEXT_VIEW_FACING)
                    warning.pose.orientation.w = 1.
                    if snap.get('body_pose'):
                        xyz = snap['body_pose']['position']
                        warning.pose.position.x, warning.pose.position.y = xyz[:2]
                        warning.pose.position.z = xyz[2]+.9
                    warning.scale.z = .10
                    warning.color.r = warning.color.g = warning.color.a = 1.
                    warning.text = 'NATIVE ROLLING | DISPLAY ONLY\n%s | nominal clearance; no body calibration'%snap['reason']
                    out['warning'] = warning
                    for k, msg in out.items():
                        pubs[k].publish(msg)
                    next_visual = mono+settings['publish_period_s']
                if mono >= next_log:
                    log.write(json.dumps(status, allow_nan=False)+'\n')
                    print('NATIVE_ROLLING', status['reason'], 'ground', status['ground_cells'],
                          'obstacles', status['obstacle_points'], 'p95_ms', status['worker_p95_ms'], flush=True)
                    next_log = mono+1.
            time.sleep(.05)
    finally:
        finished.set()
        thread.join(timeout=3.)
        if not thread.is_alive():
            input_log.close()
        log.close()


if __name__ == '__main__':
    main()
