#!/usr/bin/env python3
"""One explicit entry point; each invocation records source, command and output."""
import argparse
import datetime
import hashlib
import importlib
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]


def git(*args):
    p = subprocess.run(['git', '-C', str(ROOT), *args], stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, text=True)
    return p.stdout.strip() if p.returncode == 0 else None


def doctor():
    modules = {}
    for name in ('numpy', 'scipy', 'matplotlib', 'tkinter'):
        try:
            m = importlib.import_module(name)
            modules[name] = dict(available=True, version=getattr(m, '__version__', None),
                                 path=getattr(m, '__file__', None))
        except ImportError as exc:
            modules[name] = dict(available=False, error=str(exc))
    cfg = json.loads((ROOT/'simulation/config.json').read_text())
    result = dict(root=str(ROOT), commit=git('rev-parse', 'HEAD'),
                  dirty=bool(git('status', '--porcelain')), python=sys.executable,
                  architecture=platform.machine(), modules=modules,
                  noetic_setup=Path('/opt/ros/noetic/setup.bash').exists(),
                  calibration={k:cfg[k] for k in ('lidar_extrinsics_calibrated', 'geometry_calibrated')},
                  navigation_backend='preview_only', robot_commands_sent=0)
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return 0 if all(modules[n]['available'] for n in ('numpy', 'scipy')) else 1


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode', nargs='?', default='doctor', choices=(
        'doctor', 'test', 'sim', 'gui', 'preview', 'manual-preview', 'manual-live', 'monitor', 'monitor-demo', 'data-audit', 'mapping', 'perception-demo'))
    p.add_argument('--config', type=Path)
    p.add_argument('--log-root', type=Path, default=ROOT/'logs')
    a, extra = p.parse_known_args()
    if extra and extra[0] == '--':
        extra = extra[1:]
    perception_mode=a.mode in ('data-audit','mapping','perception-demo')
    cfg_path = (a.config or ROOT/('config/perception.json' if perception_mode else 'simulation/config.json')).resolve()
    if not cfg_path.is_file():
        p.error('config file missing: '+str(cfg_path))
    if a.mode in ('preview', 'sim', 'gui') and any(
            x in ('--mode', '--config', '--output') or x.startswith(('--mode=', '--config=', '--output=')) for x in extra):
        p.error('use the top-level mode/--config; output is managed by the launcher')
    if a.mode in ('monitor','monitor-demo','data-audit','mapping','perception-demo') and any(x in ('--output','--settings') or x.startswith(('--output=','--settings=')) for x in extra):
        p.error('output/settings are managed by the launcher')
    if a.mode == 'manual-preview' and '--live' in extra:
        p.error('use manual-live explicitly')
    now = datetime.datetime.now().astimezone()
    run = a.log_root.resolve()/(now.strftime('%Y%m%d_%H%M%S')+'_'+a.mode+'_'+uuid.uuid4().hex[:8])
    run.mkdir(parents=True, exist_ok=False)
    cfg_bytes = cfg_path.read_bytes()
    (run/'config.json').write_bytes(cfg_bytes)
    meta = dict(started_at=now.isoformat(), root=str(ROOT), commit=git('rev-parse', 'HEAD'),
                dirty=bool(git('status', '--porcelain')), argv=sys.argv, mode=a.mode,
                config=str(cfg_path), config_sha256=hashlib.sha256(cfg_bytes).hexdigest(),
                host=platform.node(), architecture=platform.machine(), python=sys.executable, launcher_pid=os.getpid(),
                robot_motion_backend_enabled=(a.mode == 'manual-live'))
    if a.mode in ('manual-live','monitor'):
        sdk = Path('/home/unitree/unitree_sdk2_python')
        meta['sdk_path'] = str(sdk)
        for key, args in [('sdk_commit', ['rev-parse', 'HEAD']), ('sdk_changes', ['status', '--porcelain'])]:
            r = subprocess.run(['git', '-C', str(sdk), *args], capture_output=True, text=True)
            meta[key] = r.stdout.strip() if r.returncode == 0 else None
    meta_path = run/'metadata.json'
    def save():
        meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=2)+'\n')
    save()
    print('SOURCE', ROOT, '\nCOMMIT', meta['commit'], '\nMODE', a.mode, '\nLOG', run, flush=True)
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONUNBUFFERED='1')
    env['MPLCONFIGDIR'] = str(run/'matplotlib')
    env.pop('PYTHONPATH', None)
    env.pop('PYTHONHOME', None)
    env['PYTHONNOUSERSITE'] = '1'
    if a.mode != 'gui':
        env.setdefault('MPLBACKEND', 'Agg')
    py = sys.executable
    nav = ROOT/'simulation'
    commands = []
    if a.mode == 'doctor':
        if extra: p.error('doctor takes no extra arguments')
        commands = [[py, str(Path(__file__).resolve()), '--doctor-child']]
    elif a.mode == 'test':
        if extra: p.error('test takes no extra arguments')
        commands = [[py, '-m', 'unittest', 'discover', '-s', str(nav), '-p', 'test*.py'],
                    [py, str(ROOT/'scripts/test_control.py')],
                    [py, '-m', 'unittest', 'discover', '-s', str(ROOT/'tests'), '-p', 'test*.py']]
    elif a.mode in ('sim', 'gui'):
        commands = [[py, str(nav/'simulation.py'), '--config', str(cfg_path),
                     *(['--gui'] if a.mode == 'gui' else ['--output', str(run/'simulation')]), *extra]]
    elif a.mode=='perception-demo':
        commands=[[py,str(ROOT/'perception/demo.py'),'--settings',str(cfg_path),'--output',str(run/'perception'),*extra]]
    elif a.mode in ('preview','data-audit','mapping'):
        setup = Path('/opt/ros/noetic/setup.bash')
        if not setup.exists():
            meta['error'] = 'ROS Noetic is required for this read-only onboard adapter'
            meta['exit_code'] = 2
            save()
            print(meta['error'], file=sys.stderr)
            return 2
        if a.mode=='preview':
            target=[py,str(nav/'ros_node.py'),'--mode','preview','--config',str(cfg_path),*extra]
        else:
            target=[py,str(ROOT/'perception/ros_node.py'),'--settings',str(cfg_path),'--output',str(run/'perception'),
                    *(['--audit-only'] if a.mode=='data-audit' else []),*extra]
        commands=[['/bin/bash','-c','source /opt/ros/noetic/setup.bash; exec "$@"','go2-nav',*target]]
    else:
        if a.mode in ('manual-live','monitor'):
            py = '/home/unitree/miniforge3/envs/rl/bin/python'
            env.pop('PYTHONNOUSERSITE', None)
            sdk = '/home/unitree/unitree_sdk2_python'
            # Pin source explicitly rather than depending on an editable install or ROS overlay.
            env['PYTHONPATH'] = sdk
            env['CYCLONEDDS_HOME'] = '/home/unitree/cyclonedds/install'
            env['LD_LIBRARY_PATH'] = '/home/unitree/cyclonedds/install/lib'+(':'+env['LD_LIBRARY_PATH'] if env.get('LD_LIBRARY_PATH') else '')
        if a.mode in ('monitor','monitor-demo'):
            commands = [[py,str(ROOT/'control/telemetry_monitor.py'),
                         '--live' if a.mode=='monitor' else '--demo','--output',str(run/'telemetry'),*extra]]
        else:
            commands = [[py, str(ROOT/'control/manual_panel.py'),
                         *(['--live'] if a.mode == 'manual-live' else []), *extra]]
    meta['commands'] = commands
    save()
    child = None
    stopped = [False]
    def forward(signum, frame):
        stopped[0] = True
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signum)
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, forward)
    code = 0
    try:
        with (run/'terminal.log').open('w', buffering=1) as log:
            for command in commands:
                if stopped[0]:
                    code = 130
                    break
                child = subprocess.Popen(command, cwd=str(ROOT), env=env, start_new_session=True,
                                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
                meta['child_pid'] = child.pid
                save()
                for line in child.stdout:
                    log.write(line)
                    try:
                        sys.stdout.write(line)
                        sys.stdout.flush()
                    except BrokenPipeError:
                        pass
                code = child.wait()
                if code: break
    except (OSError, ValueError) as exc:
        code = 2
        meta['error'] = str(exc)
        print(str(exc), file=sys.stderr)
    finally:
        meta.update(ended_at=datetime.datetime.now().astimezone().isoformat(), exit_code=code)
        save()
    return code


if __name__ == '__main__':
    raise SystemExit(doctor() if sys.argv[1:] == ['--doctor-child'] else main())
