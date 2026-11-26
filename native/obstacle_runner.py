"""Attach obstacle-only route visualization to an ALREADY running ROS voxel feed."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xmlrpc.client
ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float)
    parser.add_argument('--no-rviz',action='store_true')
    parser.add_argument('--settings',type=Path,default=ROOT/'config/native_obstacle_plan.json')
    a=parser.parse_args()
    if a.seconds is not None and not 0<a.seconds<=3600:parser.error('seconds outside 0..3600')
    cfg=json.loads((ROOT/'config/native_lidar.json').read_text())
    run=ROOT/'logs'/('obstacle_only_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    run.mkdir(parents=True,exist_ok=False)
    env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:11311',ROS_IP='127.0.0.1',
        PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',DISPLAY=cfg['display'],XAUTHORITY=str(Path.home()/'.Xauthority'))
    for key in ('ROS_HOSTNAME','PYTHONHOME','PYTHONPATH'):env.pop(key,None)
    stop=False;children=[];streams=[];commands=[];code=0
    def stopped(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGTERM,stopped);signal.signal(signal.SIGINT,stopped)
    def spawn(name,cmd):
        command=['/bin/bash','-c','source /opt/ros/noetic/setup.bash; exec "$@"','go2-obstacle-preview',*cmd]
        stream=(run/(name+'.log')).open('w');streams.append(stream)
        child=subprocess.Popen(command,cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(child);commands.append(dict(name=name,command=command,pid=child.pid))
        return child
    try:
        try:alive=xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/obstacle_preview_launcher')[0]==1
        except (OSError,xmlrpc.client.Error):alive=False
        if not alive:raise RuntimeError('Start the existing native voxel display / ROS master first; this launcher only subscribes.')
        node=spawn('planner',[sys.executable,str(ROOT/'native/obstacle_node.py'),'--settings',str(a.settings.resolve()),'--output',str(run)])
        if not a.no_rviz:spawn('rviz',['rviz','-d',str(ROOT/'config/native_obstacle_plan.rviz')])
        paths=['native/obstacle_plan.py','native/obstacle_node.py','native/obstacle_runner.py','config/native_obstacle_plan.rviz']
        hashes={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths}
        hashes[str(a.settings.resolve())]=hashlib.sha256(a.settings.read_bytes()).hexdigest()
        rev=subprocess.run(['git','rev-parse','HEAD'],cwd=ROOT,text=True,capture_output=True).stdout.strip()
        (run/'metadata.json').write_text(json.dumps(dict(source=str(ROOT),commit=rev,sha256=hashes,commands=commands,
            settings=json.loads(a.settings.read_text()),mode='obstacle_only_assumed_flat_preview',motion_commands_sent=0),indent=2))
        print('OBSTACLE_ONLY_PREVIEW','SOURCE',ROOT,'LOG',run,flush=True)
        start=time.monotonic()
        while not stop and (a.seconds is None or time.monotonic()-start<a.seconds):
            if node.poll() is not None:raise RuntimeError('Preview exited: '+str(run/'planner.log'))
            time.sleep(.2)
    except Exception:
        code=1
        raise
    finally:
        for child in reversed(children):
            if child.poll() is None:os.killpg(child.pid,signal.SIGTERM)
        for child in children:
            try:child.wait(timeout=4)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        for stream in streams:stream.close()
        (run/'exit.json').write_text(json.dumps(dict(exit_code=code,time=datetime.datetime.now().isoformat(),motion_commands_sent=0)))
    return code


if __name__=='__main__':raise SystemExit(main())
