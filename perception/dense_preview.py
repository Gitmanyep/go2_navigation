#!/usr/bin/env python3
"""Supervise only the dense display converter and uncalibrated return preview."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--settings',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float)
    a=p.parse_args()
    if a.seconds is not None and (not __import__('math').isfinite(a.seconds) or a.seconds<=0):p.error('invalid seconds')
    a.output.mkdir(parents=True,exist_ok=False)
    children=[];streams=[];stopped=[False]
    def stop(sig,frame):stopped[0]=True
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,stop)
    commands=[('dense',[sys.executable,str(ROOT/'perception/dense_node.py'),'--settings',str(a.settings),'--output',str(a.output/'dense')]),
              ('preview',[sys.executable,str(ROOT/'perception/ros_node.py'),'--reference-preview','--settings',str(a.settings),'--output',str(a.output/'preview')])]
    meta=dict(mode='dense_visualization_only',commands=commands,motion_commands_sent=0,control_ownership_changed=False)
    code=0;started=time.monotonic()
    try:
        for name,command in commands:
            log=(a.output/(name+'_terminal.log')).open('w',buffering=1);streams.append(log)
            child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT);children.append(child)
            print('DENSE_PREVIEW_CHILD',name,'PID',child.pid,'LOG',str(a.output/(name+'_terminal.log')),flush=True)
        meta['child_pids']=[c.pid for c in children]
        (a.output/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
        while not stopped[0] and (a.seconds is None or time.monotonic()-started<a.seconds):
            failed=[c for c in children if c.poll() is not None]
            if failed:code=failed[0].returncode or 1;break
            time.sleep(.1)
    finally:
        for c in children:
            if c.poll() is None:c.terminate()
        for c in children:
            try:c.wait(timeout=4.)
            except subprocess.TimeoutExpired:c.kill();c.wait()
        for log in streams:log.close()
        meta.update(exit_code=code,child_exit_codes=[c.returncode for c in children],duration_s=time.monotonic()-started)
        (a.output/'report.json').write_text(json.dumps(meta,indent=2)+'\n')
    return code

if __name__=='__main__':raise SystemExit(main())
