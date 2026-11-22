"""Local wall-clock preview watchdog probe under a CPU-busy 500ms worker stall.
This tests publication scheduling outside ROS; it does not validate onboard 10Hz.
"""
import argparse
import json
import math
import time
from pathlib import Path
from runtime import PreviewRuntime


def run():
    origin=time.monotonic();calls=[0];stalls=[]
    def processor(events):
        calls[0]+=1
        if calls[0]==2:
            begin=time.monotonic();end=begin+.5;value=0.
            while time.monotonic()<end:value=math.sin(value+1.)
            stalls.append(dict(start=begin-origin,end=time.monotonic()-origin))
        return dict(state=dict(reason='tracking',candidate=[.1,0.],path=[])),time.monotonic()+.5
    runtime=PreviewRuntime(processor,period=.1,lease=.15)
    runtime.compute_once();runtime.start();rows=[];next_tick=origin
    try:
        while time.monotonic()-origin<1.2:
            started=time.monotonic();snap=runtime.read()
            rows.append(dict(t=started-origin,candidate=snap['state']['candidate'],reason=snap['state']['reason'],read_ms=(time.monotonic()-started)*1000))
            next_tick+=.1;time.sleep(max(0.,next_tick-time.monotonic()))
    finally:runtime.close()
    gaps=[b['t']-a['t'] for a,b in zip(rows,rows[1:])]
    firstzero=next((r['t'] for r in rows if r['candidate']==[0.,0.]),None)
    return dict(passed=bool(stalls and firstzero is not None and firstzero<=.26 and max(gaps)<.15),
                scope='local non-ROS threaded preview publisher; NOT hardware or ROS timing validation',
                worker_stalls=stalls,publisher_period_s=.1,candidate_lease_s=.15,
                first_zero_s=firstzero,max_publication_gap_ms=max(gaps)*1000,
                max_read_ms=max(r['read_ms'] for r in rows),samples=rows,
                real_time_10hz_verified=False,motion_commands_sent=0)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True);a=p.parse_args()
    result=run();Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps(result));raise SystemExit(0 if result['passed'] else 1)
