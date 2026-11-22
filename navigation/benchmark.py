"""Compare identical local synthetic workloads; not an onboard timing certification."""
import argparse
import json
import platform
import sys
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',default=str(Path(__file__).parent),help='Framework source directory, e.g. v2 or v3')
    p.add_argument('--steps',type=int,default=120);p.add_argument('--scenario',default='overhead_branch')
    p.add_argument('--output',required=True);a=p.parse_args()
    sys.path.insert(0,str(Path(a.source).resolve()))
    from simulation import Simulation
    sim=Simulation(a.scenario)
    for _ in range(a.steps):sim.step()
    nav=np.array([s['navigation_s'] for s in sim.compute_times])*1000
    whole=np.array([s['total_s'] for s in sim.compute_times])*1000
    def metrics(values):return dict(p50=float(np.median(values)),p95=float(np.quantile(values,.95)),max=float(values.max()))
    report=dict(source=str(Path(a.source).resolve()),steps=a.steps,scenario=a.scenario,python=platform.python_version(),numpy=np.__version__,
                navigation_ms=metrics(nav),whole_tick_ms=metrics(whole),collisions=sim.collisions,
                real_time_10hz_verified=False,motion_commands_sent=0)
    Path(a.output).write_text(json.dumps(report,indent=2));print(json.dumps(report))

if __name__=='__main__':main()
