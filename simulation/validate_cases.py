"""Run isolated simulation cases concurrently; benchmark.py remains an isolated timing test."""
from pathlib import Path
import argparse
import concurrent.futures
import hashlib
import json
import os
import subprocess
import sys

ROOT=Path(__file__).resolve().parent


def source_hashes():
    return {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted([*ROOT.glob('*.py'),ROOT/'config.json'])}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True);p.add_argument('--duration',type=float,default=220.)
    p.add_argument('--scenarios');p.add_argument('--seeds',default='21');p.add_argument('--workers',type=int,default=4)
    p.add_argument('--worker',action='store_true');p.add_argument('--scenario');p.add_argument('--seed',type=int)
    a=p.parse_args();output=Path(a.output).resolve()
    if a.worker:
        from simulation import Simulation
        from core import config
        hashes=source_hashes();sim=Simulation(a.scenario,config(output.parent/'config.json'),a.seed)
        result=sim.run(a.duration);sim.save(output)
        (output/'source_hashes.json').write_text(json.dumps(hashes,indent=2))
        print(json.dumps(result));return 0
    from core import config
    from simulation import SCENARIOS
    cfg=config();output.mkdir(parents=True,exist_ok=False)
    (output/'config.json').write_text(json.dumps(cfg,indent=2))
    expected_hashes=source_hashes();(output/'source_hashes.json').write_text(json.dumps(expected_hashes,indent=2))
    names=a.scenarios.split(',') if a.scenarios else SCENARIOS;seeds=[int(s) for s in a.seeds.split(',')]
    if not all(n in SCENARIOS for n in names):p.error('unknown scenario')
    tasks=[(name,seed,f'{name}_seed{seed}' if len(seeds)>1 else name) for seed in seeds for name in names]
    def run(task):
        name,seed,label=task
        env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR='/tmp/go2-v3-mpl')
        cmd=[sys.executable,str(Path(__file__).resolve()),'--worker','--scenario',name,'--seed',str(seed),'--duration',str(a.duration),'--output',str(output/label)]
        done=subprocess.run(cmd,capture_output=True,text=True,env=env,cwd=ROOT)
        (output/(label+'.log')).write_text(done.stdout+done.stderr)
        if done.returncode:raise RuntimeError(label+': '+done.stderr[-1000:])
        if json.loads((output/label/'source_hashes.json').read_text())!=expected_hashes:raise RuntimeError('source_changed_during_validation')
        return json.loads((output/label/'summary.json').read_text())
    results={}
    with concurrent.futures.ThreadPoolExecutor(max_workers=a.workers) as pool:
        futures={pool.submit(run,task):task for task in tasks}
        for future in concurrent.futures.as_completed(futures):
            task=futures[future];result=future.result();results[(task[0],task[1])]=result
            print(json.dumps(result),flush=True)
    ordered=[results[(name,seed)] for name,seed,label in tasks]
    report=dict(all_passed=all(r['passed'] for r in ordered),version=3,validation_workers=a.workers,
                note='Parallel isolated functional cases; use benchmark.py for matched isolated timing.',
                results=ordered,motion_commands_sent=0)
    (output/'report.json').write_text(json.dumps(report,indent=2))
    # Reuse the existing plotter from exported traces without transferring live maps.
    from types import SimpleNamespace
    from simulation import World,plot_results
    sims=[]
    for name,seed,label in tasks:
        rows=[json.loads(line) for line in (output/label/'trace.jsonl').read_text().splitlines()]
        result=results[(name,seed)]
        sims.append(SimpleNamespace(name=label,world=World(name,dict(cfg,layout_seed=seed)),trace=rows,summary=lambda result=result:result))
    plot_results(sims,output/'overview.png');return 0 if report['all_passed'] else 1

if __name__=='__main__':raise SystemExit(main())
