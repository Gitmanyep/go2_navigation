"""Summarize completed offline checks; never equate them with hardware/ROS acceptance."""
from pathlib import Path
import hashlib
import json
import re


def main():
    root=Path(__file__).parent;v=root/'validation'
    unit=(v/'regression_tests.txt').read_text();match=re.search(r'Ran (\d+) tests',unit)
    unit_ok=bool(match and int(match.group(1))>=78 and unit.rstrip().endswith('OK'))
    suite=json.loads((v/'verified_suite/report.json').read_text())
    seeds=json.loads((v/'verified_seed_sweep/report.json').read_text())
    audits=[json.loads((v/f'{name}/truth_geometry_audit.json').read_text()) for name in ['verified_suite','verified_seed_sweep']]
    watchdog=json.loads((v/'watchdog_probe.json').read_text())
    before=json.loads((v/'benchmark_v2.json').read_text());after=json.loads((v/'benchmark_v3.json').read_text())
    hashes=json.loads((v/'final_source_hashes.json').read_text())
    source_ok=all(hashlib.sha256((root/name).read_bytes()).hexdigest()==digest for name,digest in hashes.items())
    results=suite['results']+seeds['results'];audit_rows=[row for audit in audits for row in audit['results']]
    checks=dict(unit_tests=unit_ok,main_25_scenarios=len(suite['results'])==25 and suite['all_passed'],
                six_seed_cases=len(seeds['results'])==6 and seeds['all_passed'],
                independent_geometry_audit=len(audit_rows)==31 and all(a['all_clear'] for a in audits),
                independent_watchdog_probe=watchdog['passed'],source_snapshot_matches=source_ok)
    report=dict(version=3,all_passed=all(checks.values()),checks=checks,unit_tests=int(match.group(1)) if match else 0,
                scenario_cases=len(results),geometry_audit_cases=len(audit_rows),
                approach_proximity_cases=sum(r['expected_behavior']=='directional_approach' and r['passed'] for r in results),
                safe_rejection_cases=sum(r['expected_behavior']=='safe_rejection' and r['passed'] for r in results),
                operator_confirmed_completions=sum(r['expected_behavior']=='operator_confirmed_completion' and r['task_success'] for r in results),
                direction_only_erroneous_completions=sum(r['task_completed'] for r in results if r['expected_behavior']=='directional_approach'),
                total_collision_samples=sum(r['collision_samples'] for r in results),
                total_unsafe_slope_samples=sum(r['unsafe_slope_samples'] for r in results),
                benchmark_120_steps=dict(v2=before,v3=after),watchdog=watchdog,
                maximum_case_navigation_ms=max(r['navigation_runtime_ms']['maximum'] for r in results),
                failed_cases=[r['scenario'] for r in results if not r['passed']],
                real_time_10hz_verified=False,noetic_runtime_verified=False,gui_display_verified=False,
                real_pointlio_bag_replayed=False,hardware_verified=False,motion_commands_sent=0,
                limitations=['Unmeasured head frame and sensor extrinsics; back mount is provisional.',
                             'Local rolling evidence memory, 2.5D terrain and kinematics; no contact/slip/load dynamics.',
                             'Free-volume certification API is not inferred from missing point-cloud returns.',
                             'Watchdog probe is local Python scheduling, not ROS or onboard timing certification.'])
    (v/'final_acceptance.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps({k:report[k] for k in ['all_passed','checks','unit_tests','scenario_cases','geometry_audit_cases','maximum_case_navigation_ms']}))
    return 0 if report['all_passed'] else 1

if __name__=='__main__':raise SystemExit(main())
