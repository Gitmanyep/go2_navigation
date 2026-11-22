#!/usr/bin/env python3
"""Run the imported fake-client checks and propagate their report as an exit status."""
import json
from pathlib import Path
import runpy
import sys
control = Path(__file__).resolve().parents[1]/'control'
sys.path.insert(0, str(control))
runpy.run_path(str(control/'test_interfaces.py'), run_name='__main__')
report = json.loads((control/'interface_test_results.json').read_text())
raise SystemExit(0 if report['all_passed'] and report['case_count'] == 43 else 1)
