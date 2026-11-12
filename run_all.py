"""
run_all.py - Recreate every figure (figures/) and every number (results/) in one go.
Run from the repo root:  python run_all.py        (takes about 30-60 seconds)
"""

import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))

SCRIPTS = [
    "check_theory",
    "section01_loop_diagram",
    "section02_pid_terms",
    "section03_test_system",
    "section04_metrics",
    "section05_each_part",
    "section06_all_together",
    "section07_real_world",
    "section08_ziegler_nichols",
    "section09_full_demo",
]

start = time.time()
for name in SCRIPTS:
    print(f"\n=== experiments/{name}.py ===")
    subprocess.run([sys.executable, "-m", f"experiments.{name}"], check=True, cwd=ROOT)
print(f"\nAll done in {time.time() - start:.0f} s. Figures are in figures/, numbers in results/.")
