#!/usr/bin/env python3
"""
update_plots.py
Run all Python plotting scripts in the current directory (except this file).
"""

import subprocess
import sys
from pathlib import Path

def main() -> int:
    here = Path.cwd()
    self_name = Path(__file__).name

    scripts = sorted(p for p in here.glob("*.py") if p.name != self_name)

    for script in scripts:
        print(f"Running {script.name} ...")
        rc = subprocess.call([sys.executable, str(script)])
        if rc != 0:
            print(f"ERROR: {script.name} failed (exit {rc})")
            return rc

    print("Done.")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
