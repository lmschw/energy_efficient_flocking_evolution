#!/usr/bin/env python3
"""usage: clock_offset.py <pi ip>   -> prints (Pi clock - laptop clock) in seconds, median of 7 ssh round trips,
each measured at the midpoint of its own round trip (accuracy ~ +-0.1 s)."""
import statistics, subprocess, sys, time
import fleet
ip = sys.argv[1]
key = fleet.KEY
cmd = ["ssh", "-i", key, "-o", "BatchMode=yes", "-o", "IdentitiesOnly=yes", "-o", "ConnectTimeout=6", f"{fleet.PI_USER}@{ip}",
       "python3 -c 'import time;print(repr(time.time()))'"]
offs = []
for _ in range(7):
    a = time.time(); out = subprocess.run(cmd, capture_output=True, text=True).stdout.strip(); b = time.time()
    if out:
        offs.append(float(out) - (a + b) / 2.0)
print(f"{statistics.median(offs):.3f}" if offs else "nan")
