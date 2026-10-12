#!/bin/bash
# usage: tools/identify.sh <robot 1..7> [times]   -- makes that robot's Thymio WIGGLE left-right in place (default 3x) through
# its own Pi, so you can spot it. No radio involved, so it is safe at any time (the Crazyflie is not touched).
# (A Thymio top-LED colour set by a short script does NOT stay on -- tested 2026-10-04 -- so the wiggle is used instead.)
N=$1
source "$(dirname "$0")/fleet.sh"
IP=${N:+${IPS[$N]:-}}; [ -n "$IP" ] || { echo "usage: $0 <1..7> [times]"; exit 1; }
HERE="$(cd "$(dirname "$0")/.." && pwd)"
scp -q -i $KEY -o BatchMode=yes -o IdentitiesOnly=yes "$HERE/tools/thymio_wiggle.py" $PI_USER@$IP:~/Desktop/crazy_thymio/lighthouse_deployment/tools/
ssh -i $KEY -o BatchMode=yes -o IdentitiesOnly=yes $PI_USER@$IP "cd ~/Desktop/crazy_thymio/lighthouse_deployment && timeout 30 ../.venv/bin/python tools/thymio_wiggle.py ${2:-3}" | sed "s/^wiggled$/robot $N wiggled/"
