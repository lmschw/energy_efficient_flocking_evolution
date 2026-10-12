#!/bin/bash
# usage: tools/where.sh <robot 1..7>   -> prints that robot's steady position (hold it still ~3 s first)
N=$1
source "$(dirname "$0")/fleet.sh"
IP=${N:+${IPS[$N]:-}}; [ -n "$IP" ] || { echo "usage: $0 <1..7>"; exit 1; }
HERE="$(cd "$(dirname "$0")/.." && pwd)"
HOSTS=robot-1,robot-2,robot-3,robot-4,robot-5,robot-6,robot-7; IDS=1,233,235,232,236,226,234
rsync -az -e "ssh -i $KEY -o BatchMode=yes -o IdentitiesOnly=yes" --exclude __pycache__ --exclude cache --exclude sessions "$HERE/" $PI_USER@$IP:~/Desktop/crazy_thymio/lighthouse_deployment/
ssh -i $KEY -o BatchMode=yes -o IdentitiesOnly=yes $PI_USER@$IP "cd ~/Desktop/crazy_thymio/lighthouse_deployment && ../.venv/bin/python -u run_hebbian.py --self-hostname robot-$N --hostnames $HOSTS --ids $IDS --where 2>&1 | grep -E 'WHERE|No steady'"
