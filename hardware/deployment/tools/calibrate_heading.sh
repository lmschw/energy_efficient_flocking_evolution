#!/bin/bash
# usage: tools/calibrate_heading.sh <robot number 1..7>
# Runs the heading / motor-speed calibration on that robot's Pi and appends the result to
# lighthouse_config/heading_calibration.csv. The robot DRIVES ~0.2-0.4 m straight ahead (the Thymio's
# front, where its proximity sensors are) -- put it facing open floor, stations in view, lights off.
set -e
N=$1
source "$(dirname "$0")/fleet.sh"
IP=${N:+${IPS[$N]:-}}; [ -n "$IP" ] || { echo "usage: $0 <1..7>"; exit 1; }
HERE="$(cd "$(dirname "$0")/.." && pwd)"
HOSTS=robot-1,robot-2,robot-3,robot-4,robot-5,robot-6,robot-7; IDS=1,233,235,232,236,226,234
rsync -az -e "ssh -i $KEY -o BatchMode=yes -o IdentitiesOnly=yes" --exclude __pycache__ --exclude cache --exclude sessions "$HERE/" $PI_USER@$IP:~/Desktop/crazy_thymio/lighthouse_deployment/
OUT=$(ssh -i $KEY -o BatchMode=yes -o IdentitiesOnly=yes $PI_USER@$IP "cd ~/Desktop/crazy_thymio/lighthouse_deployment && ../.venv/bin/python -u run_hebbian.py --self-hostname robot-$N --hostnames $HOSTS --ids $IDS --calibrate-heading 2>&1 | grep -v 'no LogEntry'")
echo "$OUT"
LINE=$(echo "$OUT" | grep '^RESULT ' | tail -1)
if [ -n "$LINE" ]; then
  mkdir -p "$HERE/lighthouse_config"; F="$HERE/lighthouse_config/heading_calibration.csv"
  [ -f "$F" ] || echo "timestamp,robot,heading_offset_rad,motor_units_per_mps,distance_m" > "$F"
  echo "$(date -Is),robot-$N,$(echo $LINE | sed -E 's/.*heading_offset=([-0-9.]+).*/\1/'),$(echo $LINE | sed -E 's/.*motor_units_per_mps=([0-9.]+).*/\1/'),$(echo $LINE | sed -E 's/.*distance_m=([0-9.]+).*/\1/')" >> "$F"
  echo "saved to $F"
fi
