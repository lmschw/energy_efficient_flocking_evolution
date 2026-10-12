#!/bin/bash
# Emergency stop: ends run_hebbian.py on every Pi (SIGTERM -> experiment.stop() -> motors 0).
source "$(dirname "$0")/fleet.sh"
for ip in "${IPS[@]}"; do
  ( ssh -i $KEY -o BatchMode=yes -o IdentitiesOnly=yes -o ConnectTimeout=5 $PI_USER@$ip 'pkill -TERM -f "[r]un_hebbian.py"; sleep 1.5; pgrep -f "[r]un_hebbian.py" >/dev/null && pkill -KILL -f "[r]un_hebbian.py"; sleep 0.5; cd ~/Desktop/crazy_thymio/lighthouse_deployment && timeout 15 ../.venv/bin/python tools/thymio_stop.py >/dev/null 2>&1; echo "$(hostname -I): stopped, motors 0"' 2>&1 | head -1 ) &
done
wait
