# Sourced by the bash tools: reads the fleet settings (fleet.conf, or the file in $FLEET_CONF) and defines
# PI_USER, KEY (ssh key) and IPS (robot number -> Pi IP address).
source "${FLEET_CONF:-$(dirname "${BASH_SOURCE[0]}")/fleet.conf}"
KEY=$PI_KEY
declare -gA IPS=()
for _n in 1 2 3 4 5 6 7; do _v=ROBOT_$_n; IPS[$_n]=${!_v}; done
