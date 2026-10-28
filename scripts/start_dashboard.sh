#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$EXCAVATOR_SAFETY_ROOT"
# A persistent control plane; deliberately independent of the simulation lifecycle.
if [[ ${1:-} != --serve ]]; then
 nohup "$PWD/scripts/start_dashboard.sh" --serve > /tmp/excavator_dashboard.log 2>&1 </dev/null &
 exit 0
fi
exec 8>.dashboard.lock
flock -w 5 8 || exit 0
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=33 GZ_PARTITION=excavator_safety
mkdir -p logs
echo $$ > dashboard.pid
exec python3 -m excavator_safety.ros.web_monitor
