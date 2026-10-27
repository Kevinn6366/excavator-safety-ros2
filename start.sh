#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
exec 9>.run.lock
flock -w 5 9 || { echo "本项目已在运行"; exit 1; }
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-33}
export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}
export DISPLAY=${DISPLAY:-:0}
# Inherit XAUTHORITY from the active desktop session.
export GZ_PARTITION=excavator_safety
mkdir -p logs data history
if [[ -f data/events.jsonl ]]; then mv data "history/run_$(date +%Y%m%d_%H%M%S)"; mkdir data; fi
python3 import_model.py
python3 build_project.py
children=()
cleanup(){
 trap - EXIT INT TERM
 for p in "${children[@]}"; do kill -INT -- "-$p" 2>/dev/null || true; done
 sleep 2
 for p in "${children[@]}"; do kill -KILL -- "-$p" 2>/dev/null || true; done
 wait 2>/dev/null || true
 python3 cleanup_sim.py
 rm -f run.pid
}
echo $$ > run.pid
trap cleanup EXIT INT TERM
# Dedicated domain and child PIDs isolate this project from earlier coursework.
if [[ ${1:-} == --gui ]]; then
 setsid gz sim -r worlds/safety.sdf >logs/gazebo.log 2>&1 9>&- &
else
 setsid gz sim -s -r worlds/safety.sdf >logs/gazebo.log 2>&1 9>&- &
fi
children+=($!)
setsid /opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge --ros-args -p config_file:="$PWD/bridge.yaml" >logs/bridge.log 2>&1 9>&- &
children+=($!)
setsid python3 safety_node.py --ros-args -p use_sim_time:=true >logs/safety.log 2>&1 9>&- &
children+=($!)
./start_dashboard.sh 9>&-
wait -n "${children[@]}"
