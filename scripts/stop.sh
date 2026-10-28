#!/usr/bin/env bash
set -e
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$EXCAVATOR_SAFETY_ROOT"
if [[ -f run.pid ]]; then
 pid=$(cat run.pid)
 if [[ $(ps -p "$pid" -o args=) == *start.sh* ]]; then kill -TERM "$pid"; fi
fi

python3 -m excavator_safety.simulation.cleanup
