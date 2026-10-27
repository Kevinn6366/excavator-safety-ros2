#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
if [[ -f run.pid ]]; then
 pid=$(cat run.pid)
 if [[ $(ps -p "$pid" -o args=) == *start.sh* ]]; then kill -TERM "$pid"; fi
fi

python3 cleanup_sim.py
