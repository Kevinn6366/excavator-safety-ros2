#!/usr/bin/env bash
# Source this file from any directory; no ROS installation is needed for unit tests.
export EXCAVATOR_SAFETY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$EXCAVATOR_SAFETY_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
