#!/bin/bash
# Phase 6 Part 2: full experiment matrix for the paper.
# Run order matches stated priority (S1+S4 first, the key comparison).
# Each experiment_runner invocation handles its own per-trial success/
# failure internally and appends to the shared CSV -- a failed trial is
# data, not a script error, so this does NOT use `set -e`.

source /home/f/aset_ws/install/setup.bash

run() {
  echo ""
  echo "=========================================="
  echo "  $(date): Starting $1 $2 x$3"
  echo "=========================================="
  ros2 run mobile_manipulator_core experiment_runner "$1" "$2" "$3"
}

run S1 llm 10
run S1 baseline 10
run S4 llm 9
run S4 baseline 10
run S2 llm 10
run S2 baseline 10
run S3 llm 10
run S5 llm 10

echo ""
echo "=========================================="
echo "  ALL TRIALS COMPLETE: $(date)"
echo "=========================================="
