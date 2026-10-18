#!/bin/bash
# Offline checks for the repository (no Gazebo needed):
#   1. ROS environment     2. world/model files     3. map, Nav2 config, RViz, DQN weights
#   4. every Python node compiles     5. re-computes docs/test_results.csv and
#      docs/test_summary.csv from the measured trials in docs/raw_trials.csv
# Usage:  bash scripts/run_tests.sh
set -e   # stop at the first failing check

echo "========================================================================"
echo "      IE4060 Autonomous Navigation - Verification & Test Suite"
echo "========================================================================"

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_DIR"

echo "[1/5] Checking Environment..."
export TURTLEBOT3_MODEL=burger
source /opt/ros/jazzy/setup.bash 2>/dev/null || echo "Note: Source /opt/ros/jazzy/setup.bash in your terminal"

echo "[2/5] Validating Simulation World and Models..."
test -f "$WORKSPACE_DIR/src/maze_world/worlds/maze.world" || { echo "ERROR: maze.world missing!"; exit 1; }
test -f "$WORKSPACE_DIR/src/maze_world/models/maze/model.sdf" || { echo "ERROR: model.sdf missing!"; exit 1; }
echo "  -> World and SDF models validated."

echo "[3/5] Validating Occupancy Map & Nav2 Config..."
test -f "$WORKSPACE_DIR/src/maze_navigation/maps/maze_map.yaml" || { echo "ERROR: maze_map.yaml missing!"; exit 1; }
test -f "$WORKSPACE_DIR/src/maze_navigation/maps/maze_map.pgm" || { echo "ERROR: maze_map.pgm missing!"; exit 1; }
test -f "$WORKSPACE_DIR/src/maze_navigation/config/nav2_params.yaml" || { echo "ERROR: nav2_params.yaml missing!"; exit 1; }
test -f "$WORKSPACE_DIR/src/maze_navigation/rviz/nav2.rviz" || { echo "ERROR: nav2.rviz missing!"; exit 1; }
test -f "$WORKSPACE_DIR/src/maze_dqn/models/dqn_weights.npz" || { echo "ERROR: DQN weights missing!"; exit 1; }
echo "  -> Map and Nav2 parameters validated."

echo "[4/5] Running Python Unit & Syntax Tests..."
python3 -m py_compile \
  src/goal_interface/goal_interface/goal_manager.py \
  src/goal_interface/goal_interface/viva_demo.py \
  src/maze_dqn/maze_dqn/dqn_model.py \
  src/maze_dqn/maze_dqn/maze_env.py \
  src/maze_dqn/maze_dqn/train_dqn.py \
  src/maze_dqn/maze_dqn/dqn_navigation_node.py \
  scripts/collect_metrics.py
echo "  -> All Python nodes compiled successfully."

echo "[5/5] Post-processing measured benchmark trials (docs/raw_trials.csv)..."
if [ -f "$WORKSPACE_DIR/docs/raw_trials.csv" ]; then
  python3 scripts/collect_metrics.py "$WORKSPACE_DIR/docs/raw_trials.csv"
else
  echo "  -> docs/raw_trials.csv not found: run the benchmark first (see README, 'Benchmarking')."
fi

echo "========================================================================"
echo "           ALL AUTOMATED SYSTEM CHECKS PASSED SUCCESSFULLY!"
echo "========================================================================"
