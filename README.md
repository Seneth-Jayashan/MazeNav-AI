# IE4060 Assignment 02: Autonomous Navigation of TurtleBot3 Burger in Gazebo (ROS 2 Jazzy)

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Simulator](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache_2.0-green.svg)](LICENSE)

This repository contains the complete, reproducible ROS 2 Jazzy implementation for **IE4060: Robotics and Intelligent Systems — Assignment 02** at the Sri Lanka Institute of Information Technology (SLIIT).

The project implements a complete autonomous mobile robot navigation pipeline for the **TurtleBot3 Burger** operating within an authentic $6 \times 6$ grid maze ($3.0\text{ m} \times 3.0\text{ m}$) matching the assignment specification (Figure 1).

---

## 📁 Repository Structure

```text
ie4060_assignment02/
├── README.md
├── docs/
│   ├── VM_TEST_STEPS.md                # Step-by-step build / run / benchmark guide (Ubuntu 24.04 + Jazzy)
│   ├── raw_trials.csv                  # Raw measured legs written by viva_demo (created by the benchmark)
│   ├── test_results.csv                # Per-trial results + reference length + efficiency (collect_metrics.py)
│   ├── test_summary.csv                # Per method/goal summary (collect_metrics.py)
│   └── report/final_report.md          # Report (5–7 pages)
├── scripts/
│   ├── run_tests.sh                    # File checks, syntax checks, benchmark post-processing
│   ├── collect_metrics.py              # Post-processes MEASURED trials (no simulation, no estimates)
│   ├── align_map_origin.py             # Shifts a saved SLAM map so map frame == Gazebo world frame
│   └── pad_map.py                      # Adds an unknown border so NavFn can plan long routes
└── src/
    ├── maze_world/                     # Gazebo world + launch (spawns the Burger, ros_gz bridge)
    │   ├── launch/maze_world.launch.py
    │   ├── models/maze/
    │   └── worlds/maze.world
    ├── maze_navigation/                # SLAM Toolbox + Nav2
    │   ├── config/nav2_params.yaml
    │   ├── config/slam_params.yaml     # SLAM Toolbox settings used by slam.launch.py
    │   ├── launch/navigation.launch.py # args: map, params_file, use_astar, use_rviz
    │   ├── launch/localization.launch.py # map server + AMCL only (for the DQN controller)
    │   ├── launch/slam.launch.py       # args: use_rviz, slam_params_file (also starts imu_odom.py)
    │   ├── maps/maze_map.{yaml,pgm}    # Installed map used by navigation.launch.py
    │   ├── scripts/imu_odom.py         # Wheel speed + IMU gyro odometry for SLAM
    │   └── rviz/nav2.rviz
    ├── goal_interface/
    │   └── goal_interface/
    │       ├── goal_manager.py         # /goal_pose and /clicked_point -> Nav2 (with bounds check)
    │       └── viva_demo.py            # 3-goal demo + measurement (Nav2 or DQN controller)
    └── maze_dqn/
        ├── models/                     # dqn_weights.npz (final-goal), dqn_subgoal_weights.npz (original),
        │                               # dqn_subgoal_safe_weights.npz/.json (retrained, model:=safe),
        │                               # *_training_log.csv, *_eval_log.csv, dqn_final_evaluation.csv
        └── maze_dqn/
            ├── maze_env.py             # 2D kinematic maze environment (same 18 walls)
            ├── dqn_model.py            # NumPy Double DQN (Adam, Huber loss)
            ├── train_dqn.py            # Curriculum training
            └── dqn_navigation_node.py  # ROS 2 node running the trained policy
```

---

## ⚙️ Prerequisites

* Ubuntu 24.04 LTS, ROS 2 Jazzy, Gazebo Harmonic

```bash
sudo apt update
sudo apt install -y   ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox   ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-msgs ros-jazzy-turtlebot3-gazebo ros-jazzy-ros-gz   ros-jazzy-teleop-twist-keyboard python3-colcon-common-extensions python3-numpy git
```

---

## 🚀 Build

```bash
cd ~/ie4060_assignment02            # wherever you cloned the repository
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
colcon build --symlink-install
source install/setup.bash
```

---

## 🖥️ How to run everything (WSL, step by step)

Every command below is **complete**: it goes to the project folder and loads ROS itself, so you
can paste it into any new tab. Each demo uses several tabs; open a new one in Windows Terminal
with **⌄** (next to **+**) → **Ubuntu-24.04**. Run the tabs in order and wait for the message
shown before opening the next one.

**Quick VIVA order:** Step 1 → Step 2 → Demo A (click 3 goals) → Step 2 → Demo B (retrained DQN,
click 3 goals) → Demo C only if asked.

### Step 1: copy the latest code into Ubuntu and build (each time the code changes)
Copy the code from the Windows folder (a local copy: no internet or GitHub needed):
```bash
rsync -a --delete --exclude build --exclude install --exclude log --exclude .git "/mnt/c/Users/A S U S/Downloads/ie4060_assignment02/" ~/ie4060_assignment02/
```

Build (1–2 minutes; wait for `Summary: 4 packages finished`):
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
```

### Step 2: clean up before every demo
Stops anything left over from an earlier run (it is fine if it prints nothing):
```bash
pkill -9 -f "gz sim"; pkill -f "ros2 launch"; pkill -f imu_odom; pkill -f dqn_navigation; pkill -f viva_demo
```

---

### Demo A: Nav2 with Dijkstra
**Tab 1: simulator.** Wait until Gazebo shows the maze and the robot.
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch maze_world maze_world.launch.py
```

**Tab 2: Nav2 + RViz.** Wait for `Managed nodes are active`.
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch maze_navigation navigation.launch.py
```

**Click goals:** in RViz choose **Nav2 Goal**, click in the middle of a corridor, drag to set the
direction, release. Let the robot arrive before clicking again (a new goal replaces the current
one). Good goals: G1 (−0.25, −0.75), one cell away, about 15 s; G2 (0.75, 0.75), the long route,
about 90 s; G3 (1.25, 1.25), about 15 s.

**Or Tab 3: automatic 3-goal demo** (sends G1, G2, G3 and prints time, path and collisions):
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run goal_interface viva_demo --ros-args -p method:="Nav2 (Dijkstra)" -p trials:=1
```

### Demo A2: Nav2 with A*
Run Step 2, then Tab 1 exactly as in Demo A.

**Tab 2: Nav2 (A\*) + RViz.** Wait for `Managed nodes are active`.
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch maze_navigation navigation.launch.py use_astar:=true
```

Click goals with **Nav2 Goal** as in Demo A, **or Tab 3: automatic 3-goal demo:**
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run goal_interface viva_demo --ros-args -p method:="Nav2 (A*)" -p trials:=1
```

### Demo B: DQN (machine learning, retrained model)
Never run the DQN together with Nav2 (both react to clicked goals): close Demo A and run Step 2 first.

**Tab 1: simulator.** Wait until Gazebo shows the maze.
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch maze_world maze_world.launch.py
```

**Tab 2: map + localisation (AMCL) + RViz.** Wait for `Managed nodes are active`.
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch maze_navigation localization.launch.py odom:=imu
```

**Tab 3: DQN driver, retrained model** (the one in the report: 9/9 goals, 0 collisions). Wait for
`DQN Navigation Node ready`.
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run maze_dqn dqn_navigation_node --ros-args -p use_sim_time:=true -p subgoal_mode:=true -p base_frame:=base_footprint_imu -p model:=safe
```

**Click goals:** in RViz choose **2D Goal Pose** (not Nav2 Goal), click and drag.

**Or Tab 4: automatic 3-goal demo:**
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run goal_interface viva_demo --ros-args -p controller:=dqn -p method:="DQN (subgoal, retrained)" -p base_frame:=base_footprint_imu -p trials:=1 -p goal_timeout:=180.0
```

**Other DQN variants (for comparison, all in the report).** Use one of these as Tab 3 instead:

Original subgoal DQN (reaches the goals but touches walls):
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run maze_dqn dqn_navigation_node --ros-args -p use_sim_time:=true -p subgoal_mode:=true -p base_frame:=base_footprint_imu
```

Original subgoal DQN + safety shield (action masking; it did not help):
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run maze_dqn dqn_navigation_node --ros-args -p use_sim_time:=true -p subgoal_mode:=true -p base_frame:=base_footprint_imu -p safety_shield:=true
```

Pure final-goal DQN (no route planner; fails when the goal is behind walls):
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run maze_dqn dqn_navigation_node --ros-args -p use_sim_time:=true -p subgoal_mode:=false -p base_frame:=base_footprint_imu
```

### Demo C: SLAM mapping
Run Step 2 first.

**Tab 1: simulator.**
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch maze_world maze_world.launch.py
```

**Tab 2: SLAM Toolbox + gyro odometry + RViz** (the map grows in RViz as the robot drives):
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch maze_navigation slam.launch.py
```

**Tab 3, automatic driving** through all 36 cells (about 15 minutes, ends with `EXPLORATION DONE`):
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 scripts/explore_maze.py
```

**Or Tab 3, keyboard driving.** Keep this tab selected while pressing keys: `i` forward,
`j` / `l` turn left / right, `,` back, `k` stop. Drive slowly: fast turns make the wheels slip.
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p speed:=0.15 -p turn:=0.5
```

**Optional: save the new map as the navigation map.** This **replaces** the tested map in the
Ubuntu copy; for the VIVA, just show mapping live and skip it. In Tab 4, while SLAM is still running:
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && ros2 run nav2_map_server map_saver_cli -f src/maze_navigation/maps/maze_map --ros-args -p save_map_timeout:=30.0
```

Then align it with the Gazebo world frame, add the planning border and rebuild (run once per saved map):
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && python3 scripts/align_map_origin.py src/maze_navigation/maps/maze_map.yaml && python3 scripts/pad_map.py src/maze_navigation/maps/maze_map.yaml && colcon build --symlink-install --packages-select maze_navigation
```

---

### Optional: full benchmark (3 trials × 3 tasks, results saved to CSV)
`fixed_starts:=true` places the robot at the same start before every leg (S for G1, G1 for G2,
G2 for G3), so every method and trial runs exactly the same three tasks. Start the demo setup
(Tabs 1–2 of Demo A / A2, or Tabs 1–3 of Demo B with the matching Tab 3), then run the matching
command in a new tab. Each takes about 8–15 minutes and appends to `docs/raw_trials.csv`.

Nav2 Dijkstra (Demo A running):
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run goal_interface viva_demo --ros-args -p method:="Nav2 (Dijkstra)" -p trials:=3 -p fixed_starts:=true -p results_csv:=$HOME/ie4060_assignment02/docs/raw_trials.csv
```

Nav2 A* (Demo A2 running):
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run goal_interface viva_demo --ros-args -p method:="Nav2 (A*)" -p trials:=3 -p fixed_starts:=true -p results_csv:=$HOME/ie4060_assignment02/docs/raw_trials.csv
```

Retrained DQN (Demo B running with `model:=safe`):
```bash
cd ~/ie4060_assignment02 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run goal_interface viva_demo --ros-args -p controller:=dqn -p method:="DQN (subgoal, retrained)" -p base_frame:=base_footprint_imu -p goal_timeout:=180.0 -p trials:=3 -p fixed_starts:=true -p results_csv:=$HOME/ie4060_assignment02/docs/raw_trials.csv
```

For the other DQN variants, start Demo B with their Tab 3 and use the same command with
`method:="DQN (subgoal)"`, `method:="DQN (subgoal) + shield"` or `method:="DQN (final goal)"`.

Make the result tables (`docs/test_results.csv`, `docs/test_summary.csv`) and run the offline checks:
```bash
cd ~/ie4060_assignment02 && python3 scripts/collect_metrics.py docs/raw_trials.csv && bash scripts/run_tests.sh
```

The results are written in the **Ubuntu copy**. Copy them back to Windows before running Step 1
again (Step 1's `--delete` would replace them with the Windows files):
```bash
cp ~/ie4060_assignment02/docs/raw_trials.csv ~/ie4060_assignment02/docs/test_results.csv ~/ie4060_assignment02/docs/test_summary.csv "/mnt/c/Users/A S U S/Downloads/ie4060_assignment02/docs/"
```

### Optional: retrain the DQN (2D simulator, no Gazebo needed)
The shipped models are already trained. This trains a new model with the retrained model's settings (about an
hour); `--out-dir` keeps it from overwriting the shipped weights:
```bash
cd ~/ie4060_assignment02/src/maze_dqn && python3 -m maze_dqn.train_dqn --subgoal --episodes 4000 --seed 1 --collision-dist 0.15 --turn-rate 0.5 --wall-penalty-dist 0.20 --tag safe --out-dir ~/dqn_retrain
```

### Stop everything
Press **Ctrl+C** in each tab, then:
```bash
pkill -9 -f "gz sim"; pkill -f "ros2 launch"; pkill -f imu_odom; pkill -f dqn_navigation; pkill -f viva_demo
```

### Tips and troubleshooting
- **Gazebo slow?** Add ` gui:=false` to the end of the Tab 1 command; RViz still shows the robot.
- **Robot does not move after a click?** Check the RViz tool: **Nav2 Goal** for Nav2 (Demo A),
  **2D Goal Pose** for the DQN (Demo B). Also wait for `Managed nodes are active` before clicking.
- **Robot shown in the wrong place, or two robots?** A previous run is still alive: Ctrl+C in all
  tabs, run Step 2, start again.
- **`package not found` / `command not found`?** Run the build in Step 1.
- **Far goals take time:** across the whole maze is about 2.5 minutes with Nav2.

### What the parts do
- `maze_world.launch.py` starts Gazebo with `maze.world`, spawns the Burger at (−0.75, −1.25) and
  bridges `/scan`, `/odom`, `/imu`, `/tf`, `/clock` and `/cmd_vel` (`TwistStamped`) to ROS 2.
- `slam.launch.py` runs SLAM Toolbox on `imu_odom.py`'s odometry (wheel speed + IMU gyro heading,
  because the wheels slip on turns); scan matching is off in `slam_params.yaml` (reasons in the comments there).
- `explore_maze.py` visits all 36 cells depth-first with slow turns; it steers using the Gazebo
  pose between moves, while SLAM itself only sees the robot's own sensors.
- `align_map_origin.py` makes map coordinates equal Gazebo world coordinates; `pad_map.py` adds a
  2.5 m unknown border, because NavFn cannot return a path longer than about twice the map width.
- `navigation.launch.py` runs Nav2 (map server, AMCL, NavFn Dijkstra or A*, Regulated Pure Pursuit).
- `localization.launch.py` runs only the map server and AMCL, so the DQN gets a pose without Nav2.
- `dqn_navigation_node` turns `/scan` and the AMCL pose into the 39-value state and drives the robot;
  `model:=safe` loads `dqn_subgoal_safe_weights.npz`.
- `viva_demo` sends the three goals and measures time, path length, collisions and clearance;
  `collect_metrics.py` turns the raw trials into the result tables.

---

## 📊 Results

See `docs/test_summary.csv` and Section 6 of the report. All values are produced by measured
runs (`viva_demo`) and post-processed by `scripts/collect_metrics.py`. No result is estimated.

---

## 🎓 Academic Integrity & Submission Details

* **Course:** IE4060: Robotics and Intelligent Systems (Year 04)
* **Due Date:** 18th October 2026, 11:59 PM
* **VIVA Voce Date:** 18th October 2026 (Lecture Time)
* **Author:** 
