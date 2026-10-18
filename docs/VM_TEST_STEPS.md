# VM Test Steps (Ubuntu 24.04 + ROS 2 Jazzy, VirtualBox)

Run these steps in order. Each step lists what you should see and what to send back if it
fails. **Never type a result into a CSV or the report by hand.** All numbers come from
`docs/raw_trials.csv` (written by `viva_demo`) and the DQN logs.

Conventions:
- `T1`, `T2`, ... are separate terminals. In **every** new terminal run:
  ```bash
  cd ~/ie4060_assignment02
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash          # skip this line before the first build
  export TURTLEBOT3_MODEL=burger
  ```
- To stop: `Ctrl+C` in each terminal. If Gazebo keeps running afterwards: `pkill -f "gz sim"`.
- **VirtualBox graphics:** if Gazebo crashes, shows a black window, or `/scan` never publishes,
  run `export LIBGL_ALWAYS_SOFTWARE=1` in the Gazebo terminal and relaunch. A slow VM is fine:
  all timings are measured in **simulation time**.

---

## 0. Get the code into the VM

The repository has no GitHub remote yet. On Windows, a bundle file was created:
`ie4060_assignment02.bundle` (see the chat). Copy it into the VM (shared folder or drag-and-drop), then:
```bash
cd ~
git clone ie4060_assignment02.bundle ie4060_assignment02
cd ie4060_assignment02 && git log --oneline | head -3
```
**Expect:** the latest commit is at the top.

---

## 1. Install dependencies
```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-msgs ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-ros-gz ros-jazzy-teleop-twist-keyboard python3-colcon-common-extensions \
  python3-numpy python3-pil pandoc git
```
**Expect:** finishes without errors.
**If it fails, send:** the last 30 lines of output.

---

## 2. Clean build (including a fresh clone)
```bash
cd ~/ie4060_assignment02
rm -rf build install log
colcon build --symlink-install
```
**Expect:** `Summary: 4 packages finished` and no `Failed` lines.

Fresh-clone check (proves the build does not depend on untracked files):
```bash
rm -rf /tmp/clonetest && git clone ~/ie4060_assignment02 /tmp/clonetest
cd /tmp/clonetest && colcon build && cd ~/ie4060_assignment02
```
**Expect:** again `Summary: 4 packages finished`.
**If it fails, send:** the full `colcon build` output (or `log/latest_build/<package>/stderr.log`).

---

## 3. Phase 1: robot in Gazebo
`T1`:
```bash
ros2 launch maze_world maze_world.launch.py
```
**Expect:** the Gazebo window shows the maze with the Burger in the bottom row, second cell
from the left. No `[ERROR]` lines about `create` or `parameter_bridge`.

`T2` (run these one by one; stop each `hz` with Ctrl+C after about 5 s):
```bash
ros2 topic hz /scan           # Expect: average rate ~5 Hz
ros2 topic hz /odom           # Expect: a steady rate (tens of Hz)
ros2 topic hz /clock          # Expect: a steady rate
ros2 topic info /cmd_vel -v   # Expect: Type: geometry_msgs/msg/TwistStamped, Subscription count: 1 (ros_gz_bridge)
ros2 topic echo /scan --once --field angle_min    # Expect: 0.0
ros2 topic echo /scan --once --field range_min    # Expect: 0.12
ros2 run tf2_ros tf2_echo odom base_footprint     # Expect: a transform printed every second (Ctrl+C)
```
Movement test (rotate in place for 2 s, then read the orientation):
```bash
ros2 topic pub -r 10 -t 20 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {angular: {z: 0.5}}}"
ros2 topic echo /odom --once --field pose.pose.orientation
```
**Expect:** the robot visibly turns in Gazebo, and `orientation.z` is clearly non-zero.

**If anything fails, send:** the `T1` launch output (first 80 lines plus any errors), the
output of `ros2 topic list`, and the `ros2 topic info /cmd_vel -v` output.
**Always send back (even on success):** the `ros2 topic info /cmd_vel -v` output. The report needs it.

Stop everything (Ctrl+C in all terminals).

---

## 4. Phase 3a: SLAM and saving the real map
`T1`: `ros2 launch maze_world maze_world.launch.py`
`T2`: `ros2 launch maze_navigation slam.launch.py`
`T3`:
```bash
python3 scripts/explore_maze.py
```
It drives through every cell (about 15 minutes) and prints `EXPLORATION DONE`.
(Manual alternative: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p speed:=0.15 -p turn:=0.5`,
driving **slowly** and turning gently through every corridor and dead end.) Then, in `T4`:
```bash
ros2 run nav2_map_server map_saver_cli -f src/maze_navigation/maps/maze_map --ros-args -p save_map_timeout:=30.0
python3 scripts/align_map_origin.py src/maze_navigation/maps/maze_map.yaml
python3 scripts/pad_map.py src/maze_navigation/maps/maze_map.yaml
cat src/maze_navigation/maps/maze_map.yaml
mkdir -p docs/report/figures
python3 -c "from PIL import Image; Image.open('src/maze_navigation/maps/maze_map.pgm').save('docs/report/figures/slam_map.png')"
python3 -c "from PIL import Image; im=Image.open('src/maze_navigation/maps/maze_map.pgm'); print(im.size)"
```
**Expect:** `map_saver_cli` prints `Map saved successfully`. `align_map_origin.py` prints
`origin [a, b] -> [a-0.75, b-1.25]`. Run it **only once** per saved map. `pad_map.py` prints
the image growing by 100 px on each axis (a 2.5 m unknown border); also run it only once. The PNG shows the
maze walls with sensor noise along the edges.

Stop everything, then rebuild so the new map is installed:
```bash
colcon build --symlink-install --packages-select maze_navigation
```

**If it fails, send:** the `map_saver_cli` output, the contents of `maze_map.yaml`, and the PNG.
**Always send back:** `docs/report/figures/slam_map.png`, the printed map size, and `maze_map.yaml`.

---

## 5. Phase 3b: Nav2 smoke test (clicked goals)
`T1`: `ros2 launch maze_world maze_world.launch.py`
`T2`: `ros2 launch maze_navigation navigation.launch.py`

**Expect:** within about 30 s, `T2` prints `Managed nodes are active` twice (localization and
navigation). In RViz, the robot model sits on the map at the bottom row with the laser scan
matching the walls. The pose is already set (AMCL `set_initial_pose`), so you do **not** need
"2D Pose Estimate".

Check the frame alignment:
```bash
ros2 run tf2_ros tf2_echo map base_footprint      # Expect: translation close to (-0.75, -1.25)
```
Click **Nav2 Goal** in RViz on three different corridors. **Expect:** a path appears and
the robot drives to each goal.

**If it fails, send:** `T2` output from start until the first error (especially lines with
`lifecycle_manager`, `Failed to`, `collision_monitor`, `docking_server`), plus a screenshot of
RViz. If the robot gets stuck in a corridor, say where, and note any `No valid trajectories`
or `Failed to make progress` messages.

---

## 6. Phase 3c and 4: Nav2 benchmark (Dijkstra, then A*)
Delete old raw data once before the first benchmark run:
```bash
rm -f docs/raw_trials.csv
```

### 6a. Dijkstra (3 trials = 9 legs)
`T1`: `ros2 launch maze_world maze_world.launch.py gui:=false`
`T2`: `ros2 launch maze_navigation navigation.launch.py use_astar:=false`
`T3` (after `Managed nodes are active`):
```bash
ros2 param get /planner_server GridBased.use_astar          # Expect: Boolean value is: False
ros2 run goal_interface viva_demo --ros-args -p method:="Nav2 (Dijkstra)" -p trials:=3 \
  -p results_csv:=$HOME/ie4060_assignment02/docs/raw_trials.csv
```
**Expect:** for each leg, `Goal reached in X s (sim), path Y m, collisions Z`. It ends with a
summary table of 9 rows, and then the node exits by itself.

Stop everything (Ctrl+C in T1 and T2, then `pkill -f "gz sim"`), so the next run starts again
from the spawn pose.

### 6b. A* (3 trials = 9 legs)
Same as 6a, but with `use_astar:=true` in `T2`, `Boolean value is: True` from the param check,
and `-p method:="Nav2 (A*)"`.

**If a leg fails, send:** the `viva_demo` output and the `T2` lines around the failure time.
**Always send back:** `docs/raw_trials.csv`.

---

## 7. Phase 5: DQN in Gazebo
Nav2 must **not** be running here, because Nav2's `bt_navigator` also listens on `/goal_pose`.
Instead `localization.launch.py` starts only the map server and AMCL, so the DQN node and
`viva_demo` get the AMCL pose (`map -> base_footprint`). Raw wheel odometry drifts too much in
this world to be used for the pose.

`T1`: `ros2 launch maze_world maze_world.launch.py`
`T2`: `ros2 launch maze_navigation localization.launch.py odom:=imu` (wait for `Managed nodes are active`)
`T3`:
```bash
ros2 run maze_dqn dqn_navigation_node --ros-args -p use_sim_time:=true -p subgoal_mode:=true -p base_frame:=base_footprint_imu
```
To click a goal for the DQN, use **2D Goal Pose** in RViz.
**Expect:** `Loaded DQN weights from: .../share/maze_dqn/models/dqn_subgoal_weights.npz`.

`T4`:
```bash
ros2 run goal_interface viva_demo --ros-args -p controller:=dqn -p method:="DQN (subgoal)" -p base_frame:=base_footprint_imu \
  -p trials:=3 -p goal_timeout:=180.0 -p results_csv:=$HOME/ie4060_assignment02/docs/raw_trials.csv
```
**Expect:** 9 legs, each `SUCCESS` or `FAILED` (timeout). Failures are valid results; do not
re-run to "fix" them. If the robot is wedged against a wall after a failure, the remaining
legs will probably fail too. That is also a valid result.

Optional, final-goal variant (the pure DQN): restart T1 and T2, use `-p subgoal_mode:=false` in T3
and `-p method:="DQN (final goal)"` in T4.

**If it fails, send:** the `T2` and `T3` outputs, and the output of `ros2 topic echo /cmd_vel --once`
while the node is running.

---

## 8. Post-processing and checks
```bash
python3 scripts/collect_metrics.py docs/raw_trials.csv
./scripts/run_tests.sh
```
**Expect:** a table with one row per method and goal, and the files `docs/test_results.csv` and
`docs/test_summary.csv`. `run_tests.sh` ends with `ALL AUTOMATED SYSTEM CHECKS PASSED`.
**Send back:** `docs/test_results.csv`, `docs/test_summary.csv` and the printed table.

---

## 9. Report export (after the TODO-MEASURED values are filled in)
```bash
cd docs/report
grep -n "TODO-" final_report.md                      # Expect: no output
pandoc final_report.md -o final_report.docx
libreoffice --headless --convert-to pdf final_report.docx   # or export the PDF from Word
```

---

## What to send back (summary)
1. `ros2 topic info /cmd_vel -v` output (step 3)
2. `slam_map.png`, `maze_map.yaml`, map size (step 4)
3. Notes on any costmap / stuck problems (step 5)
4. `docs/raw_trials.csv` after steps 6 and 7
5. `docs/test_results.csv` and `docs/test_summary.csv` (step 8)
