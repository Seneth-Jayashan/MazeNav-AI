#!/usr/bin/env python3
"""
viva_demo.py - the 3-goal demonstration and benchmark with built-in measurement.

It sends the robot to G1 (-0.25, -0.75), G2 (0.75, 0.75) and G3 (1.25, 1.25) in turn, a chosen
number of times, measures every leg and prints a summary table; with results_csv it also
appends one row per leg to a CSV, which scripts/collect_metrics.py turns into the report tables.

Measured per leg: success, travel time (simulation clock), path length (sum of /odom steps),
collision events (LiDAR below 0.12 m) and minimum clearance.

Main parameters
  controller      nav2 (default): goals go to Nav2's NavigateToPose action
                  dqn: goals are published on /goal_pose for dqn_navigation_node
  trials          how many times to run G1 -> G2 -> G3
  fixed_starts    true: place the robot at the same start before every leg (fair comparison)
  method          label written in the CSV (e.g. "Nav2 (Dijkstra)")
  results_csv     path of the CSV to append to
  base_frame      base_footprint, or base_footprint_imu for DQN runs with odom:=imu

Run (with Gazebo + navigation.launch.py running):
  ros2 run goal_interface viva_demo --ros-args -p method:="Nav2 (Dijkstra)" -p trials:=3       -p fixed_starts:=true -p results_csv:=$HOME/ie4060_assignment02/docs/raw_trials.csv
"""
import csv
import math
import os
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener, TransformException

# Columns written for every leg (collect_metrics.py adds the reference length and efficiency)
CSV_FIELDS = ['Method', 'Trial', 'Goal', 'Goal_x', 'Goal_y', 'Start_x', 'Start_y', 'Success', 'Status',
              'Time_s', 'Path_Length_m', 'Collisions', 'Min_Clearance_m']


class VivaDemo(Node):
    """Sends 3 goals in sequence and measures each leg in simulation time.

    controller:=nav2 sends NavigateToPose action goals; controller:=dqn publishes the goal on
    /goal_pose for maze_dqn's dqn_navigation_node and judges success by the TF distance to
    the goal (failure on timeout).

    Measured per leg: travel time (sim clock), path length (sum of /odom displacements),
    collision events (min valid /scan range dropping below collision_threshold) and success.

    fixed_starts:=true makes every trial identical: before each leg the robot is placed at a
    fixed start pose in Gazebo (S for G1, G1 for G2, G2 for G3), AMCL is told that pose and
    Nav2's costmaps are cleared. This is part of the test harness only; the navigation stack
    itself never uses the simulator's pose.
    """
    def __init__(self):
        super().__init__('viva_demo')
        if not self.get_parameter('use_sim_time').value:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.get_logger().info('=== Starting IE4060 3-Goal VIVA Demonstration ===')

        self.declare_parameter('controller', 'nav2')    # 'nav2' or 'dqn'
        self.declare_parameter('goal_tolerance', 0.15)  # [m] success radius in dqn mode
        self.declare_parameter('base_frame', 'base_footprint')  # base_footprint_imu with localization odom:=imu
        # Fallback when there is no map frame (e.g. DQN runs without AMCL): odom origin = spawn pose
        self.declare_parameter('spawn_x', -0.75)
        self.declare_parameter('spawn_y', -1.25)
        self.declare_parameter('trials', 1)
        self.declare_parameter('method', 'Nav2')
        self.declare_parameter('results_csv', '')
        self.declare_parameter('collision_threshold', 0.12)
        self.declare_parameter('goal_timeout', 240.0)  # sim seconds
        self.declare_parameter('pause', 2.0)           # sim seconds between goals
        self.declare_parameter('fixed_starts', False)  # place the robot at the same start before every leg
        self.declare_parameter('settle_time', 3.0)     # sim seconds after placing the robot
        self.declare_parameter('gz_world', 'maze_world')
        self.declare_parameter('gz_model', 'burger')

        self.controller = self.get_parameter('controller').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.trials = self.get_parameter('trials').value
        self.method = self.get_parameter('method').value
        self.results_csv = self.get_parameter('results_csv').value
        self.collision_threshold = self.get_parameter('collision_threshold').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.pause = self.get_parameter('pause').value
        self.fixed_starts = self.get_parameter('fixed_starts').value
        self.settle_time = self.get_parameter('settle_time').value

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 20)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.event_pub = self.create_publisher(String, '/viva_demo/event', 10)   # for external loggers
        self.clear_clients = [self.create_client(ClearEntireCostmap, srv) for srv in (
            '/local_costmap/clear_entirely_local_costmap', '/global_costmap/clear_entirely_global_costmap')]

        # 3 distinct goals across the maze, in the map frame (map frame == Gazebo world frame,
        # see maze_navigation/maps/maze_map.yaml). Start is around (-0.75, -1.25).
        self.goals = [
            {'name': 'Goal 1 (Corridor Junction)', 'x': -0.25, 'y': -0.75, 'yaw': 0.0},
            {'name': 'Goal 2 (Central Passage)',   'x':  0.75, 'y':  0.75, 'yaw': 1.57},
            {'name': 'Goal 3 (Far Maze Destination)','x':  1.25, 'y':  1.25, 'yaw': 0.0},
        ]
        # Fixed start of each leg (used with fixed_starts): the spawn, then the previous goal
        spawn = {'x': self.get_parameter('spawn_x').value, 'y': self.get_parameter('spawn_y').value, 'yaw': 0.0}
        self.starts = [spawn] + [dict(g, yaw=0.0) for g in self.goals[:-1]]
        self.queue = [(t + 1, g, self.starts[i]) for t in range(self.trials) for i, g in enumerate(self.goals)]
        self.placed = False
        self.results = []
        self.done = False

        # Per-leg measurement state
        self.active = False
        self.goal_handle = None
        self.next_send_time = None
        self.leg = None
        self.last_odom_xy = None

        self.timer = self.create_timer(0.1, self.tick)

    # ------------------------------------------------------------ sensors
    def odom_callback(self, msg: Odometry):
        """Add the distance moved since the last odometry message to the current leg."""
        p = msg.pose.pose.position
        if self.active and self.last_odom_xy is not None:
            self.leg['path'] += math.hypot(p.x - self.last_odom_xy[0], p.y - self.last_odom_xy[1])
        self.last_odom_xy = (p.x, p.y)

    def scan_callback(self, msg: LaserScan):
        """Track the closest LiDAR reading and count collision events (each time the closest
        reading drops below collision_threshold counts once, until it rises again)."""
        if not self.active:
            return
        valid = [r for r in msg.ranges if math.isfinite(r) and r > 0.0]
        if any(r == float('-inf') for r in msg.ranges):
            valid.append(0.0)
        if not valid:
            return
        m = min(valid)
        self.leg['min_clear'] = min(self.leg['min_clear'], m)
        colliding = m < self.collision_threshold
        if colliding and not self.leg['in_collision']:
            self.leg['collisions'] += 1
            self.get_logger().warn(f'Collision event: min scan {m:.3f} m')
        self.leg['in_collision'] = colliding

    def robot_xy(self):
        """Robot position in the map frame (from TF), or odometry + spawn offset as a fallback."""
        try:
            t = self.tf_buffer.lookup_transform('map', self.get_parameter('base_frame').value, Time())
            return t.transform.translation.x, t.transform.translation.y
        except TransformException:
            if self.last_odom_xy is None:
                return float('nan'), float('nan')
            self.get_logger().warn(f"No map->{self.get_parameter('base_frame').value} TF; using /odom + spawn offset.", once=True)
            return (self.get_parameter('spawn_x').value + self.last_odom_xy[0],
                    self.get_parameter('spawn_y').value + self.last_odom_xy[1])

    def sim_now(self):
        """Current simulation time in seconds (all times are measured on the sim clock)."""
        return self.get_clock().now().nanoseconds * 1e-9

    # -------------------------------------------------------- state machine
    def tick(self):
        """10 Hz state machine: watch the active leg (success / timeout), otherwise wait,
        place the robot (fixed_starts) and send the next goal; print the summary at the end."""
        if self.done or self.sim_now() <= 0.0:
            return  # wait for /clock
        if self.active and self.controller == 'dqn':
            # /goal_pose is a plain topic: send the goal once more after 1 s in case the first
            # message went out before the subscription was fully matched (the DQN node simply
            # sets the same goal again)
            if not self.leg['resent'] and self.sim_now() - self.leg['t0'] > 1.0:
                self.goal_pub.publish(self.leg['goal_pose'])
                self.leg['resent'] = True
            x, y = self.robot_xy()
            if math.hypot(self.leg['target']['x'] - x, self.leg['target']['y'] - y) < self.goal_tolerance:
                self.finish_leg(GoalStatus.STATUS_SUCCEEDED)
            elif self.sim_now() - self.leg['t0'] > self.goal_timeout:
                self.get_logger().warn('DQN goal timed out.')
                self.finish_leg(GoalStatus.STATUS_ABORTED)
            return
        if self.active:
            if self.sim_now() - self.leg['t0'] > self.goal_timeout and self.goal_handle is not None:
                self.get_logger().warn('Goal timed out, cancelling.')
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            return
        if self.next_send_time is not None and self.sim_now() < self.next_send_time:
            return
        if not self.queue:
            self.display_summary()
            self.done = True
            return
        if self.controller != 'dqn' and not self._action_client.server_is_ready():
            self.get_logger().info('Waiting for Nav2 NavigateToPose action server...', throttle_duration_sec=5.0)
            return
        if self.controller == 'dqn' and self.goal_pub.get_subscription_count() == 0:
            self.get_logger().info('Waiting for dqn_navigation_node to subscribe to /goal_pose...',
                                   throttle_duration_sec=5.0)
            return
        if any(math.isnan(v) for v in self.robot_xy()):
            self.get_logger().info('Waiting for a robot pose (TF or /odom)...', throttle_duration_sec=5.0)
            return
        if self.fixed_starts and not self.placed:
            self.place_robot(self.queue[0][2])
            self.placed = True
            self.next_send_time = self.sim_now() + self.settle_time
            return
        self.placed = False
        self.send_goal(*self.queue.pop(0))

    def place_robot(self, start):
        """Put the robot at the leg's fixed start pose (test harness, simulation only)."""
        x, y, yaw = start['x'], start['y'], start['yaw']
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        self.event_pub.publish(String(data='placing'))
        if self.controller == 'dqn':
            # give the DQN node the start as its goal, so it stops there instead of driving on
            stop = PoseStamped()
            stop.header.frame_id = 'map'
            stop.pose.position.x, stop.pose.position.y = x, y
            stop.pose.orientation.z, stop.pose.orientation.w = qz, qw
            self.goal_pub.publish(stop)
        req = (f'name: "{self.get_parameter("gz_model").value}", position: {{x: {x}, y: {y}, z: 0.01}}, '
               f'orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}')
        res = subprocess.run(['gz', 'service', '-s', f'/world/{self.get_parameter("gz_world").value}/set_pose',
                              '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
                              '--req', req], capture_output=True, text=True)
        if 'true' not in res.stdout:
            self.get_logger().error(f'Could not place the robot: {res.stdout.strip()} {res.stderr.strip()}')
        ip = PoseWithCovarianceStamped()
        ip.header.frame_id = 'map'
        ip.header.stamp = self.get_clock().now().to_msg()
        ip.pose.pose.position.x, ip.pose.pose.position.y = x, y
        ip.pose.pose.orientation.z, ip.pose.pose.orientation.w = qz, qw
        ip.pose.covariance[0] = ip.pose.covariance[7] = 0.01
        ip.pose.covariance[35] = 0.01
        self.initialpose_pub.publish(ip)
        for c in self.clear_clients:
            if c.service_is_ready():
                c.call_async(ClearEntireCostmap.Request())
        self.get_logger().info(f'Robot placed at the fixed start ({x:.2f}, {y:.2f}).')

    def send_goal(self, trial, target, start=None):
        """Start a leg: reset the measurements and send the goal to Nav2 or the DQN."""
        self.get_logger().info(f"--- Trial {trial}: navigating to {target['name']}: ({target['x']:.2f}, {target['y']:.2f}) ---")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target['x']
        goal_msg.pose.pose.position.y = target['y']
        goal_msg.pose.pose.position.z = 0.0

        yaw = target['yaw']
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        sx, sy = (start['x'], start['y']) if self.fixed_starts and start else self.robot_xy()
        self.leg = {'trial': trial, 'target': target, 't0': self.sim_now(), 'path': 0.0, 'collisions': 0,
                    'in_collision': False, 'min_clear': float('inf'), 'start': (sx, sy),
                    'goal_pose': goal_msg.pose, 'resent': False}
        self.active = True
        if self.controller == 'dqn':
            self.goal_pub.publish(goal_msg.pose)
            return
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """Nav2 progress (logged at most every 2 s)."""
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'Remaining: {dist:.2f} m', throttle_duration_sec=2.0)

    def goal_response_callback(self, future):
        """Nav2 accepted or rejected the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal {self.leg['target']['name']} rejected by Nav2!")
            self.finish_leg(GoalStatus.STATUS_ABORTED)
            return
        self.goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Nav2 finished the goal (succeeded, aborted or cancelled)."""
        self.finish_leg(future.result().status)

    def finish_leg(self, status):
        """End the current leg: record its result row and schedule the next goal."""
        self.active = False
        self.goal_handle = None
        leg = self.leg
        elapsed = self.sim_now() - leg['t0']
        success = (status == GoalStatus.STATUS_SUCCEEDED)
        row = {
            'Method': self.method, 'Trial': leg['trial'], 'Goal': leg['target']['name'],
            'Goal_x': leg['target']['x'], 'Goal_y': leg['target']['y'],
            'Start_x': f"{leg['start'][0]:.3f}", 'Start_y': f"{leg['start'][1]:.3f}",
            'Success': int(success), 'Status': status, 'Time_s': f'{elapsed:.2f}',
            'Path_Length_m': f"{leg['path']:.3f}", 'Collisions': leg['collisions'],
            'Min_Clearance_m': f"{leg['min_clear']:.3f}",
        }
        self.results.append(row)
        self.write_csv(row)
        if success:
            self.get_logger().info(f"Goal reached in {elapsed:.2f} s (sim), path {leg['path']:.2f} m, "
                                   f"collisions {leg['collisions']}")
        else:
            self.get_logger().warn(f"Goal failed (status {status}) after {elapsed:.2f} s (sim).")
        self.next_send_time = self.sim_now() + self.pause

    def write_csv(self, row):
        """Append one result row to results_csv (header written when the file is new)."""
        if not self.results_csv:
            return
        path = os.path.expanduser(self.results_csv)
        new_file = not os.path.exists(path)
        with open(path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            if new_file:
                writer.writeheader()
            writer.writerow(row)

    def display_summary(self):
        """Print the table of all legs at the end of the run."""
        print("\n" + "="*88)
        print("       IE4060 VIVA DEMONSTRATION EXPERIMENTAL SUMMARY")
        print("="*88)
        print(f"{'Trial':<6} | {'Goal Target':<30} | {'Status':<8} | {'Time (s)':<9} | {'Path (m)':<9} | {'Collisions':<10}")
        print("-"*88)
        for r in self.results:
            st = "SUCCESS" if r['Success'] else "FAILED"
            print(f"{r['Trial']:<6} | {r['Goal']:<30} | {st:<8} | {r['Time_s']:<9} | {r['Path_Length_m']:<9} | {r['Collisions']:<10}")
        print("="*88 + "\n")


def main(args=None):
    rclpy.init(args=args)
    demo = VivaDemo()
    try:
        while rclpy.ok() and not demo.done:
            rclpy.spin_once(demo, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        demo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
