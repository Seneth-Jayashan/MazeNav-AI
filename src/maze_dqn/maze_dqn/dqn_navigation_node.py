#!/usr/bin/env python3
"""
dqn_navigation_node.py - drives the robot in Gazebo with a trained DQN.

Every 0.1 s (10 Hz) it builds the same 39-value state as in training (maze_env.make_state)
from the real LiDAR and the robot pose, asks the network for the best action and publishes it
as a velocity command. It stops when the robot is within goal_tolerance of the goal.

Topics
  subscribes  /scan (LiDAR), /odom (fallback pose), /goal_pose (goal, e.g. RViz "2D Goal Pose")
  publishes   /cmd_vel (geometry_msgs/TwistStamped, as the TurtleBot3 Jazzy bridge expects)
  TF          map -> base_frame, from AMCL (maze_navigation localization.launch.py)

Main parameters
  subgoal_mode   true: steer to the next cell on the shortest grid route (subgoal DQN)
  model          ''  : original weights; 'safe': the retrained model (0.15 m margin, 0.5 rad/s)
  base_frame     base_footprint, or base_footprint_imu with localization.launch.py odom:=imu
  safety_shield  optional action masking (evaluated in the report; it did not help)

Run (with Gazebo and localization.launch.py odom:=imu running, Nav2 NOT running):
  ros2 run maze_dqn dqn_navigation_node --ros-args -p use_sim_time:=true \\
      -p subgoal_mode:=true -p model:=safe -p base_frame:=base_footprint_imu
"""
import json
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException

from maze_dqn.dqn_model import DQNAgent
from maze_dqn.maze_env import (MazeEnvironment, make_state, downsample_scan, make_actions, ACTIONS,
                               NUM_BEAMS, MAX_LIDAR_RANGE)


def yaw_from_quat(q):
    """Heading (rad) from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class DQNNavigationNode(Node):
    """ROS 2 node: LiDAR + pose + goal -> DQN action -> /cmd_vel."""
    def __init__(self):
        super().__init__('dqn_navigation_node')
        self.get_logger().info('=== Initializing Deep Q-Learning Navigation Node ===')

        self.declare_parameter('weights_path', '')
        self.declare_parameter('model', '')   # '' = the original weights, 'safe' = retrained with a footprint margin
        self.declare_parameter('subgoal_mode', False)
        self.declare_parameter('enable_stamped_cmd_vel', True)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('goal_tolerance', 0.15)
        # Fallback when map->base_footprint TF is unavailable: odom origin = spawn pose in the world/map frame
        self.declare_parameter('spawn_x', -0.75)
        self.declare_parameter('spawn_y', -1.25)
        self.declare_parameter('spawn_yaw', 0.0)
        # Safety shield (action masking): the DQN still ranks the actions by Q-value, but an
        # action that would drive into an obstacle closer than these LiDAR ranges is not allowed
        self.declare_parameter('safety_shield', False)
        self.declare_parameter('shield_front', 0.25)   # [m] min range within +-30 deg for any forward action
        self.declare_parameter('shield_side', 0.17)    # [m] min range 40-90 deg on a side for moving along it
        self.declare_parameter('shield_slow_range', 0.20)  # [m] below this range in any direction, slow down

        self.subgoal_mode = self.get_parameter('subgoal_mode').value
        self.shield = self.get_parameter('safety_shield').value
        self.shield_front = self.get_parameter('shield_front').value
        self.shield_side = self.get_parameter('shield_side').value
        self.shield_slow_range = self.get_parameter('shield_slow_range').value
        self.steps = 0
        self.overrides = 0
        self.escape_action = None   # rotation the shield committed to while forward motion is blocked
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.spawn = (self.get_parameter('spawn_x').value, self.get_parameter('spawn_y').value,
                      self.get_parameter('spawn_yaw').value)

        weights_path = self.get_parameter('weights_path').get_parameter_value().string_value
        if not weights_path:
            model = self.get_parameter('model').value
            name = ('dqn_subgoal' if self.subgoal_mode else 'dqn') + (f'_{model}' if model else '') + '_weights.npz'
            weights_path = os.path.join(get_package_share_directory('maze_dqn'), 'models', name)
        # Training settings saved next to the weights (turn rate of the actions)
        profile = {}
        if os.path.exists(weights_path.replace('.npz', '.json')):
            with open(weights_path.replace('.npz', '.json')) as f:
                profile = json.load(f)

        self.agent = DQNAgent(state_dim=NUM_BEAMS + 3, action_dim=len(ACTIONS))
        if os.path.exists(weights_path):
            self.agent.load(weights_path)
            self.get_logger().info(f'Loaded DQN weights from: {weights_path}')
        else:
            self.get_logger().warn(f'No pretrained weights at {weights_path}! Using initial policy.')

        # The 2D maze model provides the cell graph used for subgoals in subgoal_mode
        self.maze = MazeEnvironment(subgoal_mode=self.subgoal_mode)
        self.actions = make_actions(profile.get('turn_rate', 1.0))
        if profile:
            self.get_logger().info(f"Model settings: turn rate {profile.get('turn_rate')} rad/s, "
                                   f"collision margin {profile.get('collision_dist')} m")

        self.enable_stamped = self.get_parameter('enable_stamped_cmd_vel').get_parameter_value().bool_value
        if self.enable_stamped:
            self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        else:
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.latest_scan = None
        self.odom_pose = None
        self.goal_x = None
        self.goal_y = None
        self.last_action = 0
        self._warned_tf = False

        self.timer = self.create_timer(0.1, self.control_loop) # 10 Hz
        self.get_logger().info(f'DQN Navigation Node ready (subgoal_mode={self.subgoal_mode}, '
                               f'safety_shield={self.shield}). '
                               f'Awaiting goal on /goal_pose ({self.global_frame} frame)...')

    def odom_callback(self, msg: Odometry):
        """Keep the latest wheel-odometry pose (only used if there is no map frame)."""
        self.odom_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                          yaw_from_quat(msg.pose.pose.orientation))

    def scan_callback(self, msg: LaserScan):
        """Keep the latest LiDAR scan, reduced to 36 sector ranges in metres."""
        # Min-pool the 360 beams into 36 sectors of 10 deg, exactly as in training.
        # Returns below range_min are reported as -inf by Gazebo: treat them as very close.
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=MAX_LIDAR_RANGE, posinf=MAX_LIDAR_RANGE, neginf=msg.range_min)
        if len(ranges) == 0 or msg.angle_increment == 0.0:
            return
        self.latest_scan = downsample_scan(ranges, msg.angle_min, msg.angle_increment)

    def goal_callback(self, msg: PoseStamped):
        """New goal from /goal_pose (RViz '2D Goal Pose' or viva_demo): start driving to it."""
        if msg.header.frame_id and msg.header.frame_id != self.global_frame:
            self.get_logger().warn(f'Goal frame "{msg.header.frame_id}" != "{self.global_frame}"; using it as-is.')
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.maze.set_goal(self.goal_x, self.goal_y)
        self.steps = self.overrides = 0
        self.get_logger().info(f'Received new goal: ({self.goal_x:.2f}, {self.goal_y:.2f}). Starting DQN guidance...')

    def current_pose(self):
        """Robot pose in the map frame: TF map->base_footprint, else odom shifted by the spawn pose."""
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
            return (t.transform.translation.x, t.transform.translation.y, yaw_from_quat(t.transform.rotation))
        except TransformException:
            if self.odom_pose is None:
                return None
            if not self._warned_tf:
                self.get_logger().warn(f'No {self.global_frame}->{self.base_frame} TF; using /odom + spawn offset.')
                self._warned_tf = True
            ox, oy, oyaw = self.odom_pose
            sx, sy, syaw = self.spawn
            c, s = math.cos(syaw), math.sin(syaw)
            return (sx + c * ox - s * oy, sy + s * ox + c * oy, oyaw + syaw)

    def control_loop(self):
        """10 Hz: build the state, pick an action, publish it; stop at the goal."""
        if self.latest_scan is None or self.goal_x is None:
            return
        pose = self.current_pose()
        if pose is None:
            return
        x, y, yaw = pose

        dist = math.hypot(self.goal_x - x, self.goal_y - y)
        if dist < self.goal_tolerance:
            shield_note = f' Shield overrode {self.overrides} of {self.steps} actions.' if self.shield else ''
            self.get_logger().info('Goal Reached! Stopping robot.' + shield_note)
            self.publish_stop()
            self.goal_x = None
            return

        # Steering target: final goal, or next maze cell on the grid path in subgoal mode
        self.maze.x, self.maze.y = x, y
        tx, ty = self.maze.target()
        angle_diff = (math.atan2(ty - y, tx - x) - yaw + np.pi) % (2 * np.pi) - np.pi
        state = make_state(self.latest_scan, math.hypot(tx - x, ty - y), angle_diff, self.last_action)

        # Action from DQN (optionally restricted to safe actions by the shield)
        if self.shield:
            action_idx = self.shielded_action(state)
        else:
            action_idx = self.agent.select_action(state, evaluate=True)
        self.steps += 1
        self.last_action = action_idx
        v, w = self.actions[action_idx]
        if self.shield and min(self.latest_scan) < self.shield_slow_range:
            # Near a wall the shield also slows the chosen action: in-place turns at 1 rad/s sweep
            # the robot's body across the wall and make the wheels slip in Gazebo
            v, w = 0.6 * v, 0.5 * w
        self.publish_cmd(v, w)

    def shielded_action(self, state):
        """Highest-Q action among those that do not drive into a nearby obstacle.

        latest_scan holds the min range of 36 sectors of 10 deg (sector 0 straight ahead,
        increasing to the left). Forward actions are blocked when something is close ahead;
        when one side is close, driving straight and turning towards that side are blocked, so
        only the soft turn away from it (or a rotation) remains. In-place rotations are always
        allowed, so the robot can always turn away from a wall.
        """
        q = self.agent.q_net.forward(state)[0]
        scan = self.latest_scan
        front = min(scan[k % NUM_BEAMS] for k in range(-3, 4))       # -30 .. +30 deg
        left = min(scan[k] for k in range(4, 10))                     # 40 .. 90 deg
        right = min(scan[k] for k in range(27, 33))                   # -90 .. -40 deg
        near_left, near_right = left < self.shield_side, right < self.shield_side
        allowed = []
        for i, (v, w) in enumerate(self.actions):
            if v > 0.0:
                if front < self.shield_front:
                    continue
                if w > 0.0 and near_left or w < 0.0 and near_right:
                    continue
                if w == 0.0 and near_left != near_right:   # one wall close: steer away, not straight
                    continue
            allowed.append(i)
        if not any(self.actions[i][0] > 0.0 for i in allowed):
            # Only rotations are safe. Keep turning the same way until a forward action is safe
            # again; otherwise the Q-ranking can flip left/right every step and the robot
            # wiggles in place against the wall.
            if self.escape_action is None:
                self.escape_action = max(allowed, key=lambda i: q[i])
            best = self.escape_action
        else:
            self.escape_action = None
            best = max(allowed, key=lambda i: q[i])
        if best != int(np.argmax(q)):
            self.overrides += 1
        return best

    def publish_cmd(self, v, w):
        """Send a velocity command (TwistStamped unless enable_stamped_cmd_vel is false)."""
        if self.enable_stamped:
            t = TwistStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.base_frame
            t.twist.linear.x = v
            t.twist.angular.z = w
            self.cmd_pub.publish(t)
        else:
            t = Twist()
            t.linear.x = v
            t.angular.z = w
            self.cmd_pub.publish(t)

    def publish_stop(self):
        """Stop the robot."""
        self.publish_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = DQNNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
