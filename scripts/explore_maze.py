#!/usr/bin/env python3
"""
Scripted exploration for building the SLAM map: drives the Burger through every maze
cell (depth-first over the cell graph from maze_dqn.maze_env, backtracking at dead ends)
so slam_toolbox sees every wall.

Steering uses /odom at 30 Hz. Before each move the odom pose is re-anchored to the Gazebo
ground-truth pose (`gz model -m burger -p`), like a human driver watching the simulator,
so odometry drift never steers the robot into a wall. slam_toolbox itself only sees /odom
and /scan; the ground truth is never published to ROS.

Motion is stop-turn-drive with ramped, slow commands (turns <= 0.3 rad/s): in this world
fast or abruptly reversed in-place turns make the wheels slip (1.2 rad/s loses ~12 deg per
90 deg turn), which corrupts odometry and the map.

Usage (Gazebo and slam.launch.py already running):
  python3 scripts/explore_maze.py
"""
import math
import os
import re
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'maze_dqn'))
from maze_dqn.maze_env import MazeEnvironment  # noqa: E402

SPAWN = (-0.75, -1.25)   # robot spawn in the world frame (see maze_world.launch.py)
MODEL = 'burger'         # Gazebo model name of the spawned robot


def yaw_of(q):
    """Heading (rad) from a quaternion."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def dfs_route(env, start):
    """Depth-first tour of the cell graph from start: each cell is visited and the robot
    returns along the same corridor at dead ends, so every wall is seen."""
    route, seen = [start], {start}

    def visit(c):
        """Visit every unvisited neighbour of c, then come back to c."""
        for nb in env.adj[c]:
            if nb not in seen:
                seen.add(nb)
                route.append(nb)
                visit(nb)
                route.append(c)
    visit(start)
    return route


class Explorer(Node):
    """Drives the robot through the maze with stop-turn-drive motion."""
    def __init__(self):
        """Subscribe to /odom and create the /cmd_vel publisher; start at the spawn pose."""
        super().__init__('maze_explorer')
        self.odom = None                          # raw odom pose (x, y, yaw)
        self.anchor = (SPAWN[0], SPAWN[1], 0.0)   # world <- odom transform
        self.pose = (SPAWN[0], SPAWN[1], 0.0)     # last ground-truth pose
        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

    def odom_cb(self, m):
        """Store the latest wheel-odometry pose."""
        p = m.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))

    def truth(self):
        """Robot pose from Gazebo (gz model -p); used only to correct the steering."""
        for _ in range(5):
            out = subprocess.run(['gz', 'model', '-m', MODEL, '-p'], capture_output=True, text=True).stdout
            if 'Pose' in out:
                nums = re.findall(r'\[([-\d.e ]+)\]', out.split('Pose')[1])
                if len(nums) >= 2:
                    xyz = [float(v) for v in nums[0].split()]
                    rpy = [float(v) for v in nums[1].split()]
                    return xyz[0], xyz[1], rpy[2]
        return self.pose

    def world(self):
        """Odometry pose converted to the world frame with the last anchor."""
        ox, oy, oth = self.odom
        ax, ay, ath = self.anchor
        c, s = math.cos(ath), math.sin(ath)
        return ax + c * ox - s * oy, ay + s * ox + c * oy, oth + ath

    def reanchor(self):
        """Re-align odometry with the Gazebo pose, so drift never steers into a wall."""
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.05)
        tx, ty, tth = self.pose = self.truth()
        ox, oy, oth = self.odom
        ath = tth - oth
        c, s = math.cos(ath), math.sin(ath)
        self.anchor = (tx - (c * ox - s * oy), ty - (s * ox + c * oy), ath)

    def cmd(self, v, w):
        """Publish a velocity command (TwistStamped)."""
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.twist.linear.x = float(v)
        t.twist.angular.z = float(w)
        self.pub.publish(t)

    def settle(self, sec=0.4):
        """Send zero velocity for a short time so the robot stops completely."""
        t = time.time()
        while time.time() - t < sec:
            self.cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def turn_to(self, target, timeout=20.0):
        """Turn in place to a heading with a ramped, slow rotation (<= 0.3 rad/s)."""
        w, t0 = 0.0, time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            err = wrap(target - self.world()[2])
            if abs(err) < 0.03:
                break
            want = max(-0.3, min(0.3, 1.0 * err))
            if abs(want) < 0.08:
                want = math.copysign(0.08, err)
            w += max(-0.025, min(0.025, want - w))  # ramp: no abrupt reversal
            self.cmd(0.0, w)
        self.settle()

    def goto(self, tx, ty, timeout=60.0):
        """Turn towards (tx, ty), then drive to it; True if the robot ended within 8 cm."""
        self.reanchor()
        x, y, _ = self.world()
        if math.hypot(tx - x, ty - y) < 0.05:
            return True
        self.turn_to(math.atan2(ty - y, tx - x))
        v, t0 = 0.0, time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            x, y, th = self.world()
            d = math.hypot(tx - x, ty - y)
            err = wrap(math.atan2(ty - y, tx - x) - th)
            if d < 0.03 or abs(err) > math.pi / 2:  # reached, or just passed it
                break
            want = min(0.10, 0.5 * d + 0.02)
            v += max(-0.01, min(0.005, want - v))
            self.cmd(v, max(-0.3, min(0.3, 1.0 * err)))
        self.settle()
        self.reanchor()
        return math.hypot(tx - self.pose[0], ty - self.pose[1]) < 0.08


def main():
    """Plan the route over all 36 cells and drive it, then turn once on the spot."""
    rclpy.init()
    node = Explorer()
    env = MazeEnvironment()
    route = dfs_route(env, env.cell_of(*SPAWN))
    print(f'route: {len(route)} waypoints over {len(set(route))} cells', flush=True)
    while node.odom is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    t0 = time.time()
    for i, c in enumerate(route[1:], 1):
        if not node.goto(*env.cell_centre(c)):
            print(f'waypoint {i} {c}: not reached, robot at {node.pose}', flush=True)
        if i % 5 == 0:
            print(f'{i}/{len(route) - 1} waypoints, {time.time() - t0:.0f} s', flush=True)
    t = time.time()                      # one slow turn on the spot at the end
    while time.time() - t < 22:
        node.cmd(0.0, 0.3)
        rclpy.spin_once(node, timeout_sec=0.1)
    node.settle()
    print(f'EXPLORATION DONE in {time.time() - t0:.0f} s. Save the map now.', flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
