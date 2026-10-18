#!/usr/bin/env python3
"""
Wheel + gyro odometry for SLAM mapping.

In this Gazebo world every in-place turn of the Burger leaves a few degrees of heading
error in the wheel odometry (/odom), while the simulated IMU gyro is accurate to ~0.1 deg
over many turns. Heading error is what smears and doubles walls in the SLAM map, so this
node integrates the wheel forward speed (/odom twist) with the IMU yaw rate (/imu) and
publishes the result on a separate TF tree used only by slam_toolbox:

  odom_imu -> base_footprint_imu -> base_scan_imu      (+ /scan republished as /scan_imu)

The normal odom -> base_footprint tree from Gazebo is left untouched, so Nav2, AMCL and the
DQN nodes are unaffected. Only onboard sensors are used (wheel encoders, IMU, LiDAR).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener, StaticTransformBroadcaster


def stamp_sec(stamp):
    """ROS time stamp -> seconds (float)."""
    return stamp.sec + stamp.nanosec * 1e-9


class ImuOdom(Node):
    """Integrates wheel speed with gyro heading and publishes it as odom_imu -> base_footprint_imu."""
    def __init__(self):
        """Read the frame names, create the TF broadcasters and the subscriptions."""
        super().__init__('imu_odom')
        self.odom_frame = self.declare_parameter('odom_frame', 'odom_imu').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint_imu').value
        self.laser_frame = self.declare_parameter('laser_frame', 'base_scan_imu').value
        self.src_base = self.declare_parameter('source_base_frame', 'base_footprint').value
        self.src_laser = self.declare_parameter('source_laser_frame', 'base_scan').value

        self.x = self.y = self.yaw = 0.0
        self.last_imu_t = None
        self.last_odom_t = None
        self.laser_ready = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.static_pub = StaticTransformBroadcaster(self)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_imu', 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, 100)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 50)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_timer(0.5, self.copy_laser_mount)

    def copy_laser_mount(self):
        """Mount the laser on the new base exactly as it is mounted on base_footprint."""
        if self.laser_ready:
            return
        try:
            t = self.tf_buffer.lookup_transform(self.src_base, self.src_laser, Time())
        except TransformException:
            return
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.laser_frame
        self.static_pub.sendTransform(t)
        self.laser_ready = True
        self.get_logger().info(f'{self.base_frame} -> {self.laser_frame} copied from {self.src_base} -> {self.src_laser}')

    def imu_cb(self, msg):
        """Integrate the gyro yaw rate into the heading (the accurate part of the estimate)."""
        t = stamp_sec(msg.header.stamp)
        if self.last_imu_t is not None and t > self.last_imu_t:
            self.yaw += msg.angular_velocity.z * (t - self.last_imu_t)
        self.last_imu_t = t

    def odom_cb(self, msg):
        """Advance x, y with the wheel forward speed along the gyro heading and publish the TF."""
        t = stamp_sec(msg.header.stamp)
        if self.last_odom_t is not None and t > self.last_odom_t:
            dt = t - self.last_odom_t
            v = msg.twist.twist.linear.x
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
        self.last_odom_t = t

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = math.sin(self.yaw / 2.0)
        tf.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_pub.sendTransform(tf)

    def scan_cb(self, msg):
        """Republish the LiDAR scan in the laser frame of the new tree (/scan_imu)."""
        if not self.laser_ready:
            return
        msg.header.frame_id = self.laser_frame
        self.scan_pub.publish(msg)


def main():
    """Run the node until Ctrl+C."""
    rclpy.init()
    node = ImuOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
