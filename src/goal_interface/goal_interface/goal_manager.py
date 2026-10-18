#!/usr/bin/env python3
"""
goal_manager.py - turns points clicked in RViz into Nav2 navigation goals.

RViz offers several ways to click a goal; this node accepts two of them, checks that the goal
lies inside the maze (|x|, |y| <= 1.45 m in the map frame, which equals the Gazebo world frame)
and sends it to Nav2's NavigateToPose action, then logs progress and the result:
  /goal_pose      (geometry_msgs/PoseStamped)  - RViz "2D Goal Pose"
  /clicked_point  (geometry_msgs/PointStamped) - RViz "Publish Point" (heading set to 0)
RViz's "Nav2 Goal" tool talks to Nav2 directly and does not need this node.

Run (with Nav2 running):  ros2 run goal_interface goal_manager
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import math

class GoalManager(Node):
    """Bounds-checks clicked goals and forwards them to Nav2."""
    def __init__(self):
        super().__init__('goal_manager')
        self.get_logger().info('Initializing Goal Manager for IE4060 Navigation...')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscriptions
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)

        self._active_goal_handle = None
        self.get_logger().info('Goal Manager is active. Waiting for goals from RViz or click interface...')

    def is_valid_coordinate(self, x, y):
        """True if (x, y) is inside the maze walls (with a 5 cm margin)."""
        # Maze boundaries are -1.5m to 1.5m. The map frame equals the Gazebo world frame
        # (map origin shifted by the spawn pose, see scripts/align_map_origin.py).
        if abs(x) > 1.45 or abs(y) > 1.45:
            self.get_logger().warn(f'Goal ({x:.2f}, {y:.2f}) is outside maze bounds [-1.45, 1.45].')
            return False
        return True

    def goal_pose_callback(self, msg: PoseStamped):
        """RViz '2D Goal Pose' click: forward it to Nav2 if it is inside the maze."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(f'Received clicked Goal Pose: x={x:.2f}, y={y:.2f}')
        if self.is_valid_coordinate(x, y):
            self.send_navigation_goal(msg)

    def clicked_point_callback(self, msg: PointStamped):
        """RViz 'Publish Point' click: build a goal pose (heading 0) and forward it."""
        x = msg.point.x
        y = msg.point.y
        self.get_logger().info(f'Received Clicked Point: x={x:.2f}, y={y:.2f}')
        if not self.is_valid_coordinate(x, y):
            return

        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        self.send_navigation_goal(pose_msg)

    def send_navigation_goal(self, pose: PoseStamped):
        """Send the pose to Nav2's NavigateToPose action server (asynchronously)."""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 NavigateToPose action server not available!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f'Sending Nav2 goal to ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Nav2 accepted or rejected the goal; if accepted, wait for the result."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by Nav2 planner!')
            return

        self.get_logger().info('Goal accepted by Nav2 planner. Executing path...')
        self._active_goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Progress from Nav2 (logged at most every 2 s)."""
        feedback = feedback_msg.feedback
        dist = feedback.distance_remaining
        self.get_logger().info(f'Navigating... Distance remaining: {dist:.2f} m', throttle_duration_sec=2.0)

    def get_result_callback(self, future):
        """Final result of the navigation (succeeded, aborted or cancelled)."""
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('>>> SUCCESS: Goal reached successfully! <<<')
        else:
            self.get_logger().warn(f'Navigation finished with status code: {status}')

def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
