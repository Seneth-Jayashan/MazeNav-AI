"""
slam.launch.py - builds a map of the maze with SLAM Toolbox.

Starts imu_odom.py (wheel speed + IMU gyro odometry on its own TF tree, see scripts/imu_odom.py)
and SLAM Toolbox in online asynchronous mode with config/slam_params.yaml, plus RViz. Drive the
robot through the maze (scripts/explore_maze.py or the keyboard), then save the map with
nav2_map_server map_saver_cli and run scripts/align_map_origin.py and scripts/pad_map.py.

Arguments: use_rviz (default true), slam_params_file (default config/slam_params.yaml).
Run: ros2 launch maze_navigation slam.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    pkg_share = get_package_share_directory("maze_navigation")
    default_rviz = os.path.join(pkg_share, "rviz", "nav2.rviz")
    actions = [
        DeclareLaunchArgument("use_sim_time", default_value="True"),
        DeclareLaunchArgument("use_rviz", default_value="True", description="Start RViz2"),
        DeclareLaunchArgument("slam_params_file",
                              default_value=os.path.join(pkg_share, "config", "slam_params.yaml"),
                              description="SLAM Toolbox parameters"),
    ]

    # Wheel + gyro odometry for SLAM (see scripts/imu_odom.py)
    actions.append(Node(
        package="maze_navigation",
        executable="imu_odom.py",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    ))

    try:
        slam_toolbox_share = get_package_share_directory("slam_toolbox")
        slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_share, "launch", "online_async_launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "slam_params_file": LaunchConfiguration("slam_params_file"),
            }.items()
        )
        actions.append(slam_launch)
    except Exception:
        actions.append(LogInfo(
            msg="\n[WARNING] 'slam_toolbox' is not installed! Install via:\n  sudo apt install -y ros-jazzy-slam-toolbox\n"
        ))

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", default_rviz],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    actions.append(rviz_node)

    return LaunchDescription(actions)
