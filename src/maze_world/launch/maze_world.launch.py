"""
maze_world.launch.py - starts the simulation: Gazebo with the maze and the TurtleBot3 Burger.

Starts the Gazebo server with worlds/maze.world (and the GUI unless gui:=false), publishes
the robot description (robot_state_publisher) and spawns the Burger at (-0.75, -1.25) with
ros_gz_bridge, which connects /scan, /odom, /imu, /tf, /clock, /joint_states and /cmd_vel
between Gazebo and ROS 2. Every other part of the project needs this running first.

Arguments: gui (default true), x_pose / y_pose (spawn, default -0.75 / -1.25), use_sim_time.
Run: ros2 launch maze_world maze_world.launch.py [gui:=false]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Modelled on turtlebot3_gazebo/launch/turtlebot3_world.launch.py (ROS 2 Jazzy):
# Gazebo server + optional GUI with our maze.world, robot_state_publisher, and
# spawn_turtlebot3.launch.py (which spawns the Burger and starts the ros_gz_bridge
# for /clock, /scan, /odom, /tf, /imu, /joint_states and /cmd_vel).


def generate_launch_description():
    # Both included TurtleBot3 launch files read TURTLEBOT3_MODEL from the environment
    os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')

    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    launch_file_dir = os.path.join(tb3_gazebo, 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    maze_world_pkg = get_package_share_directory('maze_world')
    world = os.path.join(maze_world_pkg, 'worlds', 'maze.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-0.75')
    y_pose = LaunchConfiguration('y_pose', default='-1.25')
    gui = LaunchConfiguration('gui', default='true')

    declare_x_pose = DeclareLaunchArgument('x_pose', default_value='-0.75', description='Spawn x (world frame)')
    declare_y_pose = DeclareLaunchArgument('y_pose', default_value='-1.25', description='Spawn y (world frame)')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_gui = DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo GUI client')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(gui)
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # TurtleBot3 models (robot meshes) and our maze model
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(tb3_gazebo, 'models'))
    set_env_vars_maze = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(maze_world_pkg, 'models'))

    ld = LaunchDescription()

    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_gui)
    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_maze)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)

    return ld
