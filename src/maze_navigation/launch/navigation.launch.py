"""
navigation.launch.py - starts the full Nav2 stack on the saved maze map, plus RViz.

Nav2 (via nav2_bringup's bringup_launch.py): map_server + AMCL localisation, the NavFn
global planner, the Regulated Pure Pursuit controller, costmaps, behaviours and the BT
navigator that executes goals. All settings are in config/nav2_params.yaml.

Arguments
  map         map yaml (default maps/maze_map.yaml)
  params_file Nav2 parameters (default config/nav2_params.yaml)
  use_astar   false = Dijkstra (default), true = A*  (rewrites planner GridBased.use_astar)
  use_rviz    start RViz (default true)
Run: ros2 launch maze_navigation navigation.launch.py [use_astar:=true]
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_share = get_package_share_directory("maze_navigation")
    default_map = os.path.join(pkg_share, "maps", "maze_map.yaml")
    default_params = os.path.join(pkg_share, "config", "nav2_params.yaml")
    default_rviz = os.path.join(pkg_share, "rviz", "nav2.rviz")

    map_arg = DeclareLaunchArgument("map", default_value=default_map, description="Path to map file")
    params_arg = DeclareLaunchArgument("params_file", default_value=default_params, description="Nav2 params file")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="True", description="Use simulation clock")
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="True", description="Start RViz2")
    use_astar_arg = DeclareLaunchArgument("use_astar", default_value="false",
                                          description="NavFn planner: false = Dijkstra, true = A*")

    actions = [map_arg, params_arg, use_sim_time_arg, use_rviz_arg, use_astar_arg]

    # Override planner_server.GridBased.use_astar from the launch argument
    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        root_key="",
        param_rewrites={"use_astar": LaunchConfiguration("use_astar")},
        convert_types=True,
    )

    # Start Nav2 through its standard bring-up launch file with our map and parameters
    try:
        nav2_bringup_share = get_package_share_directory("nav2_bringup")
        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "params_file": configured_params,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "autostart": "true",
            }.items(),
        )
        actions.append(nav2_launch)
    except Exception:
        actions.append(LogInfo(
            msg="\n[WARNING] 'nav2_bringup' is not installed! Install via:\n  sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup\n"
        ))

    # RViz with the project's view (map, costmaps, path, Nav2 Goal / 2D Goal Pose tools)
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
