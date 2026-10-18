import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def localization(context):
    """Map server + AMCL + their lifecycle manager, optionally on imu_odom.py's odometry tree."""
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    rewrites = {
        # The map path is written into the parameter file: started on its own, Jazzy's
        # nav2_bringup localization_launch.py leaves the map server's yaml_filename empty
        "yaml_filename": LaunchConfiguration("map").perform(context),
    }
    actions = []
    if LaunchConfiguration("odom").perform(context) == "imu":
        # AMCL's motion model then gets the gyro heading instead of the slipping wheel heading
        actions.append(Node(package="maze_navigation", executable="imu_odom.py", output="screen",
                            parameters=[{"use_sim_time": use_sim_time}]))
        rewrites.update({"odom_frame_id": "odom_imu",
                         "base_frame_id": "base_footprint_imu",
                         "scan_topic": "scan_imu"})
    params = RewrittenYaml(source_file=LaunchConfiguration("params_file").perform(context),
                           root_key="", param_rewrites=rewrites, convert_types=True)
    common = [params, {"use_sim_time": use_sim_time}]
    actions += [
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen", parameters=common),
        Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen", parameters=common),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_localization", output="screen",
             parameters=[{"use_sim_time": use_sim_time, "autostart": True,
                          "node_names": ["map_server", "amcl"]}]),
    ]
    return actions


def generate_launch_description():
    """Map server + AMCL only (no planner, controller or bt_navigator).

    Used with the DQN controller: it provides the map frame, so dqn_navigation_node and
    viva_demo get the AMCL pose instead of raw wheel odometry, and nothing else listens on
    /goal_pose. With odom:=imu, AMCL runs on imu_odom.py's tree (map -> odom_imu ->
    base_footprint_imu); then pass base_frame:=base_footprint_imu to the DQN node and viva_demo.
    """
    pkg_share = get_package_share_directory("maze_navigation")
    default_map = os.path.join(pkg_share, "maps", "maze_map.yaml")
    default_params = os.path.join(pkg_share, "config", "nav2_params.yaml")
    default_rviz = os.path.join(pkg_share, "rviz", "nav2.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", default_rviz],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=default_map, description="Path to map file"),
        DeclareLaunchArgument("params_file", default_value=default_params, description="AMCL / map_server params"),
        DeclareLaunchArgument("use_sim_time", default_value="True", description="Use simulation clock"),
        DeclareLaunchArgument("use_rviz", default_value="True", description="Start RViz2"),
        DeclareLaunchArgument("odom", default_value="wheel",
                              description="wheel: AMCL on /odom; imu: AMCL on imu_odom.py (wheel speed + gyro)"),
        OpaqueFunction(function=localization),
        rviz_node,
    ])
