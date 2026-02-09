"""
RaptorLift SLAM Mapping Launch File

Launches slam_toolbox in online_async mode for map building, plus the Nav2
controller and planner for simultaneous navigation during mapping.

The collision monitor is included as part of navigation_launch.py (composable
lifecycle node) and outputs to /nav/cmd_vel via nav2_params.yaml.

Usage:
  ros2 launch raptor_navigation slam.launch.py
  ros2 launch raptor_navigation slam.launch.py use_sim_time:=true use_rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package paths
    nav_pkg = get_package_share_directory("raptor_navigation")
    nav2_bringup_pkg = get_package_share_directory("nav2_bringup")

    # Default file paths
    default_slam_params = os.path.join(nav_pkg, "config", "slam_params.yaml")
    default_nav2_params = os.path.join(nav_pkg, "config", "nav2_params.yaml")
    default_bt_xml = os.path.join(
        nav_pkg, "behavior_trees", "ackermann_navigate.xml"
    )
    default_rviz_config = os.path.join(nav_pkg, "rviz", "navigation.rviz")

    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=default_slam_params,
            description="Full path to slam_toolbox params file",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=default_nav2_params,
            description="Full path to Nav2 params file",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically start Nav2 lifecycle nodes",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz2 with navigation config",
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")

    # slam_toolbox online_async node (provides map->odom TF, no AMCL needed)
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            LaunchConfiguration("slam_params_file"),
            {"use_sim_time": use_sim_time},
        ],
    )

    # Nav2 navigation stack (without AMCL/map_server since SLAM provides TF)
    # Includes collision_monitor as composable lifecycle node
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": LaunchConfiguration("nav2_params_file"),
            "autostart": LaunchConfiguration("autostart"),
            "default_bt_xml_filename": default_bt_xml,
        }.items(),
    )

    # RViz (optional)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_slam",
        arguments=["-d", default_rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + [
            slam_toolbox_node,
            nav2_navigation,
            rviz_node,
        ]
    )
