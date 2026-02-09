"""
RaptorLift Nav2 Navigation Launch File

Launches the full Nav2 stack with AMCL localization, SmacPlannerHybrid (Hybrid-A*),
RegulatedPurePursuit controller, and collision monitor. Optimized for rear-steer
Ackermann kinematics.

Architecture:
  /scan + /odom  -->  AMCL (map->odom TF)
  /goal_pose     -->  SmacPlannerHybrid (REEDS_SHEPP)
                 -->  RegulatedPurePursuit
                 -->  VelocitySmoother
                 -->  CollisionMonitor (cmd_vel_out -> /nav/cmd_vel)
                 -->  twist_mux (priority 10)

Collision monitor is loaded as a composable lifecycle node by nav2_bringup.
Its output topic is set to /nav/cmd_vel via nav2_params.yaml.

Usage:
  ros2 launch raptor_navigation navigation.launch.py map:=/path/to/map.yaml
  ros2 launch raptor_navigation navigation.launch.py map:=/path/to/map.yaml use_sim_time:=true
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
    default_params_file = os.path.join(nav_pkg, "config", "nav2_params.yaml")
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
            "map",
            default_value="",
            description="Full path to map yaml file",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
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

    # Nav2 bringup (localization + navigation + collision_monitor as composable nodes)
    # Collision monitor params are in nav2_params.yaml with cmd_vel_out_topic: /nav/cmd_vel
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
            "autostart": LaunchConfiguration("autostart"),
            "default_bt_xml_filename": default_bt_xml,
        }.items(),
    )

    # RViz (optional)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_nav",
        arguments=["-d", default_rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + [
            nav2_bringup,
            rviz_node,
        ]
    )
