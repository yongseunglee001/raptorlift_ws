"""
RaptorLift Combined Bringup + Navigation Launch File

Launches the full stack: ros2_control, twist_mux, teleop, AND Nav2 navigation.
This is the one-command launch for a complete autonomous-capable system.

Architecture:
  raptorlift_bringup (ros2_control + twist_mux + teleop)
       +
  raptor_navigation (Nav2: AMCL + SmacPlannerHybrid + RPP + CollisionMonitor)
       =
  Full autonomous stack with teleop override (twist_mux priority)

Usage:
  # Navigation with existing map
  ros2 launch raptor_navigation bringup_with_nav.launch.py map:=/path/to/map.yaml

  # SLAM mapping mode (no map needed)
  ros2 launch raptor_navigation bringup_with_nav.launch.py slam:=True

  # Simulation mode
  ros2 launch raptor_navigation bringup_with_nav.launch.py \\
      map:=/path/to/map.yaml use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Package paths
    bringup_pkg = get_package_share_directory("raptorlift_bringup")
    nav_pkg = get_package_share_directory("raptor_navigation")

    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time (for Isaac Sim)",
        ),
        DeclareLaunchArgument(
            "simulation_mode",
            default_value="true",
            description="Hardware bridge simulation mode",
        ),
        DeclareLaunchArgument(
            "use_hardware_bridge",
            default_value="true",
            description="Launch hardware_bridge node",
        ),
        DeclareLaunchArgument(
            "use_teleop",
            default_value="true",
            description="Enable teleop nodes",
        ),
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Full path to map yaml file (required unless slam:=true)",
        ),
        DeclareLaunchArgument(
            "slam",
            default_value="False",
            description="Use SLAM instead of AMCL localization (use True/False)",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically start Nav2 lifecycle nodes",
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")

    # RaptorLift base stack (ros2_control + twist_mux + teleop)
    # RViz is disabled here; navigation launch provides its own RViz config
    raptorlift_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "raptorlift.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "simulation_mode": LaunchConfiguration("simulation_mode"),
            "use_hardware_bridge": LaunchConfiguration("use_hardware_bridge"),
            "use_rviz": "false",
            "use_teleop": LaunchConfiguration("use_teleop"),
        }.items(),
    )

    # Nav2 navigation stack (AMCL mode - requires map)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": LaunchConfiguration("map"),
            "autostart": LaunchConfiguration("autostart"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("slam")),
    )

    # SLAM mode (no map needed, slam_toolbox provides map->odom TF)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, "launch", "slam.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": LaunchConfiguration("autostart"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("slam")),
    )

    return LaunchDescription(
        declared_arguments
        + [
            raptorlift_bringup,
            navigation_launch,
            slam_launch,
        ]
    )
