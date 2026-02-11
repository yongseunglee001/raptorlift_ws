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
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


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
            default_value="false",
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

    # LiDAR processing: PointCloud2 → LaserScan conversion
    # Hesai XT32 → /scan (360°), RSE1 Front → /scan_front (limited FOV)
    lidar_processing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, "launch", "lidar_processing.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # slam_toolbox online_async node (LifecycleNode: requires configure→activate)
    # Provides map→odom TF, no AMCL needed
    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            LaunchConfiguration("slam_params_file"),
            {"use_sim_time": use_sim_time, "use_lifecycle_manager": False},
        ],
    )

    # Autostart: configure slam_toolbox when process starts
    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == slam_toolbox_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )

    # Autostart: activate slam_toolbox after configure completes
    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[slam.launch.py] slam_toolbox configured, activating..."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node == slam_toolbox_node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
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
            lidar_processing,
            slam_toolbox_node,
            configure_slam,
            activate_slam,
            nav2_navigation,
            rviz_node,
        ]
    )
