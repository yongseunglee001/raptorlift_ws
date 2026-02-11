"""
Dual LiDAR PointCloud2 → LaserScan Conversion

Converts two PointCloud2 sources into LaserScan topics for Nav2:
  - Hesai XT32 (360°): /lidar_points → /scan (AMCL, SLAM, costmap)
  - RSE1 Front (solid-state, limited FOV): /rslidar_front/points → /scan_front (costmap, collision_monitor)

Each node uses pointcloud_to_laserscan to extract a 2D slice from the 3D point cloud.
Height filtering (min_height/max_height) controls which points contribute to the scan.

Usage:
  ros2 launch raptor_navigation lidar_processing.launch.py
  ros2 launch raptor_navigation lidar_processing.launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time",
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")

    # Hesai XT32 (main 360° LiDAR) → /scan
    # Used by AMCL, SLAM, and costmap obstacle layers
    hesai_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="hesai_pointcloud_to_laserscan",
        remappings=[
            ("cloud_in", "/lidar_points"),
            ("scan", "/scan"),
        ],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "target_frame": "lidar_link",
                "transform_tolerance": 0.01,
                "min_height": -0.1,
                "max_height": 1.5,
                "angle_min": -3.14159,  # -pi (full 360°)
                "angle_max": 3.14159,   # +pi
                "angle_increment": 0.00436,  # ~0.25° → 1440 points
                "scan_time": 0.1,
                "range_min": 0.3,
                "range_max": 100.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
        output="screen",
    )

    # RSE1 Front (solid-state, limited FOV) → /scan_front
    # Used by costmap obstacle layers and collision_monitor only
    rse1_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="rse1_pointcloud_to_laserscan",
        remappings=[
            ("cloud_in", "/rslidar_front/points"),
            ("scan", "/scan_front"),
        ],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "target_frame": "lidar_rslidar_front_link",
                "transform_tolerance": 0.01,
                "min_height": -0.1,
                "max_height": 1.5,
                "angle_min": -1.5708,  # -90° (RSE1 limited FOV)
                "angle_max": 1.5708,   # +90°
                "angle_increment": 0.00436,  # ~0.25°
                "scan_time": 0.1,
                "range_min": 0.1,
                "range_max": 30.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + [
            hesai_to_scan,
            rse1_to_scan,
        ]
    )
