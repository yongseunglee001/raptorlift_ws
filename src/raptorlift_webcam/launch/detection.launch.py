"""Launch YOLOv11 detection node for RaptorLift webcam."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("raptorlift_webcam")
    config_file = os.path.join(pkg_dir, "config", "detection.yaml")
    default_model = os.path.join(pkg_dir, "models", "weights.pt")

    # -- Launch arguments --
    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="webcam",
        description="Camera namespace (must match webcam driver)",
    )
    declare_model_path = DeclareLaunchArgument(
        "model_path",
        default_value=default_model,
        description="Path to YOLO weights (.pt)",
    )
    declare_device = DeclareLaunchArgument(
        "device",
        default_value="cuda:0",
        description="Inference device (cuda:0, cpu)",
    )
    declare_confidence = DeclareLaunchArgument(
        "confidence_threshold",
        default_value="0.5",
        description="Minimum detection confidence",
    )

    # -- Detection node --
    detection_node = Node(
        package="raptorlift_webcam",
        executable="detection_node.py",
        name="detection_node",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            config_file,
            {
                "model_path": LaunchConfiguration("model_path"),
                "device": LaunchConfiguration("device"),
                "confidence_threshold": LaunchConfiguration("confidence_threshold"),
            },
        ],
        output="screen",
    )

    return LaunchDescription([
        declare_namespace,
        declare_model_path,
        declare_device,
        declare_confidence,
        detection_node,
    ])
