"""
RaptorLift Unified Bringup Launch File

Complete manual control stack with ros2_control, twist_mux, teleop, and LiDAR drivers.

Architecture:
  ┌─────────────┐     ┌───────────┐     ┌────────────────┐
  │ gamepad     │────▶│           │     │                │
  │ /teleop/    │     │ twist_mux │────▶│ ackermann_     │
  │ cmd_vel     │     │           │     │ controller     │
  │ (gear-scaled│     │           │     └────────────────┘
  │  real m/s)  │     │           │
  └─────────────┘     │           │
  ┌─────────────┐     │           │
  │ keyboard    │────▶│           │
  │ /key/cmd_vel│     │           │
  └─────────────┘     │           │
  ┌─────────────┐     │           │
  │ Nav2        │────▶│           │
  │ /nav/cmd_vel│     └───────────┘
  └─────────────┘

  LiDAR Drivers:
  ┌──────────────────┐     /lidar_points (PointCloud2)
  │ Hesai XT32 (360°)│────▶ frame: lidar_link
  └──────────────────┘
  ┌──────────────────┐     /rslidar_front/points (PointCloud2)
  │ RSE1 Front (FOV) │────▶ frame: lidar_rslidar_front_link
  └──────────────────┘

  Webcam:
  ┌──────────────────┐     /webcam/image_raw (Image)
  │ Logitech C922    │────▶ frame: webcam_optical
  └──────────────────┘

Usage:
  # Simulation mode (default)
  ros2 launch raptorlift_bringup raptorlift.launch.py

  # Real hardware mode
  ros2 launch raptorlift_bringup raptorlift.launch.py simulation_mode:=false

  # Isaac Sim mode (no hardware_bridge, no lidars)
  ros2 launch raptorlift_bringup raptorlift.launch.py use_hardware_bridge:=false use_lidars:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package paths
    description_pkg = get_package_share_directory("raptorlift_description")
    bringup_pkg = get_package_share_directory("raptorlift_bringup")
    teleop_pkg = get_package_share_directory("raptorlift_teleop")
    webcam_pkg = get_package_share_directory("raptorlift_webcam")

    # File paths
    urdf_file = os.path.join(description_pkg, "urdf", "raptorlift.urdf")
    controllers_file = os.path.join(bringup_pkg, "config", "raptorlift_controllers.yaml")
    twist_mux_config = os.path.join(teleop_pkg, "config", "twist_mux.yaml")

    # Read URDF
    with open(urdf_file, "r") as f:
        robot_description_content = f.read()

    robot_description = {"robot_description": robot_description_content}

    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time (for Isaac Sim)",
        ),
        DeclareLaunchArgument(
            "use_hardware_bridge",
            default_value="true",
            description="Launch hardware_bridge node (disable for Isaac Sim direct control)",
        ),
        DeclareLaunchArgument(
            "simulation_mode",
            default_value="true",
            description="Hardware bridge simulation mode (true=echo, false=real servos)",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 for visualization",
        ),
        DeclareLaunchArgument(
            "use_teleop",
            default_value="true",
            description="Enable teleop nodes (gamepad + keyboard support)",
        ),
        DeclareLaunchArgument(
            "use_lidars",
            default_value="true",
            description="Launch LiDAR drivers (Hesai XT32 + RSE1 Front)",
        ),
        DeclareLaunchArgument(
            "control_rate",
            default_value="100.0",
            description="Control loop rate (Hz)",
        ),
        DeclareLaunchArgument(
            "verbose",
            default_value="false",
            description="Enable detailed control pipeline logging (cmd→PID→PLC→state)",
        ),
        DeclareLaunchArgument(
            "use_webcam",
            default_value="true",
            description="Launch USB webcam driver (Logitech C922 via usb_cam)",
        ),
    ]

    # ==================== ROS2 CONTROL ====================

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # Controller Manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_file,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawn ackermann_steering_controller
    # Remappings route the controller's namespaced odom/TF topics to
    # the standard /odom and /tf expected by Nav2.
    ackermann_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ackermann_steering_controller",
            "--controller-manager", "/controller_manager",
            "--controller-ros-args",
            "--remap /ackermann_steering_controller/tf_odometry:=/tf --remap /ackermann_steering_controller/odometry:=/odom",
        ],
        output="screen",
    )

    # Delay ackermann_controller spawn
    delay_ackermann_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[ackermann_controller_spawner],
        )
    )

    # Hardware Bridge
    hardware_bridge_node = Node(
        package="raptorlift_hardware_bridge",
        executable="hardware_bridge",
        name="hardware_bridge",
        output="screen",
        parameters=[
            {"simulation_mode": LaunchConfiguration("simulation_mode")},
            {"control_rate": LaunchConfiguration("control_rate")},
            {"detailed_logging": LaunchConfiguration("verbose")},
            {"steering_kp": 50.0},
            {"steering_ki": 0.0},
            {"steering_kd": 5.0},
            {"max_steering_torque": 100.0},
            {"traction_kp": 10.0},
            {"traction_ki": 5.0},
            {"traction_kd": 0.0},
            {"max_traction_torque": 150.0},
        ],
        condition=IfCondition(LaunchConfiguration("use_hardware_bridge")),
    )

    # ==================== TWIST MUX ====================

    # twist_mux: selects between multiple cmd_vel sources
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[twist_mux_config],
        remappings=[
            ("cmd_vel_out", "/ackermann_steering_controller/reference"),
        ],
        output="screen",
    )

    # ==================== TELEOP ====================

    # Joy node (gamepad driver)
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            {
                "device_id": 0,
                "deadzone": 0.05,
                "autorepeat_rate": 20.0,
            }
        ],
        condition=IfCondition(LaunchConfiguration("use_teleop")),
        output="screen",
    )

    # Gamepad teleop node
    gamepad_teleop_node = Node(
        package="raptorlift_teleop",
        executable="gamepad_teleop",
        name="gamepad_teleop",
        parameters=[
            {
                "num_gears": 20,
                "deadzone": 0.1,
                "publish_rate": 50.0,
                "joy_timeout": 0.5,
                "min_linear_vel": 0.05,
                "max_linear_vel": 1.0,
                "max_angular_vel": 0.5,
            }
        ],
        remappings=[
            ("cmd_vel", "/teleop/cmd_vel"),
            ("gear_state", "/gear_state"),
        ],
        condition=IfCondition(LaunchConfiguration("use_teleop")),
        output="screen",
    )

    # ==================== LIDAR DRIVERS ====================

    # Hesai XT32 (360° main LiDAR)
    # Publishes: /lidar_points (PointCloud2), frame: lidar_link
    # Config loaded from hesai_ros_driver package default config.yaml
    hesai_lidar_node = Node(
        package="hesai_ros_driver",
        executable="hesai_ros_driver_node",
        name="hesai_lidar",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_lidars")),
    )

    # RoboSense RSE1 Front (solid-state, limited FOV)
    # Publishes: /rslidar_front/points (PointCloud2), frame: lidar_rslidar_front_link
    rslidar_ws_src = os.path.expanduser(
        "~/RaptorYamato/IsaacSim-ros_workspaces/jazzy_ws/src"
    )
    rslidar_config_file = os.path.join(
        rslidar_ws_src, "rslidar_sdk", "config", "config.yaml"
    )
    rslidar_front_node = Node(
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        name="rslidar_front",
        output="screen",
        parameters=[{"config_path": rslidar_config_file}],
        condition=IfCondition(LaunchConfiguration("use_lidars")),
    )

    # ==================== VISUALIZATION ====================

    # RViz
    rviz_config_file = os.path.join(description_pkg, "rviz", "raptorlift.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    # ==================== WEBCAM ====================

    webcam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(webcam_pkg, "launch", "webcam.launch.py")
        ),
        launch_arguments={
            "namespace": "webcam",
            "video_device": "/dev/video0",
            "show_image": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_webcam")),
    )

    # ==================== ASSEMBLE ====================

    nodes = [
        # ROS2 Control
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_ackermann_controller,
        hardware_bridge_node,
        # LiDAR drivers
        hesai_lidar_node,
        rslidar_front_node,
        # Velocity mux
        twist_mux_node,
        # Teleop
        joy_node,
        gamepad_teleop_node,
        # Webcam
        webcam_launch,
        # Visualization
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
