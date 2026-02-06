"""
RaptorLift Unified Bringup Launch File

Complete manual control stack with ros2_control, twist_mux, and teleop.

Architecture:
  ┌─────────────┐     ┌───────────┐     ┌─────────────────┐     ┌────────────────┐
  │ gamepad     │────▶│           │     │                 │     │                │
  │ /teleop/    │     │ twist_mux │────▶│ velocity_scaler │────▶│ ackermann_     │
  │ cmd_vel     │     │           │     │ (gear scaling)  │     │ controller     │
  └─────────────┘     │           │     └─────────────────┘     └────────────────┘
  ┌─────────────┐     │           │              ▲
  │ keyboard    │────▶│           │              │
  │ /key/cmd_vel│     └───────────┘       /gear_state
  └─────────────┘                               │
                                          gamepad_teleop

Usage:
  # Simulation mode (default)
  ros2 launch raptorlift_bringup raptorlift.launch.py

  # Real hardware mode
  ros2 launch raptorlift_bringup raptorlift.launch.py simulation_mode:=false

  # Isaac Sim mode (no hardware_bridge)
  ros2 launch raptorlift_bringup raptorlift.launch.py use_hardware_bridge:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package paths
    description_pkg = get_package_share_directory("raptorlift_description")
    bringup_pkg = get_package_share_directory("raptorlift_bringup")
    teleop_pkg = get_package_share_directory("raptorlift_teleop")

    # File paths
    urdf_file = os.path.join(description_pkg, "urdf", "raptorlift.urdf")
    controllers_file = os.path.join(bringup_pkg, "config", "raptorlift_controllers.yaml")
    twist_mux_config = os.path.join(teleop_pkg, "config", "twist_mux.yaml")
    velocity_scaler_config = os.path.join(teleop_pkg, "config", "velocity_scaler.yaml")

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
            "control_rate",
            default_value="100.0",
            description="Control loop rate (Hz)",
        ),
        DeclareLaunchArgument(
            "verbose",
            default_value="false",
            description="Enable detailed control pipeline logging (cmd→PID→PLC→state)",
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
    ackermann_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller", "--controller-manager", "/controller_manager"],
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

    # ==================== TWIST MUX + VELOCITY SCALER ====================

    # twist_mux: selects between multiple cmd_vel sources
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[twist_mux_config],
        remappings=[
            ("cmd_vel_out", "/cmd_vel_mux"),
        ],
        output="screen",
    )

    # velocity_scaler: applies gear-based scaling
    velocity_scaler_node = Node(
        package="raptorlift_teleop",
        executable="velocity_scaler",
        name="velocity_scaler",
        parameters=[velocity_scaler_config],
        remappings=[
            ("cmd_vel_in", "/cmd_vel_mux"),
            ("gear_state", "/gear_state"),
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
            }
        ],
        remappings=[
            ("cmd_vel", "/teleop/cmd_vel"),
            ("gear_state", "/gear_state"),
        ],
        condition=IfCondition(LaunchConfiguration("use_teleop")),
        output="screen",
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

    # ==================== ASSEMBLE ====================

    nodes = [
        # ROS2 Control
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_ackermann_controller,
        hardware_bridge_node,
        # Velocity control stack
        twist_mux_node,
        velocity_scaler_node,
        # Teleop
        joy_node,
        gamepad_teleop_node,
        # Visualization
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
