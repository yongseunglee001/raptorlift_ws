#!/usr/bin/env python3
"""
Isaac Sim 5.1 Setup Script for RaptorLift Robot

This script sets up the OmniGraph for ROS 2 joint control:
- Subscribes to: /raptorlift/joint_commands
- Publishes to: /raptorlift/joint_states
"""

# Check if running inside Isaac Sim
try:
    import omni
    import omni.graph.core as og
    from pxr import Usd, UsdGeom, Sdf
    ISAAC_SIM_AVAILABLE = True
except ImportError:
    ISAAC_SIM_AVAILABLE = False
    print("Warning: Isaac Sim modules not available. Run this script inside Isaac Sim.")


def setup_ros2_joint_control(robot_prim_path: str = "/World/raptorlift"):
    """
    Set up OmniGraph for ROS 2 joint control in Isaac Sim 5.1.
    """
    if not ISAAC_SIM_AVAILABLE:
        print("Error: Must run inside Isaac Sim")
        return False

    graph_path = "/World/RaptorLiftActionGraph"

    # Isaac Sim 5.1 node types
    node_types = {
        "on_playback_tick": "omni.graph.action.OnPlaybackTick",
        "subscribe_joint_state": "isaacsim.ros2.bridge.ROS2SubscribeJointState",
        "publish_joint_state": "isaacsim.ros2.bridge.ROS2PublishJointState",
        "publish_clock": "isaacsim.ros2.bridge.ROS2PublishClock",
        "articulation_controller": "isaacsim.core.nodes.IsaacArticulationController",
    }

    print(f"Using Isaac Sim 5.1 node types")
    print(f"Robot prim path: {robot_prim_path}")

    try:
        # Delete existing graph if it exists
        stage = omni.usd.get_context().get_stage()
        existing_prim = stage.GetPrimAtPath(graph_path)
        if existing_prim.IsValid():
            stage.RemovePrim(graph_path)
            print(f"Removed existing graph at {graph_path}")

        # Create OmniGraph (simplified - no ReadSimTime)
        keys = og.Controller.Keys
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", node_types["on_playback_tick"]),
                    ("SubscribeJointState", node_types["subscribe_joint_state"]),
                    ("ArticulationController", node_types["articulation_controller"]),
                    ("PublishJointState", node_types["publish_joint_state"]),
                    ("PublishClock", node_types["publish_clock"]),
                ],
                keys.CONNECT: [
                    # Execution flow
                    ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    ("SubscribeJointState.outputs:execOut", "ArticulationController.inputs:execIn"),
                    ("ArticulationController.outputs:execOut", "PublishJointState.inputs:execIn"),

                    # Joint command flow
                    ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                    ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                    ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                    ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                ],
                keys.SET_VALUES: [
                    # Subscribe settings
                    ("SubscribeJointState.inputs:topicName", "/raptorlift/joint_commands"),

                    # Articulation controller settings
                    ("ArticulationController.inputs:robotPath", robot_prim_path),
                    ("ArticulationController.inputs:usePath", True),

                    # Publish settings
                    ("PublishJointState.inputs:topicName", "/raptorlift/joint_states"),
                    ("PublishJointState.inputs:targetPrim", robot_prim_path),
                ],
            },
        )

        print(f"\n=== OmniGraph Created Successfully ===")
        print(f"Graph path: {graph_path}")
        print(f"Subscribing to: /raptorlift/joint_commands")
        print(f"Publishing to: /raptorlift/joint_states, /clock")
        print(f"\nPress Play to start simulation.")

        return True

    except Exception as e:
        print(f"Error creating OmniGraph: {e}")
        print("\nTrying minimal setup...")
        return setup_minimal(robot_prim_path)


def setup_minimal(robot_prim_path: str = "/World/raptorlift"):
    """
    Minimal setup with just joint state pub/sub.
    """
    graph_path = "/World/RaptorLiftActionGraph"

    try:
        # Delete existing graph
        stage = omni.usd.get_context().get_stage()
        existing_prim = stage.GetPrimAtPath(graph_path)
        if existing_prim.IsValid():
            stage.RemovePrim(graph_path)

        keys = og.Controller.Keys
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                    ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                    ("SubscribeJointState.outputs:execOut", "ArticulationController.inputs:execIn"),
                    ("ArticulationController.outputs:execOut", "PublishJointState.inputs:execIn"),
                    ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                    ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                    ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                    ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                ],
                keys.SET_VALUES: [
                    ("SubscribeJointState.inputs:topicName", "/raptorlift/joint_commands"),
                    ("ArticulationController.inputs:robotPath", robot_prim_path),
                    ("ArticulationController.inputs:usePath", True),
                    ("PublishJointState.inputs:topicName", "/raptorlift/joint_states"),
                    ("PublishJointState.inputs:targetPrim", robot_prim_path),
                ],
            },
        )

        print(f"\n=== Minimal OmniGraph Created ===")
        print(f"Subscribing to: /raptorlift/joint_commands")
        print(f"Publishing to: /raptorlift/joint_states")
        print(f"\nPress Play to start simulation.")
        return True

    except Exception as e:
        print(f"Minimal setup also failed: {e}")
        return False


# Run setup when script is executed
if ISAAC_SIM_AVAILABLE:
    robot_path = "/World/raptorlift"

    # Find robot prim
    try:
        stage = omni.usd.get_context().get_stage()
        if stage:
            test_paths = ["/World/raptorlift", "/raptorlift", "/World/Robot"]
            for path in test_paths:
                prim = stage.GetPrimAtPath(path)
                if prim.IsValid():
                    robot_path = path
                    print(f"Found robot at: {robot_path}")
                    break
            else:
                print(f"Warning: Robot not found. Using: {robot_path}")
                print("Available prims at /World:")
                world_prim = stage.GetPrimAtPath("/World")
                if world_prim.IsValid():
                    for child in world_prim.GetChildren():
                        print(f"  {child.GetPath()}")
    except Exception as e:
        print(f"Could not check stage: {e}")

    # Run setup
    setup_ros2_joint_control(robot_path)
else:
    print("Run this script inside Isaac Sim.")
