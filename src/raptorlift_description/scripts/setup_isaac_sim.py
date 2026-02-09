#!/usr/bin/env python3
"""
Isaac Sim 5.1 Setup Script for RaptorLift Robot

Sets up:
1. Physics scene + ground plane (prevents robot falling)
2. OmniGraph for ROS 2 joint control + clock

Run inside Isaac Sim Script Editor (Ctrl+Enter) AFTER importing URDF.
"""

try:
    import omni
    import omni.kit.commands
    import omni.graph.core as og
    from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
    ISAAC_SIM_AVAILABLE = True
except ImportError:
    ISAAC_SIM_AVAILABLE = False
    print("Error: Must run inside Isaac Sim.")


def ensure_physics_scene():
    """Create PhysicsScene with gravity if it doesn't exist."""
    stage = omni.usd.get_context().get_stage()
    physics_path = "/World/PhysicsScene"

    if stage.GetPrimAtPath(physics_path).IsValid():
        print(f"PhysicsScene already exists at {physics_path}")
        return

    scene = UsdPhysics.Scene.Define(stage, physics_path)
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    print(f"Created PhysicsScene at {physics_path} (gravity: -Z, 9.81)")


def ensure_ground_plane():
    """Create a ground plane if none exists."""
    stage = omni.usd.get_context().get_stage()

    # Check existing ground
    for path in ["/World/GroundPlane", "/World/groundPlane", "/World/Ground"]:
        if stage.GetPrimAtPath(path).IsValid():
            print(f"Ground plane already exists at {path}")
            return

    ground_path = "/World/GroundPlane"

    # Method 1: omni.physx utility
    try:
        from omni.physx.scripts import utils as physx_utils
        physx_utils.addGroundPlane(
            stage, ground_path, "Z", 50.0,
            Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5)
        )
        print(f"Created ground plane at {ground_path} (physx_utils)")
        return
    except Exception as e:
        print(f"  physx_utils.addGroundPlane failed: {e}")

    # Method 2: Kit command
    try:
        omni.kit.commands.execute(
            "AddGroundPlaneCommand",
            stage=stage,
            planePath=ground_path,
            axis="Z",
            size=50.0,
            position=Gf.Vec3f(0, 0, 0),
            color=Gf.Vec3f(0.5, 0.5, 0.5),
        )
        if stage.GetPrimAtPath(ground_path).IsValid():
            print(f"Created ground plane at {ground_path} (kit command)")
            return
    except Exception as e:
        print(f"  AddGroundPlaneCommand failed: {e}")

    # Method 3: Manual USD ground plane
    print("  Creating manual ground plane...")
    xform = UsdGeom.Xform.Define(stage, ground_path)
    cube_path = ground_path + "/CollisionMesh"
    cube = UsdGeom.Cube.Define(stage, cube_path)
    cube.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(cube).SetScale(Gf.Vec3f(100.0, 100.0, 0.01))
    UsdGeom.XformCommonAPI(cube).SetTranslate(Gf.Vec3d(0, 0, -0.005))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    print(f"Created ground plane at {ground_path} (manual cube)")


def find_articulation_root(robot_prim_path):
    """Find the prim with ArticulationRootAPI under robot_prim_path."""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(robot_prim_path)

    if not prim.IsValid():
        print(f"Error: Robot prim not found at {robot_prim_path}")
        return None

    # Check root prim itself
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        print(f"ArticulationRootAPI on {robot_prim_path}")
        return robot_prim_path

    # Search children for existing articulation root
    for child in Usd.PrimRange(prim):
        if child.HasAPI(UsdPhysics.ArticulationRootAPI):
            path = str(child.GetPath())
            print(f"Found ArticulationRootAPI on child: {path}")
            return path

    # Not found - apply to root prim
    UsdPhysics.ArticulationRootAPI.Apply(prim)
    print(f"Applied ArticulationRootAPI to {robot_prim_path}")
    return robot_prim_path


def set_prim_target(graph_path, node_name, attr_name, target_path):
    """Set a USD relationship (target) attribute on an OmniGraph node."""
    stage = omni.usd.get_context().get_stage()
    node_prim = stage.GetPrimAtPath(f"{graph_path}/{node_name}")
    if not node_prim.IsValid():
        print(f"  Warning: Node not found: {graph_path}/{node_name}")
        return False

    rel = node_prim.GetRelationship(attr_name)
    if not rel:
        rel = node_prim.CreateRelationship(attr_name)
    rel.SetTargets([Sdf.Path(target_path)])
    print(f"  {node_name}.{attr_name} -> {target_path}")
    return True


def diagnose_robot(robot_prim_path):
    """Print diagnostic info about the robot prim."""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(robot_prim_path)
    if not prim.IsValid():
        print(f"Robot prim NOT FOUND at {robot_prim_path}")
        return

    print(f"\n--- Robot Diagnostics: {robot_prim_path} ---")
    print(f"  Type: {prim.GetTypeName()}")
    print(f"  Has ArticulationRootAPI: {prim.HasAPI(UsdPhysics.ArticulationRootAPI)}")
    print(f"  Has RigidBodyAPI: {prim.HasAPI(UsdPhysics.RigidBodyAPI)}")

    # Check children
    art_count = 0
    rb_count = 0
    joint_count = 0
    for child in Usd.PrimRange(prim):
        if child.HasAPI(UsdPhysics.ArticulationRootAPI):
            art_count += 1
        if child.HasAPI(UsdPhysics.RigidBodyAPI):
            rb_count += 1
        if child.IsA(UsdPhysics.Joint):
            joint_count += 1

    print(f"  Child prims with ArticulationRootAPI: {art_count}")
    print(f"  Child prims with RigidBodyAPI: {rb_count}")
    print(f"  Physics joints: {joint_count}")
    print(f"  Direct children: {[str(c.GetPath().name) for c in prim.GetChildren()]}")
    print("---")


def setup_ros2_bridge(robot_prim_path="/World/raptorlift"):
    """
    Set up OmniGraph for ROS 2 joint control in Isaac Sim 5.1.

    Execution flow (parallel from OnPlaybackTick):
      OnPlaybackTick -> SubscribeJointState -> ArticulationController
      OnPlaybackTick -> PublishJointState
      OnPlaybackTick -> PublishClock
    """
    if not ISAAC_SIM_AVAILABLE:
        print("Error: Must run inside Isaac Sim")
        return False

    # 1. Physics environment
    ensure_physics_scene()
    ensure_ground_plane()

    # 2. Articulation check + diagnostics
    art_path = find_articulation_root(robot_prim_path)
    diagnose_robot(robot_prim_path)

    if not art_path:
        print("ERROR: No articulation root found. Re-import URDF with 'Create Articulation' checked.")
        return False

    # 3. OmniGraph
    graph_path = "/World/RaptorLiftActionGraph"
    print(f"\nRobot prim: {robot_prim_path}")
    print(f"Articulation root: {art_path}")

    stage = omni.usd.get_context().get_stage()

    # Delete existing graph
    existing_prim = stage.GetPrimAtPath(graph_path)
    if existing_prim.IsValid():
        stage.RemovePrim(graph_path)
        print(f"Removed existing graph at {graph_path}")

    create_nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
        ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
    ]

    connections = [
        ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
        ("SubscribeJointState.outputs:execOut", "ArticulationController.inputs:execIn"),
        ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
        ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
        ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
        ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
    ]

    set_values = [
        ("SubscribeJointState.inputs:topicName", "/raptorlift/joint_commands"),
        ("PublishJointState.inputs:topicName", "/raptorlift/joint_states"),
    ]

    try:
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: create_nodes,
                keys.CONNECT: connections,
                keys.SET_VALUES: set_values,
            },
        )

        # Set target prim via USD relationship API
        # Must point to the prim WITH ArticulationRootAPI (not parent Xform)
        print("Setting target prim relationships:")
        set_prim_target(graph_path, "ArticulationController", "inputs:targetPrim", art_path)
        set_prim_target(graph_path, "PublishJointState", "inputs:targetPrim", art_path)

        print(f"\n=== OmniGraph Created Successfully ===")
        print(f"  Graph:     {graph_path}")
        print(f"  Subscribe: /raptorlift/joint_commands")
        print(f"  Publish:   /raptorlift/joint_states, /clock")
        print(f"  Articulation: {art_path}")
        print(f"  Ground plane: /World/GroundPlane")
        print(f"\nRun add_lidar.py next, then press Play.")
        return True

    except Exception as e:
        print(f"Error creating OmniGraph: {e}")
        import traceback
        traceback.print_exc()
        return False


def find_robot_prim():
    """Find the robot prim path on the stage."""
    stage = omni.usd.get_context().get_stage()
    if not stage:
        return "/World/raptorlift"

    test_paths = ["/World/raptorlift", "/raptorlift", "/World/Robot"]
    for path in test_paths:
        if stage.GetPrimAtPath(path).IsValid():
            print(f"Found robot at: {path}")
            return path

    print("Warning: Robot not found at expected paths.")
    print("Available prims at /World:")
    world_prim = stage.GetPrimAtPath("/World")
    if world_prim.IsValid():
        for child in world_prim.GetChildren():
            print(f"  {child.GetPath()}")
    return "/World/raptorlift"


# --- Run ---
if ISAAC_SIM_AVAILABLE:
    setup_ros2_bridge(find_robot_prim())
else:
    print("Run this script inside Isaac Sim Script Editor.")
