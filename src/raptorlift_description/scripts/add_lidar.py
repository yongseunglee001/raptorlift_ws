#!/usr/bin/env python3
"""
Add HESAI XT32 RTX Lidar to RaptorLift and publish /scan via ROS 2.

Uses the official Isaac Sim 5.1 Replicator Writer API:
  1. IsaacSensorCreateRtxLidar  -> OmniLidar prim
  2. rep.create.render_product  -> HydraTexture
  3. rep.writers.get("RtxLidarROS2PublishLaserScan") -> /scan

Run in Isaac Sim Script Editor AFTER setup_isaac_sim.py.
"""

import omni
import omni.kit.commands
import omni.replicator.core as rep
from pxr import Gf

ROBOT_PATH = "/World/raptorlift"
LIDAR_PARENT = ROBOT_PATH + "/lidar_link"
LIDAR_PATH = LIDAR_PARENT + "/rtx_lidar"


def add_rtx_lidar():
    """Create HESAI XT32 SD10 RTX Lidar under lidar_link."""
    stage = omni.usd.get_context().get_stage()

    if not stage.GetPrimAtPath(LIDAR_PARENT).IsValid():
        print(f"Error: {LIDAR_PARENT} not found. Import URDF first.")
        return None

    if stage.GetPrimAtPath(LIDAR_PATH).IsValid():
        stage.RemovePrim(LIDAR_PATH)
        print(f"Removed existing lidar at {LIDAR_PATH}")

    configs_to_try = [
        "HESAI_XT32_SD10",
        "XT32_SD10",
        "Example_Rotary",
    ]

    for config_name in configs_to_try:
        _, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="rtx_lidar",
            parent=LIDAR_PARENT,
            config=config_name,
            translation=Gf.Vec3d(0.0, 0.0, 0.0),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        )

        if stage.GetPrimAtPath(LIDAR_PATH).IsValid():
            prim = stage.GetPrimAtPath(LIDAR_PATH)
            print(f"RTX Lidar created: {LIDAR_PATH}")
            print(f"  Config: {config_name}")
            print(f"  Prim type: {prim.GetTypeName()}")
            return sensor

        print(f"  Config '{config_name}' failed, trying next...")

    print("All configs failed. Check Nucleus connection / sensor library.")
    return None


def setup_ros2_scan(sensor):
    """Attach Replicator writer to publish /scan (LaserScan) via ROS 2."""

    # 1. Create render product from the lidar sensor
    sensor_path = sensor.GetPath() if hasattr(sensor, 'GetPath') else LIDAR_PATH
    print(f"Creating render product for: {sensor_path}")

    render_product = rep.create.render_product(
        str(sensor_path), [1, 1], name="IsaacLidar"
    )
    print(f"  Render product: {render_product}")

    # 2. Create LaserScan writer and attach
    writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
    writer.initialize(topicName="/scan", frameId="lidar_link")
    writer.attach([render_product])

    print(f"\n=== RTX Lidar ROS 2 Setup Complete ===")
    print(f"  Topic: /scan (sensor_msgs/LaserScan)")
    print(f"  Frame: lidar_link")
    print(f"\nPress Play, then run [4] SLAM in another terminal.")


# --- Run ---
sensor = add_rtx_lidar()
if sensor is not None:
    setup_ros2_scan(sensor)
