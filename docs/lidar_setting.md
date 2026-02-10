# RaptorLift LiDAR Setting

## Network Topology

```
Ethernet Switch (192.168.1.x/24)
  |
  +-- PC (host)         192.168.1.102
  +-- Hesai XT32         192.168.1.201
  +-- RoboSense RSE1     192.168.1.200  (Left + Right, different ports)
```

## LiDAR Overview

| LiDAR | Model | IP | URDF Link | ROS Topic | frame_id |
|-------|-------|----|-----------|-----------|----------|
| Main  | Hesai XT32 | 192.168.1.201 | `lidar_link` | `/lidar_points` | `lidar_link` |
| Left  | RoboSense RSE1 | 192.168.1.200 | `lidar_rslidar_left_link` | `/rslidar_left/points` | `lidar_rslidar_left_link` |
| Right | RoboSense RSE1 | 192.168.1.200 | `lidar_rslidar_right_link` | `/rslidar_right/points` | `lidar_rslidar_right_link` |

## Port Assignment

| LiDAR | MSOP Port | DIFOP Port |
|-------|-----------|------------|
| Hesai XT32 | 2368 | - |
| RSE1 Left  | 6699 | 7788 |
| RSE1 Right | 6698 | 7789 |

## URDF

3 LiDAR links defined in `src/raptorlift_description/urdf/raptorlift.urdf`:

```
base_link
  +-- lidar_link                  (Main, Hesai XT32)  xyz="0.339 0 0.55"
  +-- lidar_rslidar_left_link     (RSE1 Left)         xyz="0.0 +0.35 0.55"
  +-- lidar_rslidar_right_link    (RSE1 Right)        xyz="0.0 -0.35 0.55"
```

## Config Files

### Hesai XT32

- Path: `src/HesaiLidar_ROS_2.0/config/config.yaml`
- Key settings:
  - `device_ip_address: 192.168.1.201`
  - `udp_port: 2368`
  - `ros_frame_id: lidar_link`
  - `ros_send_point_cloud_topic: /lidar_points`

### RoboSense RSE1 (Dual)

- Path: `src/rslidar_sdk/config/config.yaml`
- Single node handles both LiDARs via `lidar:` array
- Key settings per LiDAR:

| Setting | Left | Right |
|---------|------|-------|
| `lidar_type` | RSE1 | RSE1 |
| `msop_port` | 6699 | 6698 |
| `difop_port` | 7788 | 7789 |
| `host_address` | 192.168.1.102 | 192.168.1.102 |
| `ros_frame_id` | lidar_rslidar_left_link | lidar_rslidar_right_link |
| `ros_send_point_cloud_topic` | /rslidar_left/points | /rslidar_right/points |

## Launch Order

```bash
# 1. Bringup (TF + controllers + RViz)
ros2 launch raptorlift_bringup raptorlift.launch.py

# 2. Hesai XT32
ros2 launch hesai_ros_driver start.py

# 3. RoboSense RSE1 (both Left + Right in one node)
ros2 launch rslidar_sdk start.py
```

## Host PC Network Setup

The host PC Ethernet interface must have IP `192.168.1.102` to receive LiDAR packets:

```bash
# Check current IP
ip addr show enp8s0

# Set IP if needed
sudo ip addr add 192.168.1.102/24 dev enp8s0
```

## Troubleshooting

| Error | Cause | Fix |
|-------|-------|-----|
| `ERRCODE_MSOPTIMEOUT` | No packets arriving | Check PC IP matches LiDAR destination (192.168.1.102), check cable/switch |
| `ERRCODE_WRONGMSOPLEN` | Wrong `lidar_type` in config | Set correct model (RSE1) in config.yaml |
| `Could not transform from [frame] to [frame]` | TF tree missing | Ensure `raptorlift_bringup` is running (publishes URDF TF) |
| Right LiDAR no data | 2nd RSE1 not sending on port 6698 | Configure 2nd RSE1 via its web UI to use MSOP=6698, DIFOP=7789 |

## Adding/Removing RSE1 Right LiDAR

To comment out Right LiDAR when only 1 RSE1 is available:

1. `src/rslidar_sdk/config/config.yaml` - Comment the `# ===== LiDAR 2 (Right)` block
2. URDF - No change needed (unused link is harmless)

To enable Right LiDAR:

1. Configure 2nd RSE1 hardware: MSOP port=6698, DIFOP port=7789
2. Uncomment the Right block in `config.yaml`

## References

- [rslidar_sdk Multi-LiDAR Config](https://github.com/RoboSense-LiDAR/rslidar_sdk/blob/main/doc/howto/07_online_lidar_advanced_topics.md#731-different-remote-ports)
- [rslidar_sdk GitHub](https://github.com/RoboSense-LiDAR/rslidar_sdk)
- [HesaiLidar_ROS_2.0 GitHub](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0)
