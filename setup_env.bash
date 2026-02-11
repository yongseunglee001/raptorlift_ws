#!/bin/bash
# Isaac Sim + ROS 2 Jazzy Navigation Environment Setup
# Source this before launching Isaac Sim or Nav2

# Ensure system Python is first (avoid uv Python conflicts)
export PATH="/usr/bin:$PATH"

# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Isaac Sim paths (source build)
export ISAAC_SIM_PATH="$HOME/isaacsim"
export ISAAC_SIM_BUILD="$ISAAC_SIM_PATH/_build/linux-x86_64/release"

# ROS 2 configuration
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Isaac Sim ROS 2 Bridge libraries
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ISAAC_SIM_BUILD/exts/isaacsim.ros2.bridge/jazzy/lib

# Source workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/install/local_setup.bash" ]; then
    source "$SCRIPT_DIR/install/local_setup.bash"
fi

# Aliases
alias cdws='cd /home/yongseung-b850-yamato/RaptorYamato/IsaacSim-ros_workspaces/jazzy_ws'
alias bringup='ros2 launch raptorlift_bringup raptorlift.launch.py'
alias slam='ros2 launch raptor_navigation slam.launch.py'
alias nav='ros2 launch raptor_navigation navigation.launch.py'
alias keyteleop='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=base_link -r cmd_vel:=/key/cmd_vel'

echo ""
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║        RaptorLift + Isaac Sim + ROS 2 Jazzy Ready           ║"
echo "║  ROS: $ROS_DISTRO  |  RMW: $RMW_IMPLEMENTATION  |  Python: $(python3 --version 2>&1 | cut -d' ' -f2)"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo "  ── Main Launch (2단계 실행) ──────────────────────────────────"
echo "  [1] bringup    ros2_control + twist_mux + teleop + LiDAR + rviz"
echo "                 opts: simulation_mode  use_hardware_bridge  use_teleop  use_rviz  use_lidars"
echo "  [2] slam       slam_toolbox + Nav2 planner/controller + LiDAR processing"
echo "                 opts: use_sim_time  use_rviz"
echo "  ── Modules ───────────────────────────────────────────────────"
echo "  [3] nav        Nav2 + AMCL localization (map:=/path/to/map.yaml)"
echo "  [4] keyteleop  Keyboard teleop (/key/cmd_vel, 별도 터미널)"
echo "  [5]            ros2 launch raptor_navigation lidar_processing.launch.py"
echo "  ── Shortcuts ─────────────────────────────────────────────────"
echo "  cdws           Workspace  |  isaacsim  Isaac Sim"
echo ""
