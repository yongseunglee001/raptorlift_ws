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

echo ""
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║        RaptorLift + Isaac Sim + ROS 2 Jazzy Ready           ║"
echo "║  ROS: $ROS_DISTRO  |  RMW: $RMW_IMPLEMENTATION  |  Python: $(python3 --version 2>&1 | cut -d' ' -f2)"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""
echo "  [0] Workspace    cdws"
echo "  [1] Isaac Sim    isaacsim"
echo ""
echo "  [2] Bringup (teleop + twist_mux + ros2_control + rviz)"
echo "      ros2 launch raptorlift_bringup raptorlift.launch.py"
echo "        simulation_mode:=false  use_hardware_bridge:=false  use_teleop:=false  use_rviz:=false"
echo ""
echo "  [3] Keyboard teleop (별도 터미널)"
echo "      ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/key/cmd_vel"
echo ""
echo "  [4] SLAM (매핑 + 내비게이션)"
echo "      ros2 launch raptor_navigation bringup_with_nav.launch.py slam:=True"
echo ""
echo "  [5] Nav2 (로컬라이제이션 + 내비게이션, 맵 필요)"
echo "      ros2 launch raptor_navigation bringup_with_nav.launch.py map:=/path/to/map.yaml"
echo ""
