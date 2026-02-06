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
echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║          RaptorLift + Isaac Sim + ROS 2 Jazzy Ready            ║"
echo "╠══════════════════════════════════════════════════════════════════╣"
echo "║  ROS: $ROS_DISTRO  |  RMW: $RMW_IMPLEMENTATION  |  Python: $(python3 --version 2>&1 | cut -d' ' -f2)"
echo "╠══════════════════════════════════════════════════════════════════╣"
echo "║  [0] Workspace       cdws                                       ║"
echo "╠══════════════════════════════════════════════════════════════════╣"
echo "║  [1] Isaac Sim       isaacsim                                  ║"
echo "╠══════════════════════════════════════════════════════════════════╣"
echo "║  [2] Bringup (teleop + twist_mux + ros2_control + rviz)       ║"
echo "║      ros2 launch raptorlift_bringup raptorlift.launch.py       ║"
echo "║                                                                ║"
echo "║      Options:                                                  ║"
echo "║        simulation_mode:=false    # real Modbus servos          ║"
echo "║        use_hardware_bridge:=false # Isaac Sim direct control   ║"
echo "║        use_teleop:=false          # disable gamepad            ║"
echo "║        use_rviz:=false            # disable rviz               ║"
echo "╠══════════════════════════════════════════════════════════════════╣"
echo "║  [3] Keyboard teleop (separate terminal)                      ║"
echo "║      ros2 run teleop_twist_keyboard teleop_twist_keyboard \\    ║"
echo "║        --ros-args -p stamped:=true -r cmd_vel:=/key/cmd_vel    ║"
echo "╚══════════════════════════════════════════════════════════════════╝"
