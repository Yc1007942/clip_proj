#!/bin/bash
# UR5e ROS2 Driver Launcher with Auto Network Detection

# Auto-connect to robot
echo "Setting up robot connection..."
sudo ur-connect.sh auto

# Wait a moment
sleep 3

# Try to detect robot IP
ROBOT_IP=""
for ip in "192.168.56.100" "192.168.1.102"; do
    if ping -c 1 "$ip" &>/dev/null; then
        ROBOT_IP="$ip"
        echo "Detected robot at: $ROBOT_IP"
        break
    fi
done

if [[ -z "$ROBOT_IP" ]]; then
    echo "❌ Robot not detected. Please check:"
    echo "  1. Ethernet cable connection"
    echo "  2. Robot power"
    echo "  3. Robot network settings on teach pendant"
    exit 1
fi

# Launch ROS2 driver
echo "Launching ROS2 driver for robot at $ROBOT_IP..."
cd /home/yc/panasonic/clip_proj/Universal_Robots_ROS2_Driver
source /opt/ros/humble/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=$ROBOT_IP launch_rviz:=false
