#!/bin/bash
# Simplified UR5e Robot Network Setup
# This creates a permanent configuration for common UR robot network scenarios

set -e

echo "=== UR5e Robot Network Setup (Manual Configuration) ==="

# Detect the best ethernet interface
INTERFACE=""
if ip link show enxd8ec5e11d59f &>/dev/null && [[ $(cat /sys/class/net/enxd8ec5e11d59f/operstate 2>/dev/null) == "up" ]]; then
    INTERFACE="enxd8ec5e11d59f"
    echo " Using USB Ethernet adapter: $INTERFACE"
elif ip link show enp4s0 &>/dev/null && [[ $(cat /sys/class/net/enp4s0/operstate 2>/dev/null) == "up" ]]; then
    INTERFACE="enp4s0"
    echo " Using built-in Ethernet: $INTERFACE"
else
    echo " No active ethernet interface found"
    exit 1
fi

# Create NetworkManager profiles for common UR robot configurations
create_ur_profiles() {
    echo "Creating NetworkManager profiles for UR robot..."

    # Configuration 1: 192.168.56.x network (most common for direct connection)
    PROFILE1="/etc/NetworkManager/system-connections/UR5e-Direct-56.nmconnection"
    sudo tee "$PROFILE1" > /dev/null << EOF
[connection]
id=UR5e-Direct-56
uuid=$(uuidgen)
type=ethernet
interface-name=${INTERFACE}
autoconnect=false
autoconnect-priority=90

[ethernet]

[ipv4]
address1=192.168.56.101/24
method=manual
never-default=true

[ipv6]
addr-gen-mode=stable-privacy
method=ignore

[proxy]
EOF

    # Configuration 2: 192.168.1.x network (UR factory default)
    PROFILE2="/etc/NetworkManager/system-connections/UR5e-Factory-1.nmconnection"
    sudo tee "$PROFILE2" > /dev/null << EOF
[connection]
id=UR5e-Factory-1
uuid=$(uuidgen)
type=ethernet
interface-name=${INTERFACE}
autoconnect=false
autoconnect-priority=80

[ethernet]

[ipv4]
address1=192.168.1.100/24
method=manual
never-default=true

[ipv6]
addr-gen-mode=stable-privacy
method=ignore

[proxy]
EOF

    # Configuration 3: DHCP with secondary static (hybrid approach)
    PROFILE3="/etc/NetworkManager/system-connections/UR5e-DHCP-Hybrid.nmconnection"
    sudo tee "$PROFILE3" > /dev/null << EOF
[connection]
id=UR5e-DHCP-Hybrid
uuid=$(uuidgen)
type=ethernet
interface-name=${INTERFACE}
autoconnect=true
autoconnect-priority=70

[ethernet]

[ipv4]
method=auto
never-default=true

[ipv6]
addr-gen-mode=stable-privacy
method=ignore

[proxy]
EOF

    # Set permissions
    sudo chmod 600 "$PROFILE1" "$PROFILE2" "$PROFILE3"
    
    echo " Created 3 UR robot network profiles"
}

# Function to test robot connectivity
test_robot_connection() {
    local profile_name="$1"
    local expected_robot_ip="$2"
    
    echo "Testing $profile_name..."
    
    # Activate the profile
    nmcli connection up "$profile_name" 2>/dev/null || return 1
    sleep 5
    
    # Test connectivity
    if timeout 5 ping -c 2 "$expected_robot_ip" &>/dev/null; then
        echo " Robot found at $expected_robot_ip using $profile_name"
        
        # Test dashboard port
        if timeout 3 nc -zv "$expected_robot_ip" 29999 2>&1 | grep -q succeeded; then
            echo " Robot dashboard port accessible"
            return 0
        fi
    fi
    
    return 1
}

# Create activation script
create_activation_script() {
    SCRIPT_PATH="/usr/local/bin/ur-connect.sh"
    
    sudo tee "$SCRIPT_PATH" > /dev/null << 'EOF'
#!/bin/bash
# UR Robot Connection Script
# Usage: ur-connect.sh [56|1|dhcp|auto]

INTERFACE="enxd8ec5e11d59f"
if ! ip link show "$INTERFACE" &>/dev/null; then
    INTERFACE="enp4s0"
fi

case "${1:-auto}" in
    "56")
        echo "Connecting to UR robot on 192.168.56.x network..."
        nmcli connection up "UR5e-Direct-56"
        sleep 3
        if ping -c 2 192.168.56.101 &>/dev/null; then
            echo " Connected to robot at 192.168.56.101"
            echo "Launch ROS2 driver with: robot_ip:=192.168.56.101"
        else
            echo " Robot not responding at 192.168.56.101"
        fi
        ;;
    "1")
        echo "Connecting to UR robot on 192.168.1.x network..."
        nmcli connection up "UR5e-Factory-1"
        sleep 3
        if ping -c 2 192.168.1.102 &>/dev/null; then
            echo " Connected to robot at 192.168.1.102"
            echo "Launch ROS2 driver with: robot_ip:=192.168.1.102"
        else
            echo " Robot not responding at 192.168.1.102"
        fi
        ;;
    "dhcp")
        echo "Using DHCP connection..."
        nmcli connection up "UR5e-DHCP-Hybrid"
        sleep 5
        echo "Check robot IP on teach pendant and use that IP"
        ;;
    "auto"|*)
        echo "Auto-detecting robot connection..."
        
        # Try 192.168.56.x first
        nmcli connection up "UR5e-Direct-56" 2>/dev/null || true
        sleep 3
        if ping -c 2 192.168.56.101 &>/dev/null; then
            echo " Connected to robot at 192.168.56.101"
            echo "Launch ROS2 driver with: robot_ip:=192.168.56.101"
            exit 0
        fi
        
        # Try 192.168.1.x
        nmcli connection up "UR5e-Factory-1" 2>/dev/null || true
        sleep 3
        if ping -c 2 192.168.1.102 &>/dev/null; then
            echo " Connected to robot at 192.168.1.102"
            echo "Launch ROS2 driver with: robot_ip:=192.168.1.102"
            exit 0
        fi
        
        # Try DHCP
        echo "Trying DHCP connection..."
        nmcli connection up "UR5e-DHCP-Hybrid" 2>/dev/null || true
        sleep 5
        echo "No robot auto-detected. Check robot IP on teach pendant."
        echo "Available connection commands:"
        echo "  ur-connect.sh 56    # For 192.168.56.x network"
        echo "  ur-connect.sh 1     # For 192.168.1.x network" 
        echo "  ur-connect.sh dhcp  # For DHCP network"
        ;;
esac
EOF

    sudo chmod +x "$SCRIPT_PATH"
    echo " Created ur-connect.sh script"
}

# Create ROS2 launcher script
create_ros_launcher() {
    SCRIPT_PATH="/home/$USER/panasonic/clip_proj/launch_ur_robot.sh"
    
    tee "$SCRIPT_PATH" > /dev/null << 'EOF'
#!/bin/bash
# UR5e ROS2 Driver Launcher with Auto Network Detection

# Auto-connect to robot
echo "Setting up robot connection..."
sudo ur-connect.sh auto

# Wait a moment
sleep 3

# Try to detect robot IP
ROBOT_IP=""
for ip in "192.168.56.101" "192.168.1.102"; do
    if ping -c 1 "$ip" &>/dev/null; then
        ROBOT_IP="$ip"
        echo "Detected robot at: $ROBOT_IP"
        break
    fi
done

if [[ -z "$ROBOT_IP" ]]; then
    echo " Robot not detected. Please check:"
    echo "  1. Ethernet cable connection"
    echo "  2. Robot power"
    echo "  3. Robot network settings on teach pendant"
    exit 1
fi

# Launch ROS2 driver
echo "Launching ROS2 driver for robot at $ROBOT_IP..."
cd /home/$USER/panasonic/clip_proj/Universal_Robots_ROS2_Driver
source /opt/ros/humble/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=$ROBOT_IP launch_rviz:=false
EOF

    chmod +x "$SCRIPT_PATH"
    echo " Created ROS2 launcher script: $SCRIPT_PATH"
}

# Main execution
main() {
    create_ur_profiles
    
    # Reload NetworkManager
    sudo systemctl reload NetworkManager
    sleep 2
    
    create_activation_script
    create_ros_launcher
    
    echo ""
    echo " UR5e Robot Network Configuration Complete!"
    echo ""
    echo "Available commands:"
    echo "  sudo ur-connect.sh auto    # Auto-detect robot"
    echo "  sudo ur-connect.sh 56      # Connect to 192.168.56.101"
    echo "  sudo ur-connect.sh 1       # Connect to 192.168.1.102"
    echo "  sudo ur-connect.sh dhcp    # Use DHCP"
    echo ""
    echo "ROS2 Launcher:"
    echo "  ./launch_ur_robot.sh       # Auto-connect and launch ROS2 driver"
    echo ""
    echo "Testing auto-connection now..."
    sudo ur-connect.sh auto
}

main "$@"
