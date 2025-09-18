#!/bin/bash
# UR5e Robot Network Configuration Script
# This script sets up a permanent, reliable connection to the UR5e robot

set -e

echo "=== UR5e Robot Network Configuration ==="

# Function to detect which interface is connected to the robot
detect_robot_interface() {
    echo "Detecting robot connection..."
    
    # Test interfaces for robot connectivity
    INTERFACES=("enxd8ec5e11d59f" "enp4s0")
    ROBOT_INTERFACE=""
    
    for iface in "${INTERFACES[@]}"; do
        if ip link show "$iface" &>/dev/null && [[ $(cat /sys/class/net/$iface/operstate 2>/dev/null) == "up" ]]; then
            echo "Testing interface: $iface"
            
            # Configure test IP
            sudo ip addr flush dev "$iface" 2>/dev/null || true
            sudo ip addr add 192.168.56.100/24 dev "$iface" 2>/dev/null || true
            sleep 2
            
            # Test common UR robot IPs
            for test_ip in "192.168.56.101" "192.168.1.102" "192.168.0.2"; do
                if timeout 3 ping -c 2 "$test_ip" &>/dev/null; then
                    echo "✅ Found robot at $test_ip on interface $iface"
                    ROBOT_INTERFACE="$iface"
                    ROBOT_IP="$test_ip"
                    break 2
                fi
            done
            
            # Test if robot responds on dashboard port
            for test_ip in "192.168.56.101"; do
                if timeout 3 nc -zv "$test_ip" 29999 2>&1 | grep -q succeeded; then
                    echo "✅ Found robot dashboard at $test_ip on interface $iface"
                    ROBOT_INTERFACE="$iface"
                    ROBOT_IP="$test_ip"
                    break 2
                fi
            done
        fi
    done
    
    if [[ -z "$ROBOT_INTERFACE" ]]; then
        echo "❌ Robot not detected. Please check:"
        echo "   1. Ethernet cable is properly connected"
        echo "   2. Robot is powered on"
        echo "   3. Robot network settings on teach pendant"
        exit 1
    fi
}

# Function to create NetworkManager profile
create_network_profile() {
    echo "Creating NetworkManager profile for UR robot..."
    
    PROFILE_NAME="UR5e-Robot-Connection"
    PROFILE_FILE="/etc/NetworkManager/system-connections/${PROFILE_NAME}.nmconnection"
    
    # Determine IP configuration based on detected robot
    if [[ "$ROBOT_IP" == "192.168.56.101" ]]; then
        LOCAL_IP="192.168.56.100"
        SUBNET="24"
    elif [[ "$ROBOT_IP" == "192.168.1.102" ]]; then
        LOCAL_IP="192.168.1.100"
        SUBNET="24"
    else
        LOCAL_IP="192.168.0.100"
        SUBNET="24"
    fi
    
    # Create NetworkManager connection profile
    sudo tee "$PROFILE_FILE" > /dev/null << EOF
[connection]
id=${PROFILE_NAME}
uuid=$(uuidgen)
type=ethernet
interface-name=${ROBOT_INTERFACE}
autoconnect=true
autoconnect-priority=100

[ethernet]
mac-address-blacklist=

[ipv4]
address1=${LOCAL_IP}/${SUBNET}
method=manual
never-default=false

[ipv6]
addr-gen-mode=stable-privacy
method=ignore

[proxy]
EOF
    
    # Set correct permissions
    sudo chmod 600 "$PROFILE_FILE"
    
    echo "✅ Created network profile: $PROFILE_NAME"
    echo "   Interface: $ROBOT_INTERFACE"
    echo "   Local IP: $LOCAL_IP/$SUBNET"
    echo "   Robot IP: $ROBOT_IP"
}

# Function to restart networking
restart_networking() {
    echo "Applying network configuration..."
    
    # Reload NetworkManager
    sudo systemctl reload NetworkManager
    sleep 3
    
    # Restart the connection
    nmcli connection down "UR5e-Robot-Connection" 2>/dev/null || true
    sleep 2
    nmcli connection up "UR5e-Robot-Connection"
    sleep 3
}

# Function to verify connection
verify_connection() {
    echo "Verifying robot connection..."
    
    if ping -c 3 "$ROBOT_IP" &>/dev/null; then
        echo "✅ Robot ping successful: $ROBOT_IP"
    else
        echo "❌ Robot ping failed: $ROBOT_IP"
        return 1
    fi
    
    # Test UR robot ports
    PORTS=(29999 30001 30002 30003 30004 502)
    for port in "${PORTS[@]}"; do
        if timeout 2 nc -zv "$ROBOT_IP" "$port" 2>&1 | grep -q succeeded; then
            echo "✅ Robot port $port accessible"
        fi
    done
}

# Function to create startup script
create_startup_script() {
    STARTUP_SCRIPT="/usr/local/bin/ur-robot-network-check.sh"
    
    sudo tee "$STARTUP_SCRIPT" > /dev/null << 'EOF'
#!/bin/bash
# UR Robot Network Health Check Script
# Runs at startup to ensure robot connection is working

LOG_FILE="/var/log/ur-robot-network.log"

log_message() {
    echo "$(date '+%Y-%m-%d %H:%M:%S'): $1" | sudo tee -a "$LOG_FILE"
}

# Wait for NetworkManager to be ready
sleep 10

# Check if robot connection profile exists
if nmcli connection show "UR5e-Robot-Connection" &>/dev/null; then
    log_message "UR5e robot connection profile found"
    
    # Ensure connection is up
    nmcli connection up "UR5e-Robot-Connection" &>/dev/null || true
    sleep 5
    
    # Test connectivity
    if ping -c 2 192.168.56.101 &>/dev/null || ping -c 2 192.168.1.102 &>/dev/null; then
        log_message "UR5e robot connectivity verified"
    else
        log_message "WARNING: UR5e robot not responding, attempting reconnection"
        nmcli connection down "UR5e-Robot-Connection" &>/dev/null || true
        sleep 3
        nmcli connection up "UR5e-Robot-Connection" &>/dev/null || true
    fi
else
    log_message "WARNING: UR5e robot connection profile not found"
fi
EOF
    
    sudo chmod +x "$STARTUP_SCRIPT"
    
    # Create systemd service
    sudo tee "/etc/systemd/system/ur-robot-network.service" > /dev/null << EOF
[Unit]
Description=UR Robot Network Health Check
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=oneshot
ExecStart=${STARTUP_SCRIPT}
User=root

[Install]
WantedBy=multi-user.target
EOF
    
    sudo systemctl daemon-reload
    sudo systemctl enable ur-robot-network.service
    
    echo "✅ Created startup health check service"
}

# Main execution
main() {
    echo "Starting UR5e robot network configuration..."
    
    # Check if running as root or with sudo
    if [[ $EUID -ne 0 ]] && [[ -z "$SUDO_USER" ]]; then
        echo "This script needs sudo privileges. Please run with sudo."
        exit 1
    fi
    
    detect_robot_interface
    create_network_profile
    restart_networking
    verify_connection
    create_startup_script
    
    echo ""
    echo "🎉 UR5e Robot Network Configuration Complete!"
    echo ""
    echo "Summary:"
    echo "  ✅ Robot detected at: $ROBOT_IP"
    echo "  ✅ Using interface: $ROBOT_INTERFACE"
    echo "  ✅ NetworkManager profile created"
    echo "  ✅ Startup health check enabled"
    echo ""
    echo "The robot connection will now persist across reboots."
    echo "You can now launch the ROS2 driver with: robot_ip:=$ROBOT_IP"
}

# Run main function
main "$@"
