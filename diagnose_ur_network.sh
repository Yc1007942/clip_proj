#!/bin/bash
# UR Robot Network Diagnostic Tool

echo "=== UR Robot Network Diagnostic ==="
echo

# Check ethernet interfaces
echo "📡 Available Network Interfaces:"
for iface in enxd8ec5e11d59f enp4s0; do
    if ip link show "$iface" &>/dev/null; then
        state=$(cat /sys/class/net/$iface/operstate 2>/dev/null || echo "unknown")
        echo "  $iface: $state"
        ip addr show "$iface" | grep -E "inet " | sed 's/^/    /'
    fi
done
echo

# Current active connections
echo "🔌 Active Network Connections:"
nmcli connection show --active | head -n 5
echo

# Test common robot IPs on all interfaces
echo "🤖 Scanning for UR Robot..."
test_ips=("192.168.56.101" "192.168.1.102" "192.168.0.2" "192.168.3.101" "192.168.3.102")

for ip in "${test_ips[@]}"; do
    echo -n "  Testing $ip... "
    if timeout 2 ping -c 1 "$ip" &>/dev/null; then
        echo "✅ RESPONDS"
        
        # Test robot dashboard port
        if timeout 2 nc -zv "$ip" 29999 2>&1 | grep -q succeeded; then
            echo "    🎯 UR Dashboard port 29999 accessible - This is likely your robot!"
        fi
        
        # Test other UR ports
        for port in 30001 30002 30003; do
            if timeout 1 nc -zv "$ip" "$port" 2>&1 | grep -q succeeded; then
                echo "    ✅ Port $port accessible"
            fi
        done
    else
        echo "❌ No response"
    fi
done

echo
echo "💡 Next Steps:"
echo "If you found a responding IP above, use:"
echo "  sudo ur-connect.sh 56   # if robot is at 192.168.56.101"
echo "  sudo ur-connect.sh 1    # if robot is at 192.168.1.102" 
echo "  ./launch_ur_robot.sh    # after successful connection"
echo
echo "If no robot found:"
echo "1. Check ethernet cable connection"
echo "2. Verify robot is powered on"
echo "3. Check robot IP settings on teach pendant:"
echo "   Settings → System → Network"
