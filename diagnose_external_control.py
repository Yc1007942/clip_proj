#!/usr/bin/env python3
"""
UR5e External Control Diagnostic Script
This script shows exactly what's blocking robot movement
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class ExternalControlDiagnostic(Node):
    def __init__(self):
        super().__init__('external_control_diagnostic')
        
        # Monitor robot program running status
        self.program_running_sub = self.create_subscription(
            Bool,
            '/io_and_status_controller/robot_program_running',
            self.program_running_callback,
            10
        )
        
        # Monitor robot mode
        self.robot_mode_sub = self.create_subscription(
            self.get_int32_msg_type(),
            '/io_and_status_controller/robot_mode',
            self.robot_mode_callback,
            10
        )
        
        self.program_running = None
        self.robot_mode = None
        
        print("\n🔍 UR5e External Control Diagnostic")
        print("=" * 50)

    def get_int32_msg_type(self):
        try:
            from ur_dashboard_msgs.msg import RobotMode
            return RobotMode
        except:
            from std_msgs.msg import Int32
            return Int32

    def program_running_callback(self, msg):
        self.program_running = msg.data
        status = "✅ RUNNING" if self.program_running else "❌ NOT RUNNING"
        print(f"External Control Program: {status}")

    def robot_mode_callback(self, msg):
        if hasattr(msg, 'mode'):
            self.robot_mode = msg.mode
        else:
            self.robot_mode = msg.data
            
        mode_names = {
            -1: "DISCONNECTED",
            0: "CONFIRM_SAFETY", 
            1: "BOOTING",
            2: "POWER_OFF",
            3: "POWER_ON", 
            4: "IDLE",
            5: "BACKDRIVE",
            6: "RUNNING",
            7: "UPDATING_FIRMWARE"
        }
        
        mode_name = mode_names.get(self.robot_mode, f"UNKNOWN({self.robot_mode})")
        print(f"Robot Mode: {mode_name}")

def main():
    rclpy.init()
    diagnostic = ExternalControlDiagnostic()
    
    print("Monitoring robot status for 10 seconds...")
    print("Expected for robot movement:")
    print("  ✅ External Control Program: RUNNING")
    print("  ✅ Robot Mode: RUNNING") 
    print("\nCurrent status:")
    
    start_time = time.time()
    while time.time() - start_time < 10:
        rclpy.spin_once(diagnostic, timeout_sec=0.1)
        
        if diagnostic.program_running is not None and diagnostic.robot_mode is not None:
            break
    
    print("\n" + "=" * 50)
    print("📋 DIAGNOSIS RESULT:")
    
    if diagnostic.program_running is None:
        print("⚠️  Cannot determine External Control status")
        print("   This usually means the robot driver isn't fully connected")
    elif not diagnostic.program_running:
        print("❌ PROBLEM IDENTIFIED: External Control program is NOT running")
        print("\n🔧 SOLUTION:")
        print("   1. On UR5e teach pendant, go to Program tab")
        print("   2. Find/Create External Control program")
        print("   3. Configure with:")
        print("      - Host IP: 192.168.56.101")  
        print("      - Port: 50002")
        print("   4. Press PLAY ▶️ to start program")
        print("\n💡 The robot WILL NOT move until External Control is running!")
    else:
        print("✅ External Control program is running")
        print("   Robot should be able to execute movement commands")
    
    print("\nExact network configuration needed:")
    print("  Robot IP: 192.168.56.100 (current)")
    print("  Computer IP: 192.168.56.101 (current)")  
    print("  External Control Host: 192.168.56.101 ← CHECK THIS!")
    
    diagnostic.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
