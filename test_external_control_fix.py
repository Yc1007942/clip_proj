#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ur_dashboard_msgs.msg import RobotMode
import time

class ExternalControlVerifier(Node):
    def __init__(self):
        super().__init__('external_control_verifier')
        
        # Subscribe to External Control status
        self.program_subscription = self.create_subscription(
            Bool,
            '/io_and_status_controller/robot_program_running',
            self.program_callback,
            10)
            
        # Subscribe to robot mode
        self.mode_subscription = self.create_subscription(
            RobotMode,
            '/io_and_status_controller/robot_mode',
            self.mode_callback,
            10)
            
        self.program_running = None
        self.robot_mode = None
        
        print("🔍 Verifying External Control fix...")
        print("After configuring External Control on teach pendant:")
        print("  1. Set Host IP: 192.168.56.101")
        print("  2. Set Port: 50002") 
        print("  3. Start the External Control program")
        print("\nMonitoring status...")
        
    def program_callback(self, msg):
        if self.program_running != msg.data:
            self.program_running = msg.data
            status = "✅ RUNNING" if msg.data else "❌ STOPPED"
            print(f"External Control Program: {status}")
            
            if msg.data:
                print("🎉 SUCCESS! External Control is now active!")
                print("You can now control the robot with rotate_gripper.py")
            
    def mode_callback(self, msg):
        mode_names = {
            -1: "ROBOT_MODE_NO_CONTROLLER",
            0: "ROBOT_MODE_DISCONNECTED", 
            1: "ROBOT_MODE_CONFIRM_SAFETY",
            2: "ROBOT_MODE_BOOTING",
            3: "ROBOT_MODE_POWER_OFF",
            4: "ROBOT_MODE_POWER_ON",
            5: "ROBOT_MODE_IDLE", 
            6: "ROBOT_MODE_BACKDRIVE",
            7: "ROBOT_MODE_RUNNING",
            8: "ROBOT_MODE_UPDATING_FIRMWARE"
        }
        
        if self.robot_mode != msg.mode:
            self.robot_mode = msg.mode
            mode_name = mode_names.get(msg.mode, f"UNKNOWN_MODE_{msg.mode}")
            print(f"Robot Mode: {mode_name} ({msg.mode})")

def main():
    rclpy.init()
    verifier = ExternalControlVerifier()
    
    try:
        # Monitor status - press Ctrl+C to stop
        rclpy.spin(verifier)
    except KeyboardInterrupt:
        print("\n✋ Monitoring stopped.")
        print("Configure External Control on teach pendant, then run rotate_gripper.py to test!")
    finally:
        verifier.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
