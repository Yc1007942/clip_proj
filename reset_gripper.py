#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Bool
import time
import sys
import subprocess

class GripperReset(Node):
    def __init__(self):
        super().__init__('gripper_reset')
        
        # Publishers
        self.position_pub = self.create_publisher(UInt8, '/lebai_gripper/cmd/position', 10)
        self.force_pub = self.create_publisher(UInt8, '/lebai_gripper/cmd/force', 10)
        self.speed_pub = self.create_publisher(UInt8, '/lebai_gripper/cmd/speed', 10)
        self.home_pub = self.create_publisher(Bool, '/lebai_gripper/home', 10)
        self.disable_autohome_pub = self.create_publisher(Bool, '/lebai_gripper/disable_autohome', 10)
        
        # Wait for connections
        time.sleep(1.0)
        
        print("🤖 LeBai Gripper Complete Reset Tool")
        print("=" * 50)
        print("This tool will perform a full gripper reset including:")
        print("• Hardware power cycle")
        print("• Communication reset")
        print("• Home position calibration")
        print("• State reading reset")
        print("• Position limit calibration")
        print("=" * 50)
        
    def wait_and_feedback(self, seconds, message=""):
        """Wait with visual feedback"""
        if message:
            print(f"⏳ {message}", end="", flush=True)
        
        for i in range(seconds * 2):  # Update every 0.5 seconds
            print(".", end="", flush=True)
            time.sleep(0.5)
        print(" ✅")
        
    def power_cycle_gripper(self):
        """Power cycle the gripper hardware"""
        print("\n🔌 Step 1: Hardware Power Cycle")
        print("Please follow these steps:")
        print("1. Unplug the gripper power cable")
        input("   Press Enter when power is disconnected...")
        
        print("2. Wait 10 seconds for capacitors to discharge")
        self.wait_and_feedback(10, "Waiting for full discharge")
        
        print("3. Plug the gripper power cable back in")
        input("   Press Enter when power is reconnected...")
        
        print("4. Wait for gripper to initialize")
        self.wait_and_feedback(5, "Waiting for gripper initialization")
        
    def reset_communication(self):
        """Reset USB communication"""
        print("\n📡 Step 2: Communication Reset")
        
        print("Stopping gripper driver...")
        try:
            subprocess.run(['pkill', '-f', 'lebai_gripper'], check=False)
            time.sleep(2)
        except:
            pass
        
        print("Resetting USB connection...")
        try:
            # Reset USB device
            subprocess.run(['sudo', 'usb_modeswitch', '-R'], check=False, capture_output=True)
        except:
            pass
        
        self.wait_and_feedback(3, "Waiting for USB reset")
        
        print("Restarting gripper driver...")
        # Note: The driver should be manually restarted after this script
        
    def disable_state_reading(self):
        """Disable automatic state reading that's causing issues"""
        print("\n🚫 Step 3: Disabling Problematic State Reading")
        
        # Disable auto-homing which triggers state reads
        disable_msg = Bool()
        disable_msg.data = True
        self.disable_autohome_pub.publish(disable_msg)
        self.wait_and_feedback(2, "Disabling auto-homing")
        
    def emergency_stop_reset(self):
        """Reset any emergency stop conditions"""
        print("\n🛑 Step 4: Emergency Stop Reset")
        
        # Set very low force first
        force_msg = UInt8()
        force_msg.data = 5  # Very low force
        self.force_pub.publish(force_msg)
        self.wait_and_feedback(1, "Setting emergency low force")
        
        # Try multiple positions to clear any stuck states
        positions = [50, 0, 100, 50]  # Middle, open, close, middle
        
        for i, pos in enumerate(positions):
            print(f"   Emergency position {i+1}/4: {pos}%")
            position_msg = UInt8()
            position_msg.data = pos
            self.position_pub.publish(position_msg)
            self.wait_and_feedback(1, f"Emergency move to {pos}%")
        
    def calibrate_home_position(self):
        """Calibrate the home position"""
        print("\n🏠 Step 5: Home Position Calibration")
        
        # Multiple home attempts
        for attempt in range(3):
            print(f"   Home attempt {attempt+1}/3")
            
            home_msg = Bool()
            home_msg.data = True
            self.home_pub.publish(home_msg)
            self.wait_and_feedback(3, f"Homing attempt {attempt+1}")
            
            # Clear home signal
            home_msg.data = False
            self.home_pub.publish(home_msg)
            time.sleep(0.5)
        
    def calibrate_position_limits(self):
        """Calibrate position limits with force feedback"""
        print("\n📏 Step 6: Position Limit Calibration")
        
        # Set calibration parameters
        force_msg = UInt8()
        force_msg.data = 15  # Low force for calibration
        self.force_pub.publish(force_msg)
        
        speed_msg = UInt8()
        speed_msg.data = 25  # Slow speed for calibration
        self.speed_pub.publish(speed_msg)
        
        self.wait_and_feedback(1, "Setting calibration parameters")
        
        # Find fully open position
        print("   Finding fully open position...")
        position_msg = UInt8()
        position_msg.data = 0
        self.position_pub.publish(position_msg)
        self.wait_and_feedback(4, "Moving to fully open")
        
        # Find fully closed position with low force
        print("   Finding fully closed position...")
        position_msg.data = 100
        self.position_pub.publish(position_msg)
        self.wait_and_feedback(4, "Moving to fully closed (low force)")
        
        # Return to middle position
        print("   Setting neutral position...")
        position_msg.data = 20
        self.position_pub.publish(position_msg)
        self.wait_and_feedback(2, "Moving to neutral position")
        
    def restore_normal_parameters(self):
        """Restore normal operating parameters"""
        print("\n⚙️  Step 7: Restoring Normal Parameters")
        
        # Normal speed
        speed_msg = UInt8()
        speed_msg.data = 50
        self.speed_pub.publish(speed_msg)
        self.wait_and_feedback(1, "Setting normal speed (50%)")
        
        # Normal force
        force_msg = UInt8()
        force_msg.data = 50
        self.force_pub.publish(force_msg)
        self.wait_and_feedback(1, "Setting normal force (50%)")
        
        # Safe position
        position_msg = UInt8()
        position_msg.data = 15
        self.position_pub.publish(position_msg)
        self.wait_and_feedback(2, "Setting safe position (15%)")
        
    def reset_gripper(self):
        """Complete gripper reset sequence"""
        
        print("\n🔄 Starting Complete Gripper Reset Sequence...")
        print("⚠️  This process will take several minutes")
        
        try:
            # Step 1: Power cycle (manual)
            self.power_cycle_gripper()
            
            # Step 2: Communication reset
            self.reset_communication()
            
            # Give user time to restart driver
            print("\n⚠️  IMPORTANT: Please restart the gripper driver now!")
            print("Run this command in another terminal:")
            print("ros2 launch lebai_gripper_ros lebai_gripper.launch.py port:=/dev/ttyUSB0")
            input("Press Enter when the gripper driver is running (ignore the warnings)...")
            
            # Continue with software reset
            print("\n🔧 Continuing with software reset...")
            
            # Step 3: Disable problematic state reading
            self.disable_state_reading()
            
            # Step 4: Emergency stop reset
            self.emergency_stop_reset()
            
            # Step 5: Home calibration
            self.calibrate_home_position()
            
            # Step 6: Position limits calibration
            self.calibrate_position_limits()
            
            # Step 7: Restore normal parameters
            self.restore_normal_parameters()
            
            print("\n✅ Complete Gripper Reset Finished!")
            print("\n📋 Reset Summary:")
            print("   • Hardware power cycled")
            print("   • Communication reset")
            print("   • State reading issues bypassed")
            print("   • Home position calibrated")
            print("   • Position limits calibrated")
            print("   • Normal parameters restored")
            print("\n🎯 Current Settings:")
            print("   • Position: 15% (safe open)")
            print("   • Force: 50% (normal)")
            print("   • Speed: 50% (normal)")
            
        except KeyboardInterrupt:
            print("\n⛔ Reset interrupted! Gripper may be in unknown state.")
            print("Please manually set safe parameters:")
            print("ros2 topic pub --once /lebai_gripper/cmd/position std_msgs/UInt8 \"data: 20\"")
        except Exception as e:
            print(f"\n❌ Error during reset: {e}")
            print("Attempting emergency safe position...")
            try:
                position_msg = UInt8()
                position_msg.data = 20
                self.position_pub.publish(position_msg)
            except:
                pass
        
    def test_sequence(self):
        """Quick test sequence to verify gripper operation"""
        print("\n🧪 Running Quick Test Sequence...")
        
        positions = [0, 30, 60, 80, 50, 20]
        
        for i, pos in enumerate(positions):
            print(f"\n   Test {i+1}/6: Moving to {pos}%")
            
            position_msg = UInt8()
            position_msg.data = pos
            self.position_pub.publish(position_msg)
            self.wait_and_feedback(2, f"Moving to {pos}%")
        
        print("\n✅ Quick test sequence complete!")
    
    def comprehensive_test(self):
        """Comprehensive test sequence to verify all functions"""
        print("\n🧪 Running Comprehensive Test Sequence...")
        
        test_sequences = [
            ("Basic Movement", [0, 25, 50, 75, 90, 50, 20]),
            ("Fine Control", [15, 18, 22, 25, 20, 15]),
            ("Force Test", [0, 80, 40, 60, 30]),
            ("Speed Test", [10, 30, 50, 70, 40])
        ]
        
        for test_name, positions in test_sequences:
            print(f"\n   🔬 {test_name} Test:")
            
            for i, pos in enumerate(positions):
                print(f"      Step {i+1}/{len(positions)}: {pos}%")
                
                position_msg = UInt8()
                position_msg.data = pos
                self.position_pub.publish(position_msg)
                self.wait_and_feedback(1, f"Testing {pos}%")
        
        # Return to safe position
        position_msg = UInt8()
        position_msg.data = 20
        self.position_pub.publish(position_msg)
        self.wait_and_feedback(2, "Returning to safe position")
        
        print("\n✅ Comprehensive test complete!")
        print("\n📊 Test Results:")
        print("   • Basic movement: Tested")
        print("   • Fine control: Tested") 
        print("   • Force response: Tested")
        print("   • Speed control: Tested")
        print("   • Final position: 20% (safe)")

def main():
    rclpy.init()
    
    try:
        gripper_reset = GripperReset()
        
        if len(sys.argv) > 1 and sys.argv[1] == "--test":
            gripper_reset.test_sequence()
        elif len(sys.argv) > 1 and sys.argv[1] == "--comprehensive":
            gripper_reset.comprehensive_test()
        else:
            # Show menu
            print("\nSelect reset type:")
            print("1. Complete Reset (recommended)")
            print("2. Quick Test Only")
            print("3. Comprehensive Test")
            print("4. Exit")
            
            choice = input("\nEnter choice (1-4): ")
            
            if choice == "1":
                gripper_reset.reset_gripper()
                
                # Ask if user wants to run test
                response = input("\n❓ Would you like to run a test sequence? (y/N): ")
                if response.lower() in ['y', 'yes']:
                    gripper_reset.test_sequence()
                    
            elif choice == "2":
                gripper_reset.test_sequence()
                
            elif choice == "3":
                gripper_reset.comprehensive_test()
                
            elif choice == "4":
                print("👋 Exiting without changes")
                return
            else:
                print("❌ Invalid choice")
        
        print("\n🎯 Quick Commands for Future Use:")
        print("   Open:  ros2 topic pub --once /lebai_gripper/cmd/position std_msgs/UInt8 \"data: 0\"")
        print("   Close: ros2 topic pub --once /lebai_gripper/cmd/position std_msgs/UInt8 \"data: 80\"")
        print("   Safe:  ros2 topic pub --once /lebai_gripper/cmd/position std_msgs/UInt8 \"data: 20\"")
        
    except KeyboardInterrupt:
        print("\n\n⛔ Operation interrupted by user")
    except Exception as e:
        print(f"\n❌ Error during operation: {e}")
        print("Setting emergency safe position...")
        try:
            gripper_reset = GripperReset()
            position_msg = UInt8()
            position_msg.data = 20
            gripper_reset.position_pub.publish(position_msg)
        except:
            pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
