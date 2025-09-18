

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import time
import math

class UR5eMonitorAndControl(Node):
    def __init__(self):
        super().__init__('ur5e_monitor_control')
        
        # Subscribers for robot state
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.tcp_pose_sub = self.create_subscription(
            PoseStamped,
            '/tcp_pose_broadcaster/pose',
            self.tcp_pose_callback,
            10
        )
        
        # Action client for robot control
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Robot state variables
        self.current_joint_positions = None
        self.current_joint_names = None
        self.current_tcp_pose = None
        
        # UR5e joint names in order
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        self.get_logger().info("UR5e Monitor and Control Node initialized")
        self.get_logger().info("Waiting for robot data...")

    def joint_state_callback(self, msg):
        """Callback for joint state updates"""
        self.current_joint_names = msg.name
        self.current_joint_positions = msg.position
        
    def tcp_pose_callback(self, msg):
        """Callback for TCP pose updates"""
        self.current_tcp_pose = msg
        
    def get_current_cartesian_position(self):
        """Get current cartesian coordinates of the robot TCP"""
        if self.current_tcp_pose is None:
            self.get_logger().warn("No TCP pose data available yet")
            return None
            
        pose = self.current_tcp_pose.pose
        position = pose.position
        orientation = pose.orientation
        
        # Convert quaternion to Euler angles for easier interpretation
        x, y, z = position.x, position.y, position.z
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
        
        # Convert quaternion to roll, pitch, yaw
        roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
        pitch = math.asin(2*(qw*qy - qz*qx))
        yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        
        return {
            'position': {'x': x, 'y': y, 'z': z},
            'orientation_quaternion': {'x': qx, 'y': qy, 'z': qz, 'w': qw},
            'orientation_euler': {'roll': roll, 'pitch': pitch, 'yaw': yaw},
            'timestamp': self.current_tcp_pose.header.stamp
        }
    
    def get_current_joint_positions(self):
        """Get current joint positions"""
        if self.current_joint_positions is None:
            self.get_logger().warn("No joint state data available yet")
            return None
            
        joint_dict = {}
        for i, name in enumerate(self.current_joint_names):
            if i < len(self.current_joint_positions):
                joint_dict[name] = self.current_joint_positions[i]
        
        return joint_dict
    
    def print_current_state(self):
        """Print current robot state in a formatted way"""
        print("\n" + "="*60)
        print("           UR5e ROBOT CURRENT STATE")
        print("="*60)
        
        # Print cartesian coordinates
        cartesian = self.get_current_cartesian_position()
        if cartesian:
            pos = cartesian['position']
            euler = cartesian['orientation_euler']
            
            print(f"\n🤖 TCP CARTESIAN COORDINATES:")
            print(f"   Position (m):  X={pos['x']:.4f}, Y={pos['y']:.4f}, Z={pos['z']:.4f}")
            print(f"   Orientation (rad): Roll={euler['roll']:.4f}, Pitch={euler['pitch']:.4f}, Yaw={euler['yaw']:.4f}")
            print(f"   Orientation (deg): Roll={math.degrees(euler['roll']):.2f}°, Pitch={math.degrees(euler['pitch']):.2f}°, Yaw={math.degrees(euler['yaw']):.2f}°")
        
        # Print joint positions
        joints = self.get_current_joint_positions()
        if joints:
            print(f"\n🔧 JOINT POSITIONS:")
            for joint_name in self.joint_names:
                if joint_name in joints:
                    angle_rad = joints[joint_name]
                    angle_deg = math.degrees(angle_rad)
                    print(f"   {joint_name:<20}: {angle_rad:>8.4f} rad ({angle_deg:>7.2f}°)")
        
        print("="*60)
    
    def rotate_wrist(self, angle_change_deg=30.0):
        """Rotate the wrist (gripper) by specified angle to test responsiveness"""
        
        if not self.trajectory_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Trajectory action server not available!")
            return False
            
        if self.current_joint_positions is None:
            self.get_logger().error("No current joint positions available!")
            return False
        
        # Get current joint positions in the correct order
        current_positions = []
        for joint_name in self.joint_names:
            try:
                idx = self.current_joint_names.index(joint_name)
                current_positions.append(self.current_joint_positions[idx])
            except ValueError:
                self.get_logger().error(f"Joint {joint_name} not found in current state!")
                return False
        
        # Create new positions with wrist rotation
        new_positions = current_positions.copy()
        
        # Rotate wrist_3_joint (last joint - gripper rotation)
        angle_change_rad = math.radians(angle_change_deg)
        new_positions[5] += angle_change_rad  # wrist_3_joint is index 5
        
        self.get_logger().info(f"Rotating gripper by {angle_change_deg}° ({angle_change_rad:.4f} rad)")
        
        # Create trajectory goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = new_positions
        point.time_from_start = Duration(sec=2, nanosec=0)  # 2 seconds to complete
        
        goal_msg.trajectory.points = [point]
        
        # Send goal
        self.get_logger().info("Sending gripper rotation command...")
        future = self.trajectory_client.send_goal_async(goal_msg)
        
        return True
    
    def demo_sequence(self):
        """Run a demonstration sequence"""
        self.get_logger().info("Starting UR5e demonstration sequence...")
        
        # Wait for initial data
        timeout = 10  # 10 seconds timeout
        start_time = time.time()
        
        while (self.current_tcp_pose is None or self.current_joint_positions is None):
            if time.time() - start_time > timeout:
                self.get_logger().error("Timeout waiting for robot data!")
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Display initial state
        self.get_logger().info("📊 Initial robot state:")
        self.print_current_state()
        
        # Test gripper rotation
        self.get_logger().info("\n🔄 Testing gripper responsiveness...")
        
        # Rotate gripper clockwise
        input("\nPress Enter to rotate gripper +30°...")
        if self.rotate_wrist(30.0):
            time.sleep(3)  # Wait for movement
            rclpy.spin_once(self, timeout_sec=0.5)
            self.print_current_state()
        
        # Rotate gripper counter-clockwise  
        input("\nPress Enter to rotate gripper -60°...")
        if self.rotate_wrist(-60.0):
            time.sleep(3)  # Wait for movement
            rclpy.spin_once(self, timeout_sec=0.5)
            self.print_current_state()
        
        # Return to original position
        input("\nPress Enter to rotate gripper back +30°...")
        if self.rotate_wrist(30.0):
            time.sleep(3)  # Wait for movement
            rclpy.spin_once(self, timeout_sec=0.5)
            self.print_current_state()
        
        self.get_logger().info("✅ Demo sequence completed!")

def main(args=None):
    rclpy.init(args=args)
    
    node = UR5eMonitorAndControl()
    
    try:
        # Run the demonstration
        node.demo_sequence()
        
        # Keep running to monitor position
        print("\n🔄 Continuous monitoring mode (Ctrl+C to stop)...")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
            node.print_current_state()
            time.sleep(2)  # Update every 2 seconds
            
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
