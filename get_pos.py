#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import time
import math


class UR5ePositionReader(Node):
    def __init__(self):
        super().__init__('ur5e_position_reader')
        
        # Subscribe to TCP pose
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/tcp_pose_broadcaster/pose',
            self.pose_callback,
            10
        )
        
        # Subscribe to joint states
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_pose = None
        self.current_joints = None
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
    def quaternion_to_rotvec(self, x, y, z, w):
        """Convert quaternion to rotation vector (axis-angle representation)"""
        # Normalize quaternion
        norm = math.sqrt(x*x + y*y + z*z + w*w)
        if norm < 1e-12:
            return 0.0, 0.0, 0.0
            
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        # Handle w near 1 (small rotation)
        if abs(w) > 0.999999:
            return 0.0, 0.0, 0.0
            
        # Compute angle and axis
        angle = 2.0 * math.acos(abs(w))
        if w < 0:
            # Ensure shortest rotation
            angle = 2.0 * math.pi - angle
            
        s = math.sqrt(1.0 - w*w)
        if s < 1e-12:
            return 0.0, 0.0, 0.0
            
        # Return rotation vector (axis * angle)
        return (x/s) * angle, (y/s) * angle, (z/s) * angle
        
    def joint_state_callback(self, msg):
        """Process received joint state data"""
        if len(msg.name) >= 6:
            # Reorder joints to match expected order
            joint_dict = dict(zip(msg.name, msg.position))
            self.current_joints = [joint_dict.get(name, 0.0) for name in self.joint_names]
            self.check_and_output()
    
    def pose_callback(self, msg):
        """Process received pose data"""
        self.current_pose = msg.pose
        self.check_and_output()
    
    def check_and_output(self):
        """Output data when both joints and pose are available"""
        if self.current_joints is not None and self.current_pose is not None:
            # Get pose data
            position = self.current_pose.position
            orientation = self.current_pose.orientation
            
            # Convert quaternion to rotation vector
            rx, ry, rz = self.quaternion_to_rotvec(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
            
            # Create output arrays
            joints_array = [round(j, 4) for j in self.current_joints]
            pose_array = [
                round(position.x, 4), 
                round(position.y, 4), 
                round(position.z, 4), 
                round(rx, 4), 
                round(ry, 4), 
                round(rz, 4)
            ]
            
            # Output both arrays
            print("joints:", joints_array)
            print("pose:", pose_array)
            
            # Exit after getting one reading
            raise KeyboardInterrupt


def main():
    """Main function to get current robot position"""
    rclpy.init()
    
    try:
        node = UR5ePositionReader()
        
        # Wait for pose data with timeout
        start_time = time.time()
        timeout = 5.0  # 5 second timeout
        
        try:
            while time.time() - start_time < timeout:
                rclpy.spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
            
        if time.time() - start_time >= timeout:
            print("Error: Timeout waiting for position data")
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
