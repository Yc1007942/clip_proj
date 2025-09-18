#!/usr/bin/env python3
"""
get_pos_tf.py
=============
Get robot position using TF transforms directly instead of TCP pose broadcaster
This should match exactly what MoveIt sees
"""

import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import math
import time

class UR5ePositionTF(Node):
    def __init__(self):
        super().__init__('ur5e_position_tf')
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribe to joint states
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joints = None
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Reading counter
        self.reading_count = 0
        self.max_readings = 5
        self.should_shutdown = False

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

    def check_and_output(self):
        """Output data when both joints and pose are available"""
        if self.current_joints is not None and self.reading_count < self.max_readings:
            try:
                # Get transform from base_link to tool0 (what MoveIt uses)
                transform = self.tf_buffer.lookup_transform(
                    'base_link', # target frame (same as MoveIt service)
                    'tool0',     # source frame (what MoveIt plans for)
                    rclpy.time.Time()
                )
                
                # Extract position
                position = transform.transform.translation
                orientation = transform.transform.rotation
                
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
                print("pose (base_link→tool0):", pose_array)
                print()
                
                # Increment counter
                self.reading_count += 1
                
                # Check if we've reached max readings
                if self.reading_count >= self.max_readings:
                    print(f"Completed {self.max_readings} readings. Shutting down...")
                    self.should_shutdown = True
                
            except tf2_ros.LookupException as e:
                self.get_logger().warn(f"TF lookup failed: {e}")
            except tf2_ros.ExtrapolationException as e:
                self.get_logger().warn(f"TF extrapolation failed: {e}")

def main():
    rclpy.init()
    node = UR5ePositionTF()
    
    print("Getting robot position using TF (base_link→tool0)...")
    print("What MoveIt sees")
    print()
    
    try:
        while rclpy.ok() and not node.should_shutdown:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()