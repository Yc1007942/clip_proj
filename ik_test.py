#!/usr/bin/env python3
"""
Quick IK test to diagnose the issue
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from typing import List, Tuple

# Copy the IK functions from waypoint_cartesian.py
UR5_DH = [
    (0.0,       0.1625,  math.pi/2.0),
    (-0.425,    0.0,     0.0),
    (-0.3922,   0.0,     0.0),
    (0.0,       0.1333,  math.pi/2.0),
    (0.0,       0.0997, -math.pi/2.0),
    (0.0,       0.0996,  0.0),
]

def quat_to_rotvec(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    # Ensure normalization
    n = math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    qx /= n; qy /= n; qz /= n; qw /= n
    angle = 2.0 * math.atan2(math.sqrt(qx*qx+qy*qy+qz*qz), qw)
    if angle < 1e-12:
        return (0.0, 0.0, 0.0)
    s = math.sin(angle/2.0)
    ax = qx / s
    ay = qy / s
    az = qz / s
    return (ax*angle, ay*angle, az*angle)

def dh_transform(a, d, alpha, theta):
    ca = math.cos(alpha); sa = math.sin(alpha)
    ct = math.cos(theta); st = math.sin(theta)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0.0,    sa,     ca,    d],
        [0.0,   0.0,    0.0,  1.0]
    ])

def fk_ur5(q: List[float]) -> Tuple[np.ndarray, np.ndarray]:
    T = np.eye(4)
    for i, (a,d,alpha) in enumerate(UR5_DH):
        T = T @ dh_transform(a, d, alpha, q[i])
    pos = T[0:3, 3]
    # Convert rotation to rotvec
    R = T[0:3, 0:3]
    angle = math.acos(max(-1.0, min(1.0, (np.trace(R)-1)/2)))
    if angle < 1e-9:
        rotvec = np.zeros(3)
    else:
        rx = (R[2,1]-R[1,2])/(2*math.sin(angle))
        ry = (R[0,2]-R[2,0])/(2*math.sin(angle))
        rz = (R[1,0]-R[0,1])/(2*math.sin(angle))
        rotvec = np.array([rx, ry, rz]) * angle
    return pos, rotvec

class IKTester(Node):
    def __init__(self):
        super().__init__('ik_tester')
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/tcp_pose_broadcaster/pose', self.pose_cb, 10)
        self.current_joints = None
        self.current_pose = None
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

    def joint_cb(self, msg):
        if len(msg.name) >= 6:
            jd = dict(zip(msg.name, msg.position))
            self.current_joints = [jd.get(n, 0.0) for n in self.joint_names]

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def test_fk_consistency(self):
        if self.current_joints is None or self.current_pose is None:
            print("Waiting for data...")
            return False
        
        print(f"Current joints: {[f'{j:.4f}' for j in self.current_joints]}")
        
        # Forward kinematics from joints
        fk_pos, fk_rot = fk_ur5(self.current_joints)
        print(f"FK result: pos={fk_pos}, rot={fk_rot}")
        
        # Current pose from robot
        rx, ry, rz = quat_to_rotvec(
            self.current_pose.orientation.x,
            self.current_pose.orientation.y, 
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        )
        robot_pos = np.array([self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z])
        robot_rot = np.array([rx, ry, rz])
        print(f"Robot pose: pos={robot_pos}, rot={robot_rot}")
        
        # Compare
        pos_diff = np.linalg.norm(fk_pos - robot_pos)
        rot_diff = np.linalg.norm(fk_rot - robot_rot)
        print(f"Differences: pos={pos_diff:.4f}m, rot={rot_diff:.4f}rad")
        
        return True

def main():
    rclpy.init()
    node = IKTester()
    
    print("Waiting for joint states and pose...")
    for _ in range(50):  # Wait up to 5 seconds
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.test_fk_consistency():
            break
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()