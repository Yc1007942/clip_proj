#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
import time

class GripperRotator(Node):
    def __init__(self):
        super().__init__('gripper_rotator')
        self.joint_state = None
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

    def joint_state_callback(self, msg):
        self.joint_state = msg

    def rotate_gripper(self, delta_rad=0.5236):
        self.get_logger().info('Waiting for action server...')
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return
        self.get_logger().info('Waiting for joint states...')
        timeout = time.time() + 5
        while self.joint_state is None and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.joint_state is None:
            self.get_logger().error('No joint state received!')
            return
        # Get current joint positions
        positions = list(self.joint_state.position)
        # Rotate wrist_3_joint (index 5)
        positions[5] += delta_rad
        traj = JointTrajectory()
        traj.joint_names = self.joint_state.name
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 3
        traj.points.append(point)
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj
        self.get_logger().info(f'Sending gripper rotation command: {delta_rad} rad')
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        self.get_logger().info('Goal accepted, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()
        self.get_logger().info('Gripper rotation complete!')

def main():
    rclpy.init()
    node = GripperRotator()
    # Rotate gripper +30 degrees (0.5236 rad)
    node.rotate_gripper(0.5236)
    # Optionally, rotate back -30 degrees
    # node.rotate_gripper(-0.5236)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
