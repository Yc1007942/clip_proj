#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8
import math
import time
import json


class SmoothWaypointController(Node):
    def __init__(self):
        super().__init__('smooth_waypoint_controller')
        
        # Action client for joint trajectory control
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Subscribe to current joint states
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Subscribe to TCP pose
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/tcp_pose_broadcaster/pose',
            self.pose_callback,
            10
        )
        
        # Gripper control publisher
        self.gripper_publisher = self.create_publisher(
            UInt8,
            '/lebai_gripper/cmd/position',
            10
        )
        
        # Subscribe to gripper state
        self.gripper_state_subscription = self.create_subscription(
            UInt8,
            '/lebai_gripper/state/position',
            self.gripper_state_callback,
            10
        )
        
        self.current_joints = None
        self.current_pose = None
        self.current_gripper_position = None
        
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        print("Smooth Waypoint Controller initialized")
        
    def joint_state_callback(self, msg):
        """Update current joint states"""
        if len(msg.name) >= 6:
            # Reorder joints to match expected order
            joint_dict = dict(zip(msg.name, msg.position))
            self.current_joints = [joint_dict.get(name, 0.0) for name in self.joint_names]
    
    def pose_callback(self, msg):
        """Update current TCP pose"""
        self.current_pose = msg.pose
    
    def gripper_state_callback(self, msg):
        """Update current gripper state"""
        self.current_gripper_position = msg.data

    def wait_for_connections(self, timeout=10.0):
        """Wait for action server and topics to be ready"""
        print(" Waiting for robot connections...")
        
        # Wait for action server
        if not self.trajectory_client.wait_for_server(timeout_sec=timeout):
            print(" Failed to connect to trajectory action server!")
            return False
            
        # Wait for joint states
        start_time = time.time()
        while self.current_joints is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if self.current_joints is None:
            print(" Failed to receive joint states!")
            return False
            
        print(" Robot connections ready!")
        return True

    def control_gripper(self, position, wait_time=0.5):
        """Control gripper position (0-100) - faster for continuous motion"""
        if not (0 <= position <= 100):
            print(f" Invalid gripper position {position}. Must be 0-100")
            return False
            
        msg = UInt8()
        msg.data = position
        
        print(f" Moving gripper to position {position}")
        self.gripper_publisher.publish(msg)
        
        # Shorter wait for smoother motion
        time.sleep(wait_time)
        
        return True

    def move_to_joint_waypoint(self, joint_positions, duration=3.0, gripper_position=None, wait_time=0.0):
        """Move robot to specified joint positions - optimized for smooth motion"""
        if len(joint_positions) != 6:
            print(" Joint positions must be a list of 6 values!")
            return False
            
        if self.current_joints is None:
            print(" Current joint states not available!")
            return False
        
        # Control gripper simultaneously with movement start if specified
        if gripper_position is not None:
            self.control_gripper(gripper_position, wait_time=0.2)
        
        # Create trajectory goal with smoother timing
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        # Add current position as starting point
        start_point = JointTrajectoryPoint()
        start_point.positions = list(self.current_joints)
        start_point.velocities = [0.0] * 6
        start_point.time_from_start.sec = 0
        start_point.time_from_start.nanosec = 0
        
        # Add target position with faster movement
        end_point = JointTrajectoryPoint()
        end_point.positions = joint_positions
        end_point.velocities = [0.0] * 6
        end_point.time_from_start.sec = int(duration)
        end_point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        goal_msg.trajectory.points = [start_point, end_point]
        
        print(f" Smooth motion to waypoint: {[round(j, 3) for j in joint_positions]}")
        
        # Send goal and wait for acceptance
        future = self.trajectory_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            print(" Goal rejected!")
            return False
            
        print(" Executing smooth trajectory...")
        
        # Wait for completion
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.error_code == 0:
            print(" Waypoint reached!")
            
            # Minimal wait time for continuous motion
            if wait_time > 0:
                print(f" Brief pause: {wait_time}s")
                time.sleep(wait_time)
                
            return True
        else:
            print(f" Motion failed with error code: {result.error_code}")
            return False

    def load_waypoint_set(self, filename):
        """Load a set of waypoints from a JSON file"""
        try:
            with open(filename, 'r') as f:
                waypoint_set = json.load(f)
            print(f" Loaded waypoint set from {filename}")
            return waypoint_set
        except FileNotFoundError:
            print(f" Waypoint set file {filename} not found!")
            return None
        except Exception as e:
            print(f" Failed to load waypoint set: {e}")
            return None

    def execute_smooth_sequence(self, waypoint_set, inter_waypoint_delay=0.05):
        """Execute waypoints with minimal delays for continuous motion"""
        if not waypoint_set:
            print(" No waypoint set provided!")
            return False
        
        waypoints = waypoint_set.get('waypoints', [])
        set_name = waypoint_set.get('name', 'Unknown')
        
        if not waypoints:
            print(" No waypoints found in set!")
            return False
        
        print(f" Executing SMOOTH sequence: {set_name}")
        print(f"   Number of waypoints: {len(waypoints)}")
        print(f"   Inter-waypoint delay: {inter_waypoint_delay}s for continuous motion")
        
        for i, waypoint in enumerate(waypoints, 1):
            name = waypoint.get('name', f'Waypoint_{i}')
            joints = waypoint.get('joints', [])
            gripper_pos = waypoint.get('gripper', None)
            wait_time = waypoint.get('wait_time', 0.0)
            
            # Reduce individual waypoint wait times for smoother motion
            wait_time = min(wait_time, 0.5)  # Cap wait time at 0.5s
            
            if not joints or len(joints) != 6:
                print(f" Invalid joint data for waypoint '{name}', skipping...")
                continue
            
            print(f"   {i}/{len(waypoints)}: {name}")
            
            # Move with faster duration for smoother motion
            movement_duration = 2.0  # Faster movement
            
            if not self.move_to_joint_waypoint(joints, duration=movement_duration, 
                                             gripper_position=gripper_pos, wait_time=wait_time):
                print(f" Failed to reach waypoint '{name}', stopping sequence.")
                return False
            
            # Very short delay between waypoints for continuous motion
            if i < len(waypoints) and inter_waypoint_delay > 0:
                time.sleep(inter_waypoint_delay)
        
        print(" Smooth sequence completed!")
        return True


def main():
    """Main function for smooth waypoint control"""
    rclpy.init()
    
    try:
        controller = SmoothWaypointController()
        
        if not controller.wait_for_connections():
            return
            
        print("\n🎮 Smooth Waypoint Controller Ready!")
        print("Commands:")
        print("  execute <filename.json> [delay] - Execute smooth waypoint sequence")
        print("    Default inter-waypoint delay: 0.05s (very smooth)")
        print("    Try: execute setA.json 0.02  (for ultra-smooth motion)")
        print("  quit - Exit")
        
        while True:
            try:
                command = input("\nCommand: ").strip().lower()
                
                if command == 'quit' or command == 'q':
                    break
                elif command.startswith('execute '):
                    parts = command.split(' ')
                    if len(parts) < 2:
                        print(" Usage: execute <filename.json> [delay]")
                        continue
                        
                    filename = parts[1]
                    delay = 0.05  # Default very short delay
                    
                    if len(parts) >= 3:
                        try:
                            delay = float(parts[2])
                            if delay < 0:
                                print(" Delay must be >= 0")
                                continue
                        except ValueError:
                            print(" Invalid delay. Must be a number >= 0")
                            continue
                    
                    waypoint_set = controller.load_waypoint_set(filename)
                    if waypoint_set:
                        controller.execute_smooth_sequence(waypoint_set, delay)
                else:
                    print(" Unknown command. Use 'execute <filename.json>' or 'quit'")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
                
    except Exception as e:
        print(f" Error: {e}")
    finally:
        try:
            controller.destroy_node()
        except:
            pass
        rclpy.shutdown()
        print("\n waypoint controller stopped.")


if __name__ == '__main__':
    main()
