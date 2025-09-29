
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import json
import time
import math
import argparse
import re
from typing import List, Optional, Dict
from geometry_msgs.msg import Pose, WrenchStamped
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan
from moveit_msgs.msg import RobotTrajectory, MoveItErrorCodes, MotionPlanRequest, PlanningOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

def rotvec_to_quat(rx, ry, rz):
    angle = math.sqrt(rx*rx + ry*ry + rz*rz)
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    ax, ay, az = rx/angle, ry/angle, rz/angle
    s = math.sin(angle/2.0)
    return (ax*s, ay*s, az*s, math.cos(angle/2.0))

class MultiTaskMoveIt(Node):
    GLOBAL_Z_OFFSET = 0 
    
    def __init__(self):
        super().__init__('multi_task_moveit')
        self.cartesian_path_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.motion_plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.trajectory_action_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.gripper_pub = self.create_publisher(UInt8, '/lebai_gripper/cmd/position', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Force/Torque sensor monitoring
        self.wrench_sub = self.create_subscription(
            WrenchStamped, 
            '/ur_robot/force_torque_sensor_broadcaster/wrench',
            self.wrench_callback, 
            10
        )
        self.current_wrench = None
        self.wrench_history = []  # For moving average
        self.history_size = 5  # Number of samples for moving average
        self.force_monitoring_enabled = True
        
        # Force thresholds (configurable parameters)
        self.force_threshold_x = 5.0  # Newtons
        self.force_threshold_y = 5.0  # Newtons  
        self.force_threshold_z = 5.0  # Newtons
        self.torque_threshold_x = 5.0  # Newton-meters
        self.torque_threshold_y = 5.0  # Newton-meters
        self.torque_threshold_z = 5.0  # Newton-meters
        self.force_resultant_threshold = 5.0  # Overall force magnitude limit
        self.consecutive_violations_required = 2  # Require N consecutive violations
        self.consecutive_violations_count = 0
        self.force_failure_detected = False
        self.current_goal_handle = None
        
        self.current_joint_state = None
        
        # Task execution counter for height adjustment
        self.task_counter = 0
        self.initial_height = 0.325  # Initial height for xy1_up and xy1_up_2
        self.height_decrement = 0.0235  # Height reduction per cycle
        self.minimum_height = 0.28  # Minimum allowed height

    def get_current_height(self):
        current_height = self.initial_height - (self.task_counter * self.height_decrement)
        return max(current_height, self.minimum_height)
    
    def increment_task_counter(self):
        self.task_counter += 1
        current_height = self.get_current_height()
        if current_height <= self.minimum_height:
            print(f"WARNING: Minimum height ({self.minimum_height}m) reached!")
        return current_height

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def wrench_callback(self, msg):
        self.current_wrench = msg
        self.wrench_history.append(msg)
        if len(self.wrench_history) > self.history_size:
            self.wrench_history.pop(0)
        if self.force_monitoring_enabled and len(self.wrench_history) >= self.consecutive_violations_required:
            self.check_force_violations()

    def calculate_average_wrench(self):
        if not self.wrench_history:
            return None
            
        n = len(self.wrench_history)
        avg_force_x = sum(w.wrench.force.x for w in self.wrench_history) / n
        avg_force_y = sum(w.wrench.force.y for w in self.wrench_history) / n
        avg_force_z = sum(w.wrench.force.z for w in self.wrench_history) / n
        avg_torque_x = sum(w.wrench.torque.x for w in self.wrench_history) / n
        avg_torque_y = sum(w.wrench.torque.y for w in self.wrench_history) / n
        avg_torque_z = sum(w.wrench.torque.z for w in self.wrench_history) / n
        
        return {
            'force': {'x': avg_force_x, 'y': avg_force_y, 'z': avg_force_z},
            'torque': {'x': avg_torque_x, 'y': avg_torque_y, 'z': avg_torque_z}
        }

    def check_force_violations(self):
        avg_wrench = self.calculate_average_wrench()
        if not avg_wrench:
            return False
            
        force = avg_wrench['force']
        torque = avg_wrench['torque']
        
        # Calculate force resultant magnitude
        force_magnitude = math.sqrt(force['x']**2 + force['y']**2 + force['z']**2)
        
        violations = []
        if abs(force['x']) > self.force_threshold_x:
            violations.append(f"Force X: {force['x']:.1f}N > {self.force_threshold_x}N")
        if abs(force['y']) > self.force_threshold_y:
            violations.append(f"Force Y: {force['y']:.1f}N > {self.force_threshold_y}N")
        if abs(force['z']) > self.force_threshold_z:
            violations.append(f"Force Z: {force['z']:.1f}N > {self.force_threshold_z}N")
        if abs(torque['x']) > self.torque_threshold_x:
            violations.append(f"Torque X: {torque['x']:.1f}Nm > {self.torque_threshold_x}Nm")
        if abs(torque['y']) > self.torque_threshold_y:
            violations.append(f"Torque Y: {torque['y']:.1f}Nm > {self.torque_threshold_y}Nm")
        if abs(torque['z']) > self.torque_threshold_z:
            violations.append(f"Torque Z: {torque['z']:.1f}Nm > {self.torque_threshold_z}Nm")
        if force_magnitude > self.force_resultant_threshold:
            violations.append(f"Force magnitude: {force_magnitude:.1f}N > {self.force_resultant_threshold}N")
        

        if violations:
            self.consecutive_violations_count += 1
            print(f"Force violation {self.consecutive_violations_count}/{self.consecutive_violations_required}: {', '.join(violations)}")
            
            if self.consecutive_violations_count >= self.consecutive_violations_required:
                self.trigger_force_failure(violations, force_magnitude)
                return True
        else:
            self.consecutive_violations_count = 0
            
        return False

    def trigger_force_failure(self, violations, force_magnitude):
        if self.force_failure_detected:
            return  
            
        self.force_failure_detected = True
        
        print(f"Force magnitude: {force_magnitude:.2f}N")
        print(f"Violations: {', '.join(violations)}")
        
        if self.current_goal_handle and not self.current_goal_handle.get_result_async().done():
            cancel_future = self.current_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
        
        time.sleep(0.5)
        self.set_gripper(100)
        time.sleep(1.0)  
        

    def execute_failure_recovery(self, waypoints_data, test_mode=False):
        try:
            failure_home_waypoint = None
            for waypoint in waypoints_data.get('waypoints', []):
                if waypoint.get('name') == 'failure_home':
                    failure_home_waypoint = waypoint
                    break
            
            if not failure_home_waypoint:
                print("ERROR: failure_home waypoint not found")
                return False
            
            print(f"Executing recovery: {failure_home_waypoint['pose']}")
            
            single_waypoint = [failure_home_waypoint]
            trajectory, fraction = self.plan_cartesian_path(single_waypoint, 0.01, 0.1)
            
            if trajectory and fraction > 0.1:
                print(f"Recovery path planned with {fraction*100:.1f}% coverage")
                original_monitoring = self.force_monitoring_enabled
                original_failure_state = self.force_failure_detected
                self.force_monitoring_enabled = False
                self.force_failure_detected = False
                recovery_success = self.execute_trajectory(trajectory, 60.0, 0.1, test_mode=test_mode)
                self.force_monitoring_enabled = original_monitoring
                self.force_failure_detected = original_failure_state
            else:
                recovery_success = False
            
            if recovery_success:
                return True
            else:
                return False
                
        except Exception as e:
            print(f"ERROR: {e}")
            return False
        finally:
            self.consecutive_violations_count = 0

    def wait_for_services(self, timeout=20.0):
        print("Waiting for MoveIt services...")
        
        if not self.cartesian_path_client.wait_for_service(timeout_sec=timeout):
            return False
            
        if not self.motion_plan_client.wait_for_service(timeout_sec=timeout):
            return False
            
        if not self.trajectory_action_client.wait_for_server(timeout_sec=timeout):
            return False
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.current_joint_state is not None:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            return False
    
        return True

    def create_pose_from_array(self, pose_array):
        x, y, z, rx, ry, rz = pose_array
        qx, qy, qz, qw = rotvec_to_quat(rx, ry, rz)
        z_offset = self.GLOBAL_Z_OFFSET
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y  
        pose.position.z = z + z_offset
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def plan_cartesian_path(self, waypoints, eef_step=0.005, min_fraction=0.8):
        request = GetCartesianPath.Request()
        request.header.frame_id = "base_link"
        request.header.stamp = self.get_clock().now().to_msg()
        request.waypoints = [self.create_pose_from_array(wp['pose']) for wp in waypoints]
        request.max_step = eef_step
        request.jump_threshold = 2.0
        request.group_name = "ur_manipulator"
        request.link_name = "tool0"  
        if self.current_joint_state:
            request.start_state.joint_state = self.current_joint_state
            print(f"Using current joint state with {len(self.current_joint_state.position)} joints")
        else:
            print("WARNING: No current joint state available for planning!")
        
        future = self.cartesian_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.error_code.val == MoveItErrorCodes.SUCCESS:
                return response.solution, response.fraction
            else:
                print(f"Planning failed: {response.error_code.val}")
                return None, 0.0
        else:
            return None, 0.0

    def execute_trajectory(self, trajectory: RobotTrajectory, timeout=120.0, velocity_scale=0.1, test_mode=False):
        if not trajectory or not trajectory.joint_trajectory.points:
            return False
        
        if test_mode:
            print(f" TEST MODE: Simulating trajectory execution with {len(trajectory.joint_trajectory.points)} points")
            print(f" TEST MODE: Would execute at {velocity_scale*100:.1f}% speed for ~{timeout}s max")
            time.sleep(1.0) 
            return True
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory
        for point in goal.trajectory.points:
            if point.velocities:
                point.velocities = [v * velocity_scale for v in point.velocities]
            if point.accelerations:
                point.accelerations = [a * velocity_scale * velocity_scale for a in point.accelerations]
            total_time_ns = point.time_from_start.sec * 1e9 + point.time_from_start.nanosec
            scaled_time_ns = int(total_time_ns / velocity_scale)
            point.time_from_start.sec = int(scaled_time_ns // 1e9)
            point.time_from_start.nanosec = int(scaled_time_ns % 1e9)
       
        future = self.trajectory_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Trajectory goal rejected")
            return False
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        start_time = time.time()
        
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.time() - start_time
            if self.force_failure_detected:
                return False  # Force failure handling already done in trigger_force_failure
            
            if elapsed > timeout:
                print(f"Timed out after {timeout} ")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                return False
        self.current_goal_handle = None
            
        result = result_future.result()
        if result and result.result.error_code == 0:
            return True
        else:
            error_code = result.result.error_code if result else "unknown"
            print(f"Failed with error: {error_code}")
            return False

    def set_gripper(self, value):
        msg = UInt8()
        msg.data = value
        self.gripper_pub.publish(msg)

    def execute_waypoints(self, waypoints: List[dict], eef_step=0.005, min_fraction=0.8, timeout=120.0, velocity_scale=0.1, waypoints_data=None, test_mode=False):
        
        if test_mode:
        
        for i, waypoint in enumerate(waypoints):
            print(f"\nWaypoint {i+1}: {waypoint.get('name', f'waypoint_{i+1}')}")
            if self.force_failure_detected and waypoints_data and not test_mode:
                recovery_success = self.execute_failure_recovery(waypoints_data, test_mode=test_mode)
                if recovery_success:
                else:
                    print("Recovery failed. Manual intervention required.")
                return False
            single_waypoint = [waypoint]
            trajectory, fraction = self.plan_cartesian_path(single_waypoint, eef_step, min_fraction)
            
            if not trajectory:
                return False
                
            print(f"Planned trajectory covers {fraction*100:.2f}% of path")
            if fraction < min_fraction:
                print(f"Path coverage {fraction*100:.2f}% is below minimum required {min_fraction*100:.2f}%")
                return False
            success = self.execute_trajectory(trajectory, timeout, velocity_scale, test_mode=test_mode)
            
            if self.force_failure_detected and not test_mode:
                if waypoints_data:
                    recovery_success = self.execute_failure_recovery(waypoints_data, test_mode=test_mode)
                    if recovery_success:
                        print("Recovery completed. Task execution aborted.")
                    else:
                        print("Manual intervention required.")
                return False
            
            if not success:
                return False
            
            if 'gripper' in waypoint:
                if test_mode:
                    print(f" TEST MODE: Would set gripper to {waypoint['gripper']}")
                else:
                    self.set_gripper(waypoint['gripper'])
                time.sleep(0.1 if test_mode else 0.5) 
            
            if 'wait_time' in waypoint and waypoint['wait_time'] > 0:
                wait_time = 0.2 if test_mode else waypoint['wait_time']  # Shorter wait in test mode
                print(f"Waiting {wait_time} seconds..." + (" (test mode)" if test_mode else ""))
                time.sleep(wait_time)
        
        return True

def update_dynamic_coordinates(waypoints_data, target_x, target_y):
    updated_count = 0
    for waypoint in waypoints_data.get('waypoints', []):
        if waypoint.get('name', '').startswith('xy1'):
            old_x, old_y = waypoint['pose'][0], waypoint['pose'][1]
            waypoint['pose'][0] = target_x
            waypoint['pose'][1] = target_y
            updated_count += 1
            print(f"  Updated {waypoint['name']}: ({old_x:.4f}, {old_y:.4f}) → ({target_x:.4f}, {target_y:.4f})")
    
    return updated_count

def update_height_coordinates(waypoints_data, new_height):
    updated_count = 0
    target_waypoints = ['xy1_up', 'xy1_up_2']
    
    for waypoint in waypoints_data.get('waypoints', []):
        waypoint_name = waypoint.get('name', '')
        if waypoint_name in target_waypoints:
            old_z = waypoint['pose'][2]
            waypoint['pose'][2] = new_height
            updated_count += 1
            print(f"  Updated {waypoint_name} height: {old_z:.3f}m to {new_height:.3f}m")
    
    return updated_count

def get_fresh_coordinates(test_mode=False):
    if test_mode:
        return 0.5, 0.3  
        
    try:
        import sys
        import os
        sys.path.append(os.path.dirname(os.path.abspath(__file__)))
        from get_multi_coordinates import get_target_coordinates
        
        target_x, target_y = get_target_coordinates()
        print(f"Fresh coordinates from get_coordinates.py: X={target_x:.4f}, Y={target_y:.4f}")
        return target_x, target_y
    except ImportError:
        print("ERROR: Could not import get_coordinates.py")
        print("Falling back to test coordinates...")
        return 0.5, 0.3  # Fallback test coordinates-----------------------
    except Exception as e:
        return 0.5, 0.3  
def extract_tasks_from_waypoints(waypoints_data):
    tasks = set()
    for waypoint in waypoints_data.get('waypoints', []):
        name = waypoint.get('name', '')
        # Look for task patterns like 'clip1_', 'clip2_'
        match = re.match(r'^(clip\d+)_', name)
        if match:
            tasks.add(match.group(1))
    
    return sorted(list(tasks))

def filter_waypoints_by_task(waypoints_data, task_name):
    filtered_waypoints = []
    for waypoint in waypoints_data.get('waypoints', []):
        name = waypoint.get('name', '')
        if (name.startswith(f'{task_name}_') or 
            name in ['home', 'xy1', 'xy1_down', 'xy1_up'] or
            name.startswith('xy1')):
            filtered_waypoints.append(waypoint)
    
    return filtered_waypoints

def display_task_waypoints(waypoints, task_name):
    for i, wp in enumerate(waypoints):
        name = wp.get('name', f'waypoint_{i+1}')
        pose = wp.get('pose', [0, 0, 0, 0, 0, 0])
        gripper = wp.get('gripper', 'N/A')
        print(f"  {i+1:2d}. {name:20s} | XYZ: [{pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}] | Gripper: {gripper}")

def main():
    parser = argparse.ArgumentParser(description='Multi-task waypoint execution with dynamic coordinates')
    parser.add_argument('--file', '-f', default='seti.json', 
                        help='Waypoint JSON file to execute (default: seti.json)')
    parser.add_argument('--eef-step', type=float, default=0.005,
                        help='End-effector step size for Cartesian planning (default: 0.005)')
    parser.add_argument('--min-fraction', type=float, default=0.15,
                        help='Minimum path fraction for successful planning (default: 0.15)')
    parser.add_argument('--timeout', type=int, default=30,
                        help='Execution timeout in seconds (default: 30)')
    parser.add_argument('--velocity-scale', type=float, default=0.3,
                        help='Velocity scaling factor for safe execution (default: 0.3)')
    parser.add_argument('--force-threshold', type=float, default=5.0,
                        help='Force threshold in Newtons for X,Y,Z axes (default: 50.0)')
    parser.add_argument('--torque-threshold', type=float, default=5.0,
                        help='Torque threshold in Newton-meters for X,Y,Z axes (default: 10.0)')
    parser.add_argument('--force-resultant-threshold', type=float, default=5.0,
                        help='Overall force magnitude threshold in Newtons (default: 60.0)')
    parser.add_argument('--consecutive-violations', type=int, default=3,
                        help='Number of consecutive violations required to trigger abort (default: 3)')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = MultiTaskMoveIt()
    node.force_threshold_x = args.force_threshold
    node.force_threshold_y = args.force_threshold
    node.force_threshold_z = args.force_threshold
    node.torque_threshold_x = args.torque_threshold
    node.torque_threshold_y = args.torque_threshold
    node.torque_threshold_z = args.torque_threshold
    node.force_resultant_threshold = args.force_resultant_threshold
    node.consecutive_violations_required = args.consecutive_violations
    if not node.wait_for_services():
        print("Failed to connect to services")
        rclpy.shutdown()
        return
    try:
        with open(args.file, 'r') as f:
            base_waypoints_data = json.load(f)
    except FileNotFoundError:
        print(f"ERROR: File {args.file} not found")
        rclpy.shutdown()
        return
    except Exception as e:
        print(f"ERROR: Failed to load waypoints: {e}")
        rclpy.shutdown()
        return
    available_tasks = extract_tasks_from_waypoints(base_waypoints_data)
    
    if not available_tasks:
        print("No tasks found in waypoint file")
        rclpy.shutdown()
        return
    
    print(f"\nAvailable tasks in {args.file}: {', '.join(available_tasks)}")
    try:
        while True:
            print(f"Available tasks: {', '.join(available_tasks)}")
            print(f"Task cycles completed: {node.task_counter}")
            print(f"Current height setting: {node.get_current_height():.3f}m")
            if node.current_wrench is not None:
                wrench = node.current_wrench.wrench
                force_mag = math.sqrt(wrench.force.x**2 + wrench.force.y**2 + wrench.force.z**2)
                print(f"Force sensor:  Active (Current: {force_mag:.1f}N)")
            else:
                print(f"Force sensor:  No data (monitoring disabled)")
                node.force_monitoring_enabled = False
            
            if node.force_failure_detected:
                print(" FORCE FAILURE STATE Recovery required")
            
            print("Commands: <task_name> | 'list' | 'test' | 'test-task' | 'quit'")
            
            user_input = input("\nSelect task to execute: ").strip().lower()
            
            if user_input == 'quit' or user_input == 'q':
                break
            elif user_input == 'test' or user_input == 't':
                print("This will simulate a force failure and test the recovery sequence")
                confirm = input("Proceed with force failure test? (y/N): ").strip().lower()
                if confirm == 'y':
                    print(" Simulating force failure...")
                    violations = ["Test Force X: 75.0N > 50.0N", "Test simulation"]
                    node.trigger_force_failure(violations, 75.0)
                    recovery_success = node.execute_failure_recovery(base_waypoints_data, test_mode=True)
                    
                    if recovery_success:
                        print(" Recovery successful")
                    else:
                        print(" Recovery unsuccessful")
                    input("Press Enter to acknowledge and continue...")
                    node.force_failure_detected = False
                else:
                    print("Force failure test cancelled")
                continue
            elif user_input == 'test-task':
                print(f"Available tasks: {', '.join(available_tasks)}")
                task_input = input("Enter task name to test: ").strip().lower()
                
                if task_input in available_tasks:
                    print(f" Testing task '{task_input}' with simulated robot execution")
                    target_x, target_y = get_fresh_coordinates(test_mode=True)
                    working_data = json.loads(json.dumps(base_waypoints_data))
                    updated_count = update_dynamic_coordinates(working_data, target_x, target_y)
                    if updated_count > 0:
                        print(f"Updated {updated_count} waypoints with test coordinates")
                    current_height = node.get_current_height()
                    height_updated_count = update_height_coordinates(working_data, current_height)
                    if height_updated_count > 0:
                        print(f"Updated {height_updated_count} waypoints with cycle height: {current_height:.3f}m")
                    task_waypoints = filter_waypoints_by_task(working_data, task_input)
                    display_task_waypoints(task_waypoints, task_input)
                    print(f"\n Executing {len(task_waypoints)} waypoints in TEST MODE")
                    success = node.execute_waypoints(task_waypoints, 
                                                   eef_step=args.eef_step, 
                                                   min_fraction=args.min_fraction,
                                                   timeout=args.timeout,
                                                   velocity_scale=args.velocity_scale,
                                                   waypoints_data=working_data,
                                                   test_mode=True)
                    
                    if success:
                        print(f" Test execution of task '{task_input}' completed successfully")
                    else:
                        print(f" Test execution of task '{task_input}' failed")
                else:
                    print(f"Invalid task '{task_input}'. Available: {', '.join(available_tasks)}")
                continue
            elif user_input == 'list' or user_input == 'l':
                for task in available_tasks:
                    task_waypoints = filter_waypoints_by_task(base_waypoints_data, task)
                    display_task_waypoints(task_waypoints, task)
                continue
            elif user_input not in available_tasks:
                print(f"Invalid task '{user_input}'. Available: {', '.join(available_tasks)}")
                continue
            
            selected_task = user_input
            target_x, target_y = get_fresh_coordinates()
            if target_x is None or target_y is None:
                continue
            
            working_data = json.loads(json.dumps(base_waypoints_data))  # Deep copy
            updated_count = update_dynamic_coordinates(working_data, target_x, target_y)
            if updated_count > 0:
                print(f"Updated {updated_count} waypoints")
            current_height = node.get_current_height()
            height_updated_count = update_height_coordinates(working_data, current_height)
            if height_updated_count > 0:
                print(f"Updated {height_updated_count} waypoints with cycle height: {current_height:.3f}m")
            task_waypoints = filter_waypoints_by_task(working_data, selected_task)
            
            if not task_waypoints:
                print(f"No waypoints found for task '{selected_task}'")
                continue
            display_task_waypoints(task_waypoints, selected_task)
            print(f"\nReady to execute {len(task_waypoints)} waypoints for task '{selected_task}'")
            confirm = input("Proceed with execution? (y/N): ").strip().lower()
            if confirm != 'y':
                print("Task execution cancelled")
                continue
            
            print(f"\n Starting execution of task: {selected_task.upper()}")
            print(f"Task execution cycle: {node.task_counter + 1}")
            print(f"Using coordinates: X={target_x:.4f}, Y={target_y:.4f}")
            print(f"Current cycle height: {current_height:.3f}m")
            print(f"Z-offset applied: {node.GLOBAL_Z_OFFSET*100:.0f}cm")
            success = node.execute_waypoints(task_waypoints, 
                                           eef_step=args.eef_step, 
                                           min_fraction=args.min_fraction,
                                           timeout=args.timeout,
                                           velocity_scale=args.velocity_scale,
                                           waypoints_data=working_data)
            
            if success:
                if not node.force_failure_detected:
                    next_height = node.increment_task_counter()
                    if next_height <= node.minimum_height:
                        print(f" Minimum height reached. Next cycles will use {node.minimum_height:.3f}m")
                else:
                    print(" Force failure occurred  counter not incremented")
            else:
                print(f" Task '{selected_task}' execution failed")
                if node.force_failure_detected:
                    input("Press Enter to acknowledge and continue")
                    node.force_failure_detected = False
                    print(" Failure state cleared")
                else:
                    print("Task counter not incremented due to execution failure")
                
            print("\nReady for next task...")
            
    except KeyboardInterrupt:
        print("\n\nInterrupted by user. Exiting")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
