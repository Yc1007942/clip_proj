
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import json
import time
import math
import argparse
import re
from typing import List, Optional, Dict

from geometry_msgs.msg import Pose
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
    GLOBAL_Z_OFFSET = 0  # No Z-offset by default
    
    def __init__(self):
        super().__init__('multi_task_moveit')
        self.cartesian_path_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.motion_plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.trajectory_action_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.gripper_pub = self.create_publisher(UInt8, '/lebai_gripper/cmd/position', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.current_joint_state = None
        
        print("[MultiTaskMoveIt] Initialized")

    def joint_state_callback(self, msg):
        """Store current joint state for planning"""
        self.current_joint_state = msg

    def wait_for_services(self, timeout=10.0):
        """Wait for required services to become available"""
        print("Waiting for MoveIt services...")
        
        if not self.cartesian_path_client.wait_for_service(timeout_sec=timeout):
            print("Cartesian path service not available")
            return False
            
        if not self.motion_plan_client.wait_for_service(timeout_sec=timeout):
            print("Motion planning service not available") 
            return False
            
        if not self.trajectory_action_client.wait_for_server(timeout_sec=timeout):
            print("Trajectory action server not available")
            return False
        
        # Wait for joint state
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.current_joint_state is not None:
                break
            rclpy.spin_once(self, timeout_sec=0.1)  # Process callbacks
        else:
            print("No joint state received")
            return False
            
        print("All services ready!")
        return True

    def create_pose_from_array(self, pose_array):
        """Convert [x,y,z,rx,ry,rz] to geometry_msgs/Pose with Z-offset"""
        x, y, z, rx, ry, rz = pose_array
        qx, qy, qz, qw = rotvec_to_quat(rx, ry, rz)
        
        # Apply global Z-offset
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
        """Plan Cartesian path through waypoints"""
        request = GetCartesianPath.Request()
        
        # Set header
        request.header.frame_id = "base_link"
        request.header.stamp = self.get_clock().now().to_msg()
        
        # Convert waypoint arrays to Pose messages
        request.waypoints = [self.create_pose_from_array(wp['pose']) for wp in waypoints]
        request.max_step = eef_step
        request.jump_threshold = 2.0
        request.group_name = "ur_manipulator"
        request.link_name = "tool0"  # Explicitly specify the tool frame
        
        # Add current robot state (simplified)
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
                print(f"Cartesian planning failed: {response.error_code.val}")
                return None, 0.0
        else:
            print("Cartesian planning service call failed")
            return None, 0.0

    def execute_trajectory(self, trajectory: RobotTrajectory, timeout=30.0, velocity_scale=0.1):
        """Execute a planned trajectory"""
        if not trajectory or not trajectory.joint_trajectory.points:
            print("No valid trajectory to execute")
            return False
        
        # Create trajectory goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory
        
        # Scale velocities, accelerations, and timing
        for point in goal.trajectory.points:
            if point.velocities:
                point.velocities = [v * velocity_scale for v in point.velocities]
            if point.accelerations:
                point.accelerations = [a * velocity_scale * velocity_scale for a in point.accelerations]
            # Scale time (inverse of velocity scaling) - be careful with nanosec overflow
            total_time_ns = point.time_from_start.sec * 1e9 + point.time_from_start.nanosec
            scaled_time_ns = int(total_time_ns / velocity_scale)
            point.time_from_start.sec = int(scaled_time_ns // 1e9)
            point.time_from_start.nanosec = int(scaled_time_ns % 1e9)
        
        print(f"Executing trajectory with {len(goal.trajectory.points)} points at {velocity_scale*100:.1f}% speed...")
        
        # Send goal
        print("Sending trajectory goal...")
        future = self.trajectory_action_client.send_goal_async(goal)
        
        # Wait for goal acceptance
        print("Waiting for goal acceptance...")
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Trajectory goal rejected")
            return False
            
        print("Goal accepted, executing trajectory...")
        
        # Wait for completion
        result_future = goal_handle.get_result_async()
        completed = rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        
        if not completed:
            print(f"Trajectory execution timed out after {timeout} seconds")
            # Cancel the goal
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            return False
        
        if not result_future.done():
            print("Trajectory execution incomplete")
            return False
            
        result = result_future.result()
        if result and result.result.error_code == 0:
            print("Trajectory execution completed successfully!")
            return True
        else:
            error_code = result.result.error_code if result else "unknown"
            print(f"Trajectory execution failed with error: {error_code}")
            return False

    def set_gripper(self, value):
        """Set gripper position"""
        msg = UInt8()
        msg.data = value
        self.gripper_pub.publish(msg)

    def execute_waypoints(self, waypoints: List[dict], eef_step=0.005, min_fraction=0.8, timeout=30.0, velocity_scale=0.1):
        """Execute a sequence of Cartesian waypoints with individual gripper control"""
        print(f"Executing {len(waypoints)} waypoints individually...")
        
        for i, waypoint in enumerate(waypoints):
            print(f"\n--- Waypoint {i+1}: {waypoint.get('name', f'waypoint_{i+1}')} ---")
            
            # Plan path to single waypoint
            single_waypoint = [waypoint]
            trajectory, fraction = self.plan_cartesian_path(single_waypoint, eef_step, min_fraction)
            
            if not trajectory:
                print(f"Failed to plan path to waypoint {i+1}")
                return False
                
            print(f"Planned trajectory covers {fraction*100:.2f}% of path")
            if fraction < min_fraction:
                print(f"Path coverage {fraction*100:.2f}% is below minimum required {min_fraction*100:.2f}%")
                return False
            
            # Execute trajectory to this waypoint
            success = self.execute_trajectory(trajectory, timeout, velocity_scale)
            if not success:
                print(f"Failed to execute trajectory to waypoint {i+1}")
                return False
            
            # Handle gripper control for this waypoint
            if 'gripper' in waypoint:
                print(f"Setting gripper to {waypoint['gripper']}")
                self.set_gripper(waypoint['gripper'])
                time.sleep(0.5)  # Allow gripper time to move
            
            # Handle wait time for this waypoint
            if 'wait_time' in waypoint and waypoint['wait_time'] > 0:
                print(f"Waiting {waypoint['wait_time']} seconds...")
                time.sleep(waypoint['wait_time'])
        
        print("\nAll waypoints executed successfully!")
        return True

def update_dynamic_coordinates(waypoints_data, target_x, target_y):
    """Update X,Y coordinates for waypoints starting with 'xy1'"""
    updated_count = 0
    for waypoint in waypoints_data.get('waypoints', []):
        if waypoint.get('name', '').startswith('xy1'):
            # Update X and Y coordinates (first two elements of pose array)
            old_x, old_y = waypoint['pose'][0], waypoint['pose'][1]
            waypoint['pose'][0] = target_x
            waypoint['pose'][1] = target_y
            updated_count += 1
            print(f"  Updated {waypoint['name']}: ({old_x:.4f}, {old_y:.4f}) → ({target_x:.4f}, {target_y:.4f})")
    
    return updated_count

def get_fresh_coordinates():
    """Get fresh coordinates from get_coordinates.py"""
    try:
        import sys
        import os
        sys.path.append(os.path.dirname(os.path.abspath(__file__)))
        from get_coordinates import get_target_coordinates
        
        target_x, target_y = get_target_coordinates()
        print(f"Fresh coordinates from get_coordinates.py: X={target_x:.4f}, Y={target_y:.4f}")
        return target_x, target_y
    except ImportError:
        print("ERROR: Could not import get_coordinates.py")
        return None, None
    except Exception as e:
        print(f"ERROR: Failed to get coordinates: {e}")
        return None, None

def extract_tasks_from_waypoints(waypoints_data):
    """Extract available tasks from waypoint data"""
    tasks = set()
    for waypoint in waypoints_data.get('waypoints', []):
        name = waypoint.get('name', '')
        # Look for task patterns like 'clip1_', 'clip2_', etc.
        match = re.match(r'^(clip\d+)_', name)
        if match:
            tasks.add(match.group(1))
    
    return sorted(list(tasks))

def filter_waypoints_by_task(waypoints_data, task_name):
    """Filter waypoints to only include those belonging to a specific task"""
    filtered_waypoints = []
    
    # Add common waypoints (home, xy1, etc.) and task-specific waypoints
    for waypoint in waypoints_data.get('waypoints', []):
        name = waypoint.get('name', '')
        
        # Include waypoints that:
        # 1. Start with the task name (e.g., 'clip1_')
        # 2. Are common waypoints (home, xy1, etc.)
        if (name.startswith(f'{task_name}_') or 
            name in ['home', 'xy1', 'xy1_down', 'xy1_up'] or
            name.startswith('xy1')):
            filtered_waypoints.append(waypoint)
    
    return filtered_waypoints

def display_task_waypoints(waypoints, task_name):
    """Display waypoints for a specific task"""
    print(f"\n=== Task: {task_name.upper()} ===")
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
    
    args = parser.parse_args()
    
    rclpy.init()
    node = MultiTaskMoveIt()
    
    if not node.wait_for_services():
        print("Failed to connect to services")
        rclpy.shutdown()
        return
    
    # Load base waypoint file
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
    
    # Extract available tasks
    available_tasks = extract_tasks_from_waypoints(base_waypoints_data)
    
    if not available_tasks:
        print("No tasks found in waypoint file!")
        rclpy.shutdown()
        return
    
    print(f"\nAvailable tasks in {args.file}: {', '.join(available_tasks)}")
    
    # Main execution loop
    try:
        while True:
            print(f"\n{'='*50}")
            print("MULTI-TASK WAYPOINT CONTROLLER")
            print(f"{'='*50}")
            print(f"Available tasks: {', '.join(available_tasks)}")
            print("Commands: <task_name> | 'list' | 'quit'")
            
            user_input = input("\nSelect task to execute: ").strip().lower()
            
            if user_input == 'quit' or user_input == 'q':
                print("Exiting multi-task controller...")
                break
            elif user_input == 'list' or user_input == 'l':
                for task in available_tasks:
                    task_waypoints = filter_waypoints_by_task(base_waypoints_data, task)
                    display_task_waypoints(task_waypoints, task)
                continue
            elif user_input not in available_tasks:
                print(f"Invalid task '{user_input}'. Available: {', '.join(available_tasks)}")
                continue
            
            selected_task = user_input
            print(f"\n--- Preparing Task: {selected_task.upper()} ---")
            
            # Get fresh coordinates before each task
            target_x, target_y = get_fresh_coordinates()
            if target_x is None or target_y is None:
                print("Failed to get coordinates, skipping task")
                continue
            
            # Create working copy of waypoints data and update coordinates
            working_data = json.loads(json.dumps(base_waypoints_data))  # Deep copy
            updated_count = update_dynamic_coordinates(working_data, target_x, target_y)
            
            if updated_count > 0:
                print(f"Updated {updated_count} waypoints with fresh coordinates")
            
            # Filter waypoints for selected task
            task_waypoints = filter_waypoints_by_task(working_data, selected_task)
            
            if not task_waypoints:
                print(f"No waypoints found for task '{selected_task}'")
                continue
            
            # Display task details
            display_task_waypoints(task_waypoints, selected_task)
            
            # Get user confirmation
            print(f"\nReady to execute {len(task_waypoints)} waypoints for task '{selected_task}'")
            confirm = input("Proceed with execution? (y/N): ").strip().lower()
            
            if confirm != 'y':
                print("Task execution cancelled")
                continue
            
            print(f"\n🚀 Starting execution of task: {selected_task.upper()}")
            print(f"Using coordinates: X={target_x:.4f}, Y={target_y:.4f}")
            print(f"Z-offset applied: {node.GLOBAL_Z_OFFSET*100:.0f}cm")
            
            # Execute the task
            success = node.execute_waypoints(task_waypoints, 
                                           eef_step=args.eef_step, 
                                           min_fraction=args.min_fraction,
                                           timeout=args.timeout,
                                           velocity_scale=args.velocity_scale)
            
            if success:
                print(f"✅ Task '{selected_task}' completed successfully!")
            else:
                print(f"❌ Task '{selected_task}' execution failed!")
                
            print("\nReady for next task...")
            
    except KeyboardInterrupt:
        print("\n\nInterrupted by user. Exiting...")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()