#!/usr/bin/env python3
"""
# Custom parameters
python3 waypoint_service_moveit.py --file setd.json --eef-step 0.01 --min-fraction 0.8 --timeout 45
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import json
import time
import math
import argparse
from typing import List, Optional

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan
from moveit_msgs.msg import RobotTrajectory, MoveItErrorCodes, MotionPlanRequest, PlanningOptions
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8, Header
from trajectory_msgs.msg import JointTrajectory

def rotvec_to_quat(rx, ry, rz):
    angle = math.sqrt(rx*rx + ry*ry + rz*rz)
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    ax, ay, az = rx/angle, ry/angle, rz/angle
    s = math.sin(angle/2.0)
    return (ax*s, ay*s, az*s, math.cos(angle/2.0))

class ServiceBasedMoveIt(Node):
    # Global Z-offset configuration (in meters)
    GLOBAL_Z_OFFSET = 0  # 20cm upward offset
    
    def __init__(self):
        super().__init__('service_based_moveit')
        
        # Service clients
        self.cartesian_path_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.motion_plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        # Action client for trajectory execution
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, 
                                              '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        
        # Subscriptions
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        
        # Publishers
        self.gripper_pub = self.create_publisher(UInt8, '/lebai_gripper/cmd/position', 10)
        
        self.current_joint_state = None
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        print("[ServiceMoveIt] Initialized")

    def joint_state_cb(self, msg):
        self.current_joint_state = msg

    def wait_for_services(self, timeout=10.0):
        print("Waiting for MoveIt services...")
        
        if not self.cartesian_path_client.wait_for_service(timeout_sec=timeout):
            print("Cartesian path service not available")
            return False
            
        if not self.motion_plan_client.wait_for_service(timeout_sec=timeout):
            print("Motion planning service not available") 
            return False
            
        if not self.trajectory_client.wait_for_server(timeout_sec=timeout):
            print("Trajectory action server not available")
            return False
            
        start = time.time()
        while self.current_joint_state is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if self.current_joint_state is None:
            print("No joint state received")
            return False
            
        print("All services ready!")
        return True

    def create_pose_from_array(self, pose_array):
        """Convert [x,y,z,rx,ry,rz] to geometry_msgs/Pose with 20cm Z-offset"""
        x, y, z, rx, ry, rz = pose_array
        qx, qy, qz, qw = rotvec_to_quat(rx, ry, rz)
        
        # Apply global Z-offset
        z_offset = self.GLOBAL_Z_OFFSET  # 20cm upward offset
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y  
        pose.position.z = z + z_offset
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def plan_cartesian_path(self, waypoints, eef_step=0.01, min_fraction=0.8):
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

    def scale_trajectory_velocity(self, trajectory, velocity_scale=0.1):
        """Scale the velocity and timing of a trajectory"""
        if not trajectory.joint_trajectory.points:
            return trajectory
            
        # Scale all velocities and timing
        for point in trajectory.joint_trajectory.points:
            # Scale velocities if they exist
            if point.velocities:
                point.velocities = [v * velocity_scale for v in point.velocities]
            # Scale accelerations if they exist  
            if point.accelerations:
                point.accelerations = [a * velocity_scale * velocity_scale for a in point.accelerations]
            # Scale time (inverse of velocity scaling) - be careful with nanosec overflow
            total_time_ns = point.time_from_start.sec * 1e9 + point.time_from_start.nanosec
            scaled_time_ns = int(total_time_ns / velocity_scale)
            point.time_from_start.sec = int(scaled_time_ns // 1e9)
            point.time_from_start.nanosec = int(scaled_time_ns % 1e9)
            
        return trajectory

    def execute_trajectory(self, trajectory, timeout=30.0, velocity_scale=0.1):
        """Execute a trajectory using the joint trajectory controller"""
        if not trajectory.joint_trajectory.points:
            print("Empty trajectory, nothing to execute")
            return False
            
        # Scale trajectory velocity for safety
        trajectory = self.scale_trajectory_velocity(trajectory, velocity_scale)
        print(f"Executing trajectory with {len(trajectory.joint_trajectory.points)} points at {velocity_scale*100}% speed...")
        
        # Wait for action server
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            print("Trajectory action server not available")
            return False
        
        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory
        
        # Send goal
        print("Sending trajectory goal...")
        goal_future = self.trajectory_client.send_goal_async(goal)
        
        # Wait for goal response with timeout
        print("Waiting for goal acceptance...")
        try:
            start_time = time.time()
            while not goal_future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
                if time.time() - start_time > 5.0:  # 5 second timeout for goal acceptance
                    print("Timeout waiting for goal acceptance")
                    return False
            
            goal_handle = goal_future.result()
            if not goal_handle.accepted:
                print("Goal rejected by action server")
                return False
                
            print("Goal accepted, executing trajectory...")
            
            # Wait for execution result with timeout
            result_future = goal_handle.get_result_async()
            start_time = time.time()
            while not result_future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
                if time.time() - start_time > timeout:
                    print(f"Timeout after {timeout} seconds, canceling trajectory")
                    goal_handle.cancel_goal_async()
                    return False
            
            result = result_future.result()
            if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                print("Trajectory execution completed successfully!")
                return True
            else:
                print(f"Trajectory execution failed with error code: {result.result.error_code}")
                return False
                
        except KeyboardInterrupt:
            print("\nExecution interrupted by user")
            return False
        except Exception as e:
            print(f"Error during trajectory execution: {e}")
            return False

    def set_gripper(self, value):
        """Set gripper position"""
        msg = UInt8()
        msg.data = max(0, min(100, value))
        self.gripper_pub.publish(msg)
        time.sleep(0.3)

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
    """Update X,Y coordinates for specific waypoints that need dynamic positioning"""
    # List of waypoint names that should use dynamic coordinates
    dynamic_waypoint_names = ["get_x_y", "x_y_down", "x_y_close_half", "x_y_closed", "x_y_up", "x_y_up_2"]
    
    updated_count = 0
    for waypoint in waypoints_data.get('waypoints', []):
        if waypoint.get('name') in dynamic_waypoint_names:
            # Update X and Y coordinates (first two elements of pose array)
            old_x, old_y = waypoint['pose'][0], waypoint['pose'][1]
            waypoint['pose'][0] = target_x
            waypoint['pose'][1] = target_y
            updated_count += 1
            print(f"  Updated {waypoint['name']}: ({old_x:.4f}, {old_y:.4f}) → ({target_x:.4f}, {target_y:.4f})")
    
    return updated_count

def load_and_update_waypoints(filename):
    """Load waypoint file and update dynamic coordinates if needed"""
    try:
        # Check if this is a file that needs dynamic coordinate updates
        if filename.startswith('sete') or filename.startswith('setf'):
            # Import and get dynamic coordinates
            import sys
            import os
            sys.path.append(os.path.dirname(os.path.abspath(__file__)))
            from get_coordinates import get_target_coordinates
            
            target_x, target_y = get_target_coordinates()
            print(f"Dynamic coordinates from get_coordinates.py: X={target_x:.4f}, Y={target_y:.4f}")
            
            # Load the original file
            with open(filename, 'r') as f:
                waypoints_data = json.load(f)
            
            # Update dynamic coordinates
            updated_count = update_dynamic_coordinates(waypoints_data, target_x, target_y)
            
            if updated_count > 0:
                print(f"Updated {updated_count} waypoints with dynamic coordinates")
                # Ask for confirmation before proceeding
                print("\nUpdated waypoint coordinates:")
                for wp in waypoints_data.get('waypoints', []):
                    if wp.get('name') in ["get_x_y", "x_y_down", "x_y_close_half", "x_y_closed", "x_y_up"]:
                        print(f"  {wp['name']}: X={wp['pose'][0]:.4f}, Y={wp['pose'][1]:.4f}, Z={wp['pose'][2]:.4f}")
                
                response = input("\nProceed with these coordinates? (y/N): ").strip().lower()
                if response != 'y':
                    print("Execution cancelled by user")
                    return None
            
            return waypoints_data
        else:
            # For non-dynamic files, load normally
            with open(filename, 'r') as f:
                return json.load(f)
                
    except ImportError:
        print("ERROR: Could not import get_coordinates.py")
        return None
    except FileNotFoundError:
        print(f"ERROR: File {filename} not found")
        return None
    except Exception as e:
        print(f"ERROR: Failed to load waypoints: {e}")
        return None

def main():
    parser = argparse.ArgumentParser(description='Execute waypoints using MoveIt services')
    parser.add_argument('--file', '-f', default='setz.json', 
                        help='Waypoint JSON file to execute (default: setd.json)')
    parser.add_argument('--eef-step', type=float, default=0.005,
                        help='End-effector step size for Cartesian planning (default: 0.005)')
    parser.add_argument('--min-fraction', type=float, default=0.05,
                        help='Minimum fraction of path that must be planned (default: 0.7)')
    parser.add_argument('--timeout', type=int, default=30,
                        help='Execution timeout in seconds (default: 30)')
    parser.add_argument('--velocity-scale', type=float, default=0.3,
                        help='Velocity scaling factor for safe execution (default: 0.1)')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = ServiceBasedMoveIt()
    
    if not node.wait_for_services():
        print("Failed to connect to services")
        rclpy.shutdown()
        return
        
    # Load and execute waypoints from specified file with dynamic coordinate updates
    data = load_and_update_waypoints(args.file)
    if data is None:
        rclpy.shutdown()
        return
        
    waypoints = data.get('waypoints', [])
    if len(waypoints) < 2:
        print("Need at least 2 waypoints")
        rclpy.shutdown()
        return
        
    print(f"Loaded {len(waypoints)} waypoints with {node.GLOBAL_Z_OFFSET*100:.0f}cm Z-offset applied")
    for i, wp in enumerate(waypoints):
        original_z = wp['pose'][2]
        adjusted_z = original_z + node.GLOBAL_Z_OFFSET
        print(f"  Waypoint {i+1}: Z {original_z:.3f}m → {adjusted_z:.3f}m")
        
    print(f"Executing waypoints from {args.file}")
    success = node.execute_waypoints(waypoints, 
                                    eef_step=args.eef_step, 
                                    min_fraction=args.min_fraction,
                                    timeout=args.timeout,
                                    velocity_scale=args.velocity_scale)
    
    if success:
        print("Waypoint execution completed successfully!")
    else:
        print("Waypoint execution failed")
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()