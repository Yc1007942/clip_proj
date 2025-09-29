
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
    GLOBAL_Z_OFFSET = 0
    
    def __init__(self):
        super().__init__('service_based_moveit')
        self.cartesian_path_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.motion_plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, 
                                              '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.gripper_pub = self.create_publisher(UInt8, '/lebai_gripper/cmd/position', 10)
        self.current_joint_state = None
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    def joint_state_cb(self, msg):
        self.current_joint_state = msg

    def wait_for_services(self, timeout=10.0):
        
        if not self.cartesian_path_client.wait_for_service(timeout_sec=timeout):
        
            return False
            
        if not self.motion_plan_client.wait_for_service(timeout_sec=timeout):
            return False
            
        if not self.trajectory_client.wait_for_server(timeout_sec=timeout):
            return False
            
        start = time.time()
        while self.current_joint_state is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if self.current_joint_state is None:
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

    def plan_cartesian_path(self, waypoints, eef_step=0.01, min_fraction=0.8):
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
            print("WARNING: No current joint state ")
        
        future = self.cartesian_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.error_code.val == MoveItErrorCodes.SUCCESS:
                return response.solution, response.fraction
            else:
                print(f"Failed: {response.error_code.val}")
                return None, 0.0
        else:
            return None, 0.0

    def scale_trajectory_velocity(self, trajectory, velocity_scale=0.1):
        if not trajectory.joint_trajectory.points:
            return trajectory
        for point in trajectory.joint_trajectory.points:
            # Scale velocities if they exist
            if point.velocities:
                point.velocities = [v * velocity_scale for v in point.velocities]
        
            if point.accelerations:
                point.accelerations = [a * velocity_scale * velocity_scale for a in point.accelerations]
        
            total_time_ns = point.time_from_start.sec * 1e9 + point.time_from_start.nanosec
            scaled_time_ns = int(total_time_ns / velocity_scale)
            point.time_from_start.sec = int(scaled_time_ns // 1e9)
            point.time_from_start.nanosec = int(scaled_time_ns % 1e9)
            
        return trajectory

    def execute_trajectory(self, trajectory, timeout=30.0, velocity_scale=0.1):
        if not trajectory.joint_trajectory.points:
            print("Empty trajectory, nothing to execute")
            return False
            
        trajectory = self.scale_trajectory_velocity(trajectory, velocity_scale)
        print(f"Executing trajectory with {len(trajectory.joint_trajectory.points)} points at {velocity_scale*100}% speed...")
        
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            return False
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory
        
        goal_future = self.trajectory_client.send_goal_async(goal)
        
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
                
        
            result_future = goal_handle.get_result_async()
            start_time = time.time()
            while not result_future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
                if time.time() - start_time > timeout:
                    print(f"Timeout{timeout}")
                    goal_handle.cancel_goal_async()
                    return False
            
            result = result_future.result()
            if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                return True
            else:
                return False
                
        except KeyboardInterrupt:
            return False
        except Exception as e:
            return False

    def set_gripper(self, value):
        msg = UInt8()
        msg.data = max(0, min(100, value))
        self.gripper_pub.publish(msg)
        time.sleep(0.3)

    def execute_waypoints(self, waypoints: List[dict], eef_step=0.005, min_fraction=0.8, timeout=30.0, velocity_scale=0.1):
        for i, waypoint in enumerate(waypoints):
            single_waypoint = [waypoint]
            trajectory, fraction = self.plan_cartesian_path(single_waypoint, eef_step, min_fraction)
            
            if not trajectory:
                return False
                
            print(f"Planned trajectory covers {fraction*100:.2f}% of path")
            if fraction < min_fraction:
                print(f"Path coverage {fraction*100:.2f}% below {min_fraction*100:.2f}%")
                return False
            
            success = self.execute_trajectory(trajectory, timeout, velocity_scale)
            if not success:
                return False
            
            if 'gripper' in waypoint:
                self.set_gripper(waypoint['gripper'])
                time.sleep(0.5) 
            
            if 'wait_time' in waypoint and waypoint['wait_time'] > 0:
                time.sleep(waypoint['wait_time'])
        return True

def update_dynamic_coordinates(waypoints_data, target_x, target_y):
    dynamic_waypoint_names = ["get_x_y", "x_y_down", "x_y_close_half", "x_y_closed", "x_y_up", "x_y_up_2"]
    
    updated_count = 0
    for waypoint in waypoints_data.get('waypoints', []):
        if waypoint.get('name') in dynamic_waypoint_names:
            old_x, old_y = waypoint['pose'][0], waypoint['pose'][1]
            waypoint['pose'][0] = target_x
            waypoint['pose'][1] = target_y
            updated_count += 1
            print(f"  Updated {waypoint['name']}: ({old_x:.4f}, {old_y:.4f}) → ({target_x:.4f}, {target_y:.4f})")
    
    return updated_count

def load_and_update_waypoints(filename):
    try:
        if filename.startswith('sete') or filename.startswith('setf') or filename.startswith('seth'): #------------------------include sets
            import sys
            import os
            sys.path.append(os.path.dirname(os.path.abspath(__file__)))
            from get_coordinates import get_target_coordinates
            
            target_x, target_y = get_target_coordinates()
            print(f"Dynamic coordinates from get_coordinates.py: X={target_x:.4f}, Y={target_y:.4f}")
            with open(filename, 'r') as f:
                waypoints_data = json.load(f)
            updated_count = update_dynamic_coordinates(waypoints_data, target_x, target_y)
            
            if updated_count > 0:
                for wp in waypoints_data.get('waypoints', []):
                    if wp.get('name') in ["get_x_y", "x_y_down", "x_y_close_half", "x_y_closed", "x_y_up"]:
                        print(f"  {wp['name']}: X={wp['pose'][0]:.4f}, Y={wp['pose'][1]:.4f}, Z={wp['pose'][2]:.4f}")
                
                response = input("\nProceed with these coordinates? (y/N): ").strip().lower()
                if response != 'y':
                    print("Execution cancelled by user")
                    return None
            
            return waypoints_data
        else:
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
        
    data = load_and_update_waypoints(args.file)
    if data is None:
        rclpy.shutdown()
        return
        
    waypoints = data.get('waypoints', [])
    if len(waypoints) < 2:
        print("Need at least 2 waypoints")
        rclpy.shutdown()
        return
       
    for i, wp in enumerate(waypoints):
        original_z = wp['pose'][2]
        adjusted_z = original_z + node.GLOBAL_Z_OFFSET
        print(f"  Waypoint {i+1}: Z {original_z:.3f}m → {adjusted_z:.3f}m")
        
    success = node.execute_waypoints(waypoints, 
                                    eef_step=args.eef_step, 
                                    min_fraction=args.min_fraction,
                                    timeout=args.timeout,
                                    velocity_scale=args.velocity_scale)
    
    if success:
        print("Waypoint execution completed")
    else:
        print("Waypoint execution failed")
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()
