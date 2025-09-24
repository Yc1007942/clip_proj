
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
    GLOBAL_Z_OFFSET = 0  # No Z-offset by default
    
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
        
        # Failure handling state
        self.force_failure_detected = False
        self.current_goal_handle = None
        
        self.current_joint_state = None
        
        # Task execution counter for height adjustment
        self.task_counter = 0
        self.initial_height = 0.32  # Initial height for xy1_up and xy1_up_2
        self.height_decrement = 0.0235  # Height reduction per cycle
        self.minimum_height = 0.28  # Minimum allowed height
        
        print("[MultiTaskMoveIt] Initialized")
        print(f"Height adjustment: Start={self.initial_height}m, Decrement={self.height_decrement}m, Minimum={self.minimum_height}m")
        print(f"Force monitoring enabled with thresholds:")
        print(f"  Force: X={self.force_threshold_x}N, Y={self.force_threshold_y}N, Z={self.force_threshold_z}N")
        print(f"  Torque: X={self.torque_threshold_x}Nm, Y={self.torque_threshold_y}Nm, Z={self.torque_threshold_z}Nm")
        print(f"  Resultant force limit: {self.force_resultant_threshold}N")
        print(f"  Consecutive violations required: {self.consecutive_violations_required}")

    def get_current_height(self):
        """Calculate current height based on task counter"""
        current_height = self.initial_height - (self.task_counter * self.height_decrement)
        # Ensure height doesn't go below minimum
        return max(current_height, self.minimum_height)
    
    def increment_task_counter(self):
        """Increment task counter and log height changes"""
        self.task_counter += 1
        current_height = self.get_current_height()
        print(f"Task counter incremented to {self.task_counter}")
        print(f"Next cycle height will be: {current_height:.3f}m")
        if current_height <= self.minimum_height:
            print(f"WARNING: Minimum height ({self.minimum_height}m) reached!")
        return current_height

    def joint_state_callback(self, msg):
        """Store current joint state for planning"""
        self.current_joint_state = msg

    def wrench_callback(self, msg):
        """Process force/torque sensor data and check for violations"""
        self.current_wrench = msg
        
        # Add to history for moving average
        self.wrench_history.append(msg)
        if len(self.wrench_history) > self.history_size:
            self.wrench_history.pop(0)
        
        # Check for force violations if monitoring is enabled
        if self.force_monitoring_enabled and len(self.wrench_history) >= self.consecutive_violations_required:
            self.check_force_violations()

    def calculate_average_wrench(self):
        """Calculate moving average of recent wrench readings"""
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
        """Check if current force readings exceed thresholds"""
        avg_wrench = self.calculate_average_wrench()
        if not avg_wrench:
            return False
            
        force = avg_wrench['force']
        torque = avg_wrench['torque']
        
        # Calculate force resultant magnitude
        force_magnitude = math.sqrt(force['x']**2 + force['y']**2 + force['z']**2)
        
        # Check individual axis thresholds
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
        
        # Additional failure triggers can be added here

        if violations:
            self.consecutive_violations_count += 1
            print(f"Force violation {self.consecutive_violations_count}/{self.consecutive_violations_required}: {', '.join(violations)}")
            
            if self.consecutive_violations_count >= self.consecutive_violations_required:
                self.trigger_force_failure(violations, force_magnitude)
                return True
        else:
            # Reset violation count if no violations detected
            self.consecutive_violations_count = 0
            
        return False

    def trigger_force_failure(self, violations, force_magnitude):
        if self.force_failure_detected:
            return  
            
        self.force_failure_detected = True
        
        print(f"\n FORCE FAILURE DETECTED! ")
        print(f"Force magnitude: {force_magnitude:.2f}N")
        print(f"Violations: {', '.join(violations)}")
        print("Initiating emergency stop and recovery...")
        
        # Cancel current trajectory if one is executing
        if self.current_goal_handle and not self.current_goal_handle.get_result_async().done():
            print("Cancelling current trajectory...")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
        
        # Wait a moment for trajectory to stop
        time.sleep(0.5)
        
        # Open gripper immediately
        print("Opening gripper to 100...")
        self.set_gripper(100)
        time.sleep(1.0)  # Give gripper time to open
        
        print("Force failure handling completed. Ready for recovery sequence.")

    def execute_failure_recovery(self, waypoints_data, test_mode=False):
        """Execute recovery sequence - move to failure_home waypoint"""
        try:
            # Find failure_home waypoint
            failure_home_waypoint = None
            for waypoint in waypoints_data.get('waypoints', []):
                if waypoint.get('name') == 'failure_home':
                    failure_home_waypoint = waypoint
                    break
            
            if not failure_home_waypoint:
                print("ERROR: failure_home waypoint not found in waypoint data!")
                return False
            
            print(f"Executing recovery to failure_home: {failure_home_waypoint['pose']}")
            
            # Execute movement to failure_home with slower speed for safety
            # Use direct trajectory execution to avoid recursion
            single_waypoint = [failure_home_waypoint]
            trajectory, fraction = self.plan_cartesian_path(single_waypoint, 0.01, 0.1)
            
            if trajectory and fraction > 0.1:
                print(f"Recovery path planned with {fraction*100:.1f}% coverage")
                # Temporarily clear failure flag and disable force monitoring during recovery
                original_monitoring = self.force_monitoring_enabled
                original_failure_state = self.force_failure_detected
                self.force_monitoring_enabled = False
                self.force_failure_detected = False  # Clear flag so execute_trajectory can run
                print(" Temporarily cleared failure state for recovery movement")
                
                recovery_success = self.execute_trajectory(trajectory, 60.0, 0.1, test_mode=test_mode)
                
                # Restore original states after recovery attempt
                self.force_monitoring_enabled = original_monitoring
                self.force_failure_detected = original_failure_state
                print(" Restored original failure state after recovery")
            else:
                print("Failed to plan recovery path to failure_home")
                recovery_success = False
            
            if recovery_success:
                print(" Recovery to failure_home completed successfully")
                return True
            else:
                print(" Recovery to failure_home failed!")
                return False
                
        except Exception as e:
            print(f"ERROR during failure recovery: {e}")
            return False
        finally:
            # Reset consecutive violations count but keep failure state for user acknowledgment
            self.consecutive_violations_count = 0

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

    def execute_trajectory(self, trajectory: RobotTrajectory, timeout=120.0, velocity_scale=0.1, test_mode=False):
        """Execute a planned trajectory"""
        if not trajectory or not trajectory.joint_trajectory.points:
            print("No valid trajectory to execute")
            return False
        
        if test_mode:
            print(f" TEST MODE: Simulating trajectory execution with {len(trajectory.joint_trajectory.points)} points")
            print(f" TEST MODE: Would execute at {velocity_scale*100:.1f}% speed for ~{timeout}s max")
            time.sleep(1.0)  # Simulate execution time
            print(" TEST MODE: Trajectory execution simulation completed successfully!")
            return True
        
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
            
        # Store goal handle for potential cancellation
        self.current_goal_handle = goal_handle
        print("Goal accepted, executing trajectory...")
        
        # Wait for completion using manual loop with force monitoring
        result_future = goal_handle.get_result_async()
        start_time = time.time()
        
        while not result_future.done():
            # Process callbacks (including force sensor data)
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.time() - start_time
            
            # Check for force failure during execution
            if self.force_failure_detected:
                print(" Force failure detected during trajectory execution!")
                print("Trajectory will be cancelled...")
                return False  # Force failure handling already done in trigger_force_failure
            
            if elapsed > timeout:
                print(f"Trajectory execution timed out after {timeout} seconds")
                # Cancel the goal
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                return False
        
        # Clear goal handle when execution completes
        self.current_goal_handle = None
            
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

    def execute_waypoints(self, waypoints: List[dict], eef_step=0.005, min_fraction=0.8, timeout=120.0, velocity_scale=0.1, waypoints_data=None, test_mode=False):
        """Execute a sequence of Cartesian waypoints with individual gripper control and force monitoring"""
        print(f"Executing {len(waypoints)} waypoints individually...")
        if test_mode:
            print(" TEST MODE: Simulating waypoint execution without sending robot commands")
        
        for i, waypoint in enumerate(waypoints):
            print(f"\n--- Waypoint {i+1}: {waypoint.get('name', f'waypoint_{i+1}')} ---")
            
            # Check if we have a previous force failure that needs recovery
            if self.force_failure_detected and waypoints_data and not test_mode:
                print(" Force failure detected! Initiating recovery sequence...")
                recovery_success = self.execute_failure_recovery(waypoints_data, test_mode=test_mode)
                if recovery_success:
                    print("Recovery completed. Task execution aborted.")
                else:
                    print("Recovery failed! Manual intervention required.")
                return False
            
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
            
            # Execute trajectory to this waypoint (with force monitoring)
            success = self.execute_trajectory(trajectory, timeout, velocity_scale, test_mode=test_mode)
            
            # Check for force failure after trajectory execution (skip in test mode)
            if self.force_failure_detected and not test_mode:
                print(f" Force failure occurred during waypoint {i+1} execution!")
                if waypoints_data:
                    print("Initiating recovery sequence...")
                    recovery_success = self.execute_failure_recovery(waypoints_data, test_mode=test_mode)
                    if recovery_success:
                        print("Recovery completed. Task execution aborted.")
                    else:
                        print("Recovery failed! Manual intervention required.")
                return False
            
            if not success:
                print(f"Failed to execute trajectory to waypoint {i+1}")
                return False
            
            # Handle gripper control for this waypoint
            if 'gripper' in waypoint:
                if test_mode:
                    print(f" TEST MODE: Would set gripper to {waypoint['gripper']}")
                else:
                    print(f"Setting gripper to {waypoint['gripper']}")
                    self.set_gripper(waypoint['gripper'])
                time.sleep(0.1 if test_mode else 0.5)  # Shorter wait in test mode
            
            # Handle wait time for this waypoint
            if 'wait_time' in waypoint and waypoint['wait_time'] > 0:
                wait_time = 0.2 if test_mode else waypoint['wait_time']  # Shorter wait in test mode
                print(f"Waiting {wait_time} seconds..." + (" (test mode)" if test_mode else ""))
                time.sleep(wait_time)
        
        print("\nAll waypoints executed successfully!")
        return True

    def should_check_clip_status(self, waypoint_name):
        """Determine if clip status should be checked after this waypoint"""
        # Check status only after returning home from clip insertion
        check_after = [
            'clip1_home',      # After clip1 insertion and return to home
            'clip2_home'       # After clip2 insertion and return to home
        ]
        
        return waypoint_name in check_after

    def check_clip_status_after_waypoint(self, waypoint_name, task_name, test_mode=False):
        """Check clip status after executing a waypoint"""
        print(f"\n🔍 Checking clip status after waypoint: {waypoint_name}")
        
        if test_mode:
            # Simulate clip status in test mode with new response format
            import random
            simulated_response = {
                "overall_status": "cable_in_both" if random.random() > 0.3 else "cable_out_some",
                "clip_count": 2,
                "clip_results": [
                    {
                        "clip_id": 1,
                        "position": "right", 
                        "status": "cable_in" if random.random() > 0.3 else "cable_out",
                        "confidence": random.randint(75, 95),
                        "center_x": 640.5
                    },
                    {
                        "clip_id": 2,
                        "position": "left",
                        "status": "cable_in" if random.random() > 0.3 else "cable_out", 
                        "confidence": random.randint(75, 95),
                        "center_x": 240.3
                    }
                ],
                "processing_time": random.uniform(1.5, 3.0)
            }
            
            print(f" TEST MODE: Simulated clip analysis response")
            return simulated_response
        
        try:
            # Import the clip status function
            from get_multi_coordinates_withclip import get_clip_status
            
            # Extract clip number from waypoint name (e.g., 'clip1_home' -> 1)
            if 'clip1' in waypoint_name:
                clip_number = 1
            elif 'clip2' in waypoint_name:
                clip_number = 2
            else:
                # Get status for all clips if can't determine specific clip
                clip_number = None
            
            # Get clip status from detection system
            response = get_clip_status(clip_number)
            
            if isinstance(response, dict) and 'clip_results' in response:
                # Handle new format response - return full analysis
                print(f"📊 Full clip analysis received:")
                print(f"    Overall: {response.get('overall_status', 'unknown')}")
                print(f"    Clips: {response.get('clip_count', 0)} detected")
                return response
            elif isinstance(response, str):
                # Handle legacy single clip response
                if response == 'in':
                    print(f"✅ Clip {clip_number} successfully placed and confirmed!")
                    return 'in'
                elif response == 'out':
                    print(f"❌ Clip {clip_number} placement failed - clip not properly inserted!")
                    return 'out'
                else:
                    print(f"⚠️  Could not determine clip {clip_number} status: {response}")
                    return 'unknown'
            else:
                print(f"⚠️  Invalid response format from detection system")
                return None
                
        except ImportError:
            print("WARNING: Could not import clip status checking - continuing without status")
            return None
        except Exception as e:
            print(f"ERROR checking clip status: {e}")
            return None

    def execute_waypoints(self, waypoints: List[dict], eef_step=0.005, min_fraction=0.8, timeout=120.0, velocity_scale=0.1, waypoints_data=None, test_mode=False, task_name=None):
        """Execute a sequence of Cartesian waypoints with individual gripper control, force monitoring, and clip status checking"""
        print(f"Executing {len(waypoints)} waypoints individually...")
        if test_mode:
            print(" TEST MODE: Simulating waypoint execution without sending robot commands")
        
        clip_status_results = []  # Track clip status for each attempt
        
        for i, waypoint in enumerate(waypoints):
            waypoint_name = waypoint.get('name', f'waypoint_{i+1}')
            print(f"\n--- Waypoint {i+1}: {waypoint_name} ---")
            
            # Check if we have a previous force failure that needs recovery
            if self.force_failure_detected and waypoints_data and not test_mode:
                print(" Force failure detected! Initiating recovery sequence...")
                recovery_success = self.execute_failure_recovery(waypoints_data, test_mode=test_mode)
                if recovery_success:
                    print("Recovery completed. Task execution aborted.")
                else:
                    print("Recovery failed! Manual intervention required.")
                return False, clip_status_results
            
            # Plan path to single waypoint
            single_waypoint = [waypoint]
            trajectory, fraction = self.plan_cartesian_path(single_waypoint, eef_step, min_fraction)
            
            if not trajectory:
                print(f"Failed to plan path to waypoint {i+1}")
                return False, clip_status_results
                
            print(f"Planned trajectory covers {fraction*100:.2f}% of path")
            if fraction < min_fraction:
                print(f"Path coverage {fraction*100:.2f}% is below minimum required {min_fraction*100:.2f}%")
                return False, clip_status_results
            
            # Execute trajectory to this waypoint (with force monitoring)
            success = self.execute_trajectory(trajectory, timeout, velocity_scale, test_mode=test_mode)
            
            # Check for force failure after trajectory execution (skip in test mode)
            if self.force_failure_detected and not test_mode:
                print(f" Force failure occurred during waypoint {i+1} execution!")
                if waypoints_data:
                    print("Initiating recovery sequence...")
                    recovery_success = self.execute_failure_recovery(waypoints_data, test_mode=test_mode)
                    if recovery_success:
                        print("Recovery completed. Task execution aborted.")
                    else:
                        print("Recovery failed! Manual intervention required.")
                return False, clip_status_results
            
            if not success:
                print(f"Failed to execute trajectory to waypoint {i+1}")
                return False, clip_status_results
            
            # Handle gripper control for this waypoint
            if 'gripper' in waypoint:
                if test_mode:
                    print(f" TEST MODE: Would set gripper to {waypoint['gripper']}")
                else:
                    print(f"Setting gripper to {waypoint['gripper']}")
                    self.set_gripper(waypoint['gripper'])
                time.sleep(0.1 if test_mode else 0.5)  # Shorter wait in test mode
            
            # Handle wait time for this waypoint
            if 'wait_time' in waypoint and waypoint['wait_time'] > 0:
                wait_time = 0.2 if test_mode else waypoint['wait_time']  # Shorter wait in test mode
                print(f"Waiting {wait_time} seconds..." + (" (test mode)" if test_mode else ""))
                time.sleep(wait_time)
            
            # Check for clip status after specific waypoints (typically after insertion attempts)
            if self.should_check_clip_status(waypoint_name) and task_name:
                clip_status = self.check_clip_status_after_waypoint(waypoint_name, task_name, test_mode)
                if clip_status:
                    clip_status_results.append({
                        'waypoint': waypoint_name,
                        'task': task_name,
                        'response': clip_status,
                        'timestamp': time.time()
                    })
        
        print("\nAll waypoints executed successfully!")
        return True, clip_status_results

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

def update_height_coordinates(waypoints_data, new_height):
    """Update Z coordinates for xy1_up and xy1_up_2 waypoints"""
    updated_count = 0
    target_waypoints = ['xy1_up', 'xy1_up_2']
    
    for waypoint in waypoints_data.get('waypoints', []):
        waypoint_name = waypoint.get('name', '')
        if waypoint_name in target_waypoints:
            # Update Z coordinate (third element of pose array)
            old_z = waypoint['pose'][2]
            waypoint['pose'][2] = new_height
            updated_count += 1
            print(f"  Updated {waypoint_name} height: {old_z:.3f}m → {new_height:.3f}m")
    
    return updated_count

def get_fresh_coordinates(test_mode=False):
    """Get fresh coordinates from get_coordinates.py or use test coordinates"""
    if test_mode:
        # Return default test coordinates
        print(" Using test coordinates for force failure testing")
        return 0.5, 0.3  # Safe test coordinates
        
    try:
        import sys
        import os
        sys.path.append(os.path.dirname(os.path.abspath(__file__)))
        from get_multi_coordinates_withclip import get_target_coordinates
        
        target_x, target_y = get_target_coordinates()
        print(f"Fresh coordinates from get_coordinates.py: X={target_x:.4f}, Y={target_y:.4f}")
        return target_x, target_y
    except ImportError:
        print("ERROR: Could not import get_coordinates.py")
        print("Falling back to test coordinates...")
        return 0.5, 0.3  # Fallback test coordinates
    except Exception as e:
        print(f"ERROR: Failed to get coordinates: {e}")
        print("Falling back to test coordinates...")
        return 0.5, 0.3  # Fallback test coordinates

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
    
    # Configure force thresholds from command line arguments
    node.force_threshold_x = args.force_threshold
    node.force_threshold_y = args.force_threshold
    node.force_threshold_z = args.force_threshold
    node.torque_threshold_x = args.torque_threshold
    node.torque_threshold_y = args.torque_threshold
    node.torque_threshold_z = args.torque_threshold
    node.force_resultant_threshold = args.force_resultant_threshold
    node.consecutive_violations_required = args.consecutive_violations
    
    print(f"Force monitoring configured:")
    print(f"  Force thresholds: {args.force_threshold}N per axis")
    print(f"  Torque thresholds: {args.torque_threshold}Nm per axis")
    print(f"  Resultant force limit: {args.force_resultant_threshold}N")
    print(f"  Consecutive violations required: {args.consecutive_violations}")
    
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
            print(f"Task cycles completed: {node.task_counter}")
            print(f"Current height setting: {node.get_current_height():.3f}m")
            
            # Force monitoring status
            if node.current_wrench is not None:
                wrench = node.current_wrench.wrench
                force_mag = math.sqrt(wrench.force.x**2 + wrench.force.y**2 + wrench.force.z**2)
                print(f"Force sensor:  Active (Current: {force_mag:.1f}N)")
            else:
                print(f"Force sensor:  No data (monitoring disabled)")
                node.force_monitoring_enabled = False
            
            if node.force_failure_detected:
                print(" FORCE FAILURE STATE - Recovery required!")
            
            print("Commands: <task_name> | 'list' | 'test' | 'test-task' | 'quit'")
            
            user_input = input("\nSelect task to execute: ").strip().lower()
            
            if user_input == 'quit' or user_input == 'q':
                print("Exiting multi-task controller...")
                break
            elif user_input == 'test' or user_input == 't':
                print("\n FORCE FAILURE TEST MODE")
                print("This will simulate a force failure and test the recovery sequence")
                confirm = input("Proceed with force failure test? (y/N): ").strip().lower()
                if confirm == 'y':
                    print(" Simulating force failure...")
                    # Simulate force failure
                    violations = ["Test Force X: 75.0N > 50.0N", "Test simulation"]
                    node.trigger_force_failure(violations, 75.0)
                    
                    # Test recovery
                    print(" Testing recovery sequence...")
                    recovery_success = node.execute_failure_recovery(base_waypoints_data, test_mode=True)
                    
                    if recovery_success:
                        print(" Force failure test completed - recovery successful!")
                    else:
                        print(" Force failure test failed - recovery unsuccessful!")
                        
                    # Prompt for acknowledgment
                    print(" Please acknowledge the test failure before continuing")
                    input("Press Enter to acknowledge and continue...")
                    node.force_failure_detected = False
                    print(" Test failure state cleared - ready for normal operation")
                else:
                    print("Force failure test cancelled")
                continue
            elif user_input == 'test-task':
                print("\n TASK EXECUTION TEST MODE")
                print(f"Available tasks: {', '.join(available_tasks)}")
                task_input = input("Enter task name to test: ").strip().lower()
                
                if task_input in available_tasks:
                    print(f" Testing task '{task_input}' with simulated robot execution...")
                    
                    # Get test coordinates
                    target_x, target_y = get_fresh_coordinates(test_mode=True)
                    
                    # Create working copy and update coordinates
                    working_data = json.loads(json.dumps(base_waypoints_data))
                    updated_count = update_dynamic_coordinates(working_data, target_x, target_y)
                    if updated_count > 0:
                        print(f"Updated {updated_count} waypoints with test coordinates")
                    
                    # Update height coordinates
                    current_height = node.get_current_height()
                    height_updated_count = update_height_coordinates(working_data, current_height)
                    if height_updated_count > 0:
                        print(f"Updated {height_updated_count} waypoints with cycle height: {current_height:.3f}m")
                    
                    # Filter waypoints for selected task
                    task_waypoints = filter_waypoints_by_task(working_data, task_input)
                    display_task_waypoints(task_waypoints, task_input)
                    
                    # Execute task in test mode
                    print(f"\n🧪 Executing {len(task_waypoints)} waypoints in TEST MODE...")
                    success, clip_status_results = node.execute_waypoints(task_waypoints, 
                                                   eef_step=args.eef_step, 
                                                   min_fraction=args.min_fraction,
                                                   timeout=args.timeout,
                                                   velocity_scale=args.velocity_scale,
                                                   waypoints_data=working_data,
                                                   test_mode=True,
                                                   task_name=task_input)
                    
                    if success:
                        print(f"✅ Test execution of task '{task_input}' completed successfully!")
                        
                        # Display simulated clip status results
                        if clip_status_results:
                            print(f"\n📋 Simulated Clip Status Results:")
                            for result in clip_status_results:
                                response = result['response']
                                waypoint = result['waypoint']
                                
                                if isinstance(response, dict) and 'clip_results' in response:
                                    print(f"  📊 Simulated analysis after {waypoint}:")
                                    print(f"      Overall: {response.get('overall_status', 'unknown')}")
                                    for clip in response.get('clip_results', []):
                                        clip_id = clip.get('clip_id', '?')
                                        status = clip.get('status', 'unknown')
                                        confidence = clip.get('confidence', 0)
                                        status_icon = "✅" if "in" in status else "❌"
                                        print(f"      {status_icon} Clip {clip_id}: {status} ({confidence}% confidence)")
                        
                        print("🧪 In real mode, this would increment the task counter")
                    else:
                        print(f" Test execution of task '{task_input}' failed!")
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
            print(f"\n--- Preparing Task: {selected_task.upper()} ---")
            
            # Get fresh coordinates before each task
            target_x, target_y = get_fresh_coordinates()
            if target_x is None or target_y is None:
                print("Failed to get coordinates, skipping task")
                continue
            
            # Create working copy of waypoints data and update coordinates
            working_data = json.loads(json.dumps(base_waypoints_data))  # Deep copy
            
            # Update X,Y coordinates
            updated_count = update_dynamic_coordinates(working_data, target_x, target_y)
            if updated_count > 0:
                print(f"Updated {updated_count} waypoints with fresh coordinates")
            
            # Update height coordinates based on task counter
            current_height = node.get_current_height()
            height_updated_count = update_height_coordinates(working_data, current_height)
            if height_updated_count > 0:
                print(f"Updated {height_updated_count} waypoints with cycle height: {current_height:.3f}m")
            
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
            
            print(f"\n Starting execution of task: {selected_task.upper()}")
            print(f"Task execution cycle: {node.task_counter + 1}")
            print(f"Using coordinates: X={target_x:.4f}, Y={target_y:.4f}")
            print(f"Current cycle height: {current_height:.3f}m")
            print(f"Z-offset applied: {node.GLOBAL_Z_OFFSET*100:.0f}cm")
            
            # Execute the task with force monitoring and clip status checking
            success, clip_status_results = node.execute_waypoints(task_waypoints, 
                                           eef_step=args.eef_step, 
                                           min_fraction=args.min_fraction,
                                           timeout=args.timeout,
                                           velocity_scale=args.velocity_scale,
                                           waypoints_data=working_data,
                                           task_name=selected_task)
            
            if success:
                print(f"✅ Task '{selected_task}' completed successfully!")
                
                # Display clip status results
                if clip_status_results:
                    print(f"\n📋 Clip Status Summary for {selected_task}:")
                    
                    for result in clip_status_results:
                        response = result['response']
                        waypoint = result['waypoint']
                        
                        # Handle new format response
                        if isinstance(response, dict) and 'clip_results' in response:
                            overall_status = response.get('overall_status', 'unknown')
                            clip_count = response.get('clip_count', 0)
                            processing_time = response.get('processing_time', 0)
                            
                            print(f"  📊 Analysis after {waypoint}:")
                            print(f"      Overall status: {overall_status}")
                            print(f"      Clips detected: {clip_count}")
                            print(f"      Processing time: {processing_time:.2f}s")
                            
                            # Display individual clip results
                            clip_results = response.get('clip_results', [])
                            successful_clips = []
                            failed_clips = []
                            
                            for clip in clip_results:
                                clip_id = clip.get('clip_id', 'unknown')
                                position = clip.get('position', 'unknown') 
                                status = clip.get('status', 'unknown')
                                confidence = clip.get('confidence', 0)
                                center_x = clip.get('center_x', 0)
                                
                                status_icon = "✅" if "in" in status else "❌" if "out" in status else "⚠️"
                                print(f"      {status_icon} Clip {clip_id} ({position}): {status} (confidence: {confidence}%, x: {center_x:.1f})")
                                
                                if "in" in status:
                                    successful_clips.append(clip_id)
                                elif "out" in status:
                                    failed_clips.append(clip_id)
                            
                            # Summary
                            print(f"\n📊 Final Results: {len(successful_clips)} successful, {len(failed_clips)} failed")
                            
                            if failed_clips:
                                print(f"\n⚠️  {len(failed_clips)} clip(s) failed to be placed properly:")
                                for clip_id in failed_clips:
                                    print(f"     - Clip {clip_id}: Cable not inserted")
                                
                                # Ask user what to do about failed clips
                                retry_failed = input("\nWould you like to note this for retry in next cycle? (y/N): ").strip().lower()
                                if retry_failed == 'y':
                                    print("📝 Failed clips noted for potential retry")
                            
                            if successful_clips:
                                print(f"✅ {len(successful_clips)} clip(s) successfully placed and confirmed!")
                        
                        # Handle legacy single status response  
                        else:
                            status_icon = "✅" if response == 'in' else "❌" if response == 'out' else "⚠️"
                            clip_num = "1" if "clip1" in waypoint else "2" if "clip2" in waypoint else "?"
                            print(f"  {status_icon} Clip {clip_num}: {str(response).upper()} (checked after {waypoint})")
                
                # Increment task counter for next cycle only if no force failure
                if not node.force_failure_detected:
                    next_height = node.increment_task_counter()
                    if next_height <= node.minimum_height:
                        print(f"⚠️  Minimum height reached! Next cycles will use {node.minimum_height:.3f}m")
                else:
                    print("⚠️  Task completed but force failure occurred - counter not incremented")
            else:
                print(f" Task '{selected_task}' execution failed!")
                if node.force_failure_detected:
                    print(" Failure was due to force threshold violation")
                    print("📋 Force failure recovery sequence was executed")
                    print(" Please acknowledge the failure before continuing")
                    input("Press Enter to acknowledge and continue...")
                    # Clear failure state after user acknowledgment
                    node.force_failure_detected = False
                    print(" Failure state cleared - ready for next task")
                else:
                    print("Task counter not incremented due to execution failure")
                
            print("\nReady for next task...")
            
    except KeyboardInterrupt:
        print("\n\nInterrupted by user. Exiting...")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()