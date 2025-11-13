import time
import threading
import csv
import os
from pathlib import Path
from typing import List, Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Int32, String
from segment_task_pkg.action import ExecuteMoveHandWithStiffness, ExecuteSegmentTask
from segment_task_pkg.msg import PoseWithStiffness, SegmentTaskCmd, SegmentTaskStatus

from .config.config_loader import get_config_loader

# Load YAML configuration
_config_loader = get_config_loader()
config = _config_loader.get_common_config()

class MotionController(Node):
    """
    ROS node for controlling robot motion.
    Receives path commands and gripper operation commands from higher-level systems and controls robot arm motion.
    """
    def __init__(self) -> None:
        super().__init__("motion_controller")
        self.get_logger().info("Motion Controller starting up.")

        self._log_rows: List[List[float]] = []
        self._cg = ReentrantCallbackGroup()

        self._move_hand_active = False
        self._robot_action_active = False

        self._move_hand_server = ActionServer(
            self,
            ExecuteMoveHandWithStiffness,
            "/cobotta/move_hand_with_stiffness",
            execute_callback=self._execute_path_cb,
            goal_callback=self._goal_callback,
            callback_group=self._cg,
        )

        self._gripper_server = ActionServer(
            self,
            ExecuteSegmentTask,
            "/cobotta/robot_action",
            execute_callback=self._execute_gripper_cb,
            goal_callback=self._goal_callback,
            callback_group=self._cg,
        )

        self.create_subscription(
            PoseStamped, "/robot_driver/current_pose", self._robot_pose_cb, 10, callback_group=self._cg
        )
        self.create_subscription(
            WrenchStamped, "/robot_driver/current_wrench", self._robot_wrench_cb, 10, callback_group=self._cg
        )

        self._target_pub = self.create_publisher(PoseWithStiffness, "/motion_controller/target_pose", 10)
        self._gripper_pub = self.create_publisher(Int32, "/motion_controller/gripper_command", 10)
        
        self.create_subscription(String, "/robot_driver/gripper_result", self._gripper_result_cb, 10)

        self._dt = 1.0 / config['MOTION_CONTROLLER_RATE_HZ']
        self._control_timer = None
        self._log_timer = None

        self._waypoints: List[PoseWithStiffness] = []
        self._wp_idx = 0

        self._path_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._goal_lock = threading.Lock()

        self._current_pose: Optional[PoseStamped] = None
        self._current_wrench: Optional[WrenchStamped] = None
        self._wp_start_pose: Optional[PoseStamped] = None

        self._pending_gripper_event: Optional[threading.Event] = None
        self._pending_gripper_result: Optional[str] = None

    def _robot_pose_cb(self, msg: PoseStamped):
        with self._state_lock:
            self._current_pose = msg

    def _robot_wrench_cb(self, msg: WrenchStamped):
        with self._state_lock:
            self._current_wrench = msg

    def _goal_callback(self, goal_request) -> GoalResponse:
        with self._goal_lock:
            if self._move_hand_active or self._robot_action_active:
                self.get_logger().warn("Rejecting new goal because another action is already active.")
                return GoalResponse.REJECT
            if isinstance(goal_request, ExecuteMoveHandWithStiffness.Goal):
                self._move_hand_active = True
            elif isinstance(goal_request, ExecuteSegmentTask.Goal):
                self._robot_action_active = True

        return GoalResponse.ACCEPT

    def _execute_path_cb(self, goal_handle: ServerGoalHandle):
        self._move_hand_active = True
        try:
            path = goal_handle.request.path
            with self._path_lock:
                self._waypoints = list(path.poses_with_stiffness)
                self._wp_idx = 0
                self._wp_start_pose = None
            
            self._log_rows = []
            if not self._waypoints:
                goal_handle.abort()
                return ExecuteMoveHandWithStiffness.Result(status=-1, message="Path is empty.")

            self.get_logger().info(f"Received new path command with {len(self._waypoints)} waypoints.")

            for i, wp in enumerate(self._waypoints):
                self.get_logger().info(f"Waypoint {i + 1}: pos=({wp.pose.pose.position.x}, "
                                        f"{wp.pose.pose.position.y}, {wp.pose.pose.position.z}), "
                                        f"quat=({wp.pose.pose.orientation.x}, "
                                        f"{wp.pose.pose.orientation.y}, "
                                        f"{wp.pose.pose.orientation.z}, "
                                        f"{wp.pose.pose.orientation.w}), "
                                        f"stiffness_id={wp.stiffness_id}")
            
            self._stop_timers()
            self._control_timer = self.create_timer(self._dt, self._control_loop, callback_group=self._cg)
            self._log_timer = self.create_timer(self._dt, self._log_data, callback_group=self._cg)

            start_time = self.get_clock().now()
            timeout_sec = 15.0 + len(self._waypoints) * 15.0

            while rclpy.ok() and (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout_sec:
                time.sleep(0.1)
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info("Path execution was canceled.")
                    break
                with self._path_lock:
                    if self._wp_idx >= len(self._waypoints):
                        self.get_logger().info("Reached the end of the path.")
                        break
            
            self._stop_timers()
            self._write_csv()
            
            finished = self._wp_idx >= len(self._waypoints)
            if finished and not goal_handle.is_cancel_requested:
                goal_handle.succeed()
                return ExecuteMoveHandWithStiffness.Result(status=1, message="Path completed.")
            else:
                goal_handle.abort()
                return ExecuteMoveHandWithStiffness.Result(status=-1, message="Path aborted.")
        finally:
            self._move_hand_active = False

    def _execute_gripper_cb(self, goal_handle: ServerGoalHandle):
        self._robot_action_active = True
        try:
            cmd = goal_handle.request.cmd
            if cmd not in [SegmentTaskCmd.OPEN_END_EFFECTOR, SegmentTaskCmd.CLOSE_END_EFFECTOR]:
                goal_handle.abort()
                return ExecuteSegmentTask.Result(
                    status=SegmentTaskStatus.CONTROLLER_PROBLEMS,
                    message=f"Unknown command: {cmd}"
                )

            self.get_logger().info(f"Received gripper command: cmd={cmd}")
            gripper_msg = Int32(data=cmd)
            self._gripper_pub.publish(gripper_msg)

            self._pending_gripper_event = threading.Event()
            if self._pending_gripper_event.wait(timeout=10.0):
                self.get_logger().info(f"Gripper operation completed: {self._pending_gripper_result}")
                goal_handle.succeed()
                return ExecuteSegmentTask.Result(
                    status=SegmentTaskStatus.SUCCESS,
                    message=self._pending_gripper_result
                )
            else:
                self.get_logger().error("Gripper operation timed out.")
                goal_handle.abort()
                return ExecuteSegmentTask.Result(
                    status=SegmentTaskStatus.STOPPED,
                    message="Timeout"
                )
        finally:
            self._robot_action_active = False

    def _gripper_result_cb(self, msg: String):
        if self._pending_gripper_event:
            self._pending_gripper_result = msg.data
            self._pending_gripper_event.set()

    def _control_loop(self):
        with self._state_lock:
            if self._current_pose is None:
                return
            current_pose_locked = self._current_pose

        with self._path_lock:
            if self._wp_idx >= len(self._waypoints):
                last_wp = self._waypoints[-1]

                final_target = PoseWithStiffness()
                final_target.pose = last_wp.pose
                final_target.stiffness_id = 0
                
                final_target.pose.header.stamp = self.get_clock().now().to_msg()
                self._target_pub.publish(final_target)
                return

            if self._wp_start_pose is None:
                self._wp_start_pose = current_pose_locked

            current_wp = self._waypoints[self._wp_idx]
            if self._check_waypoint_reached(current_wp, current_pose_locked, self._wp_start_pose):
                self.get_logger().info(f"Reached waypoint {self._wp_idx + 1}/{len(self._waypoints)}.")
                self._wp_idx += 1
                self._wp_start_pose = None

                if self._wp_idx >= len(self._waypoints):
                    self.get_logger().info("All waypoints reached. Switching to hold-position mode.")
                    return
            
            wp_to_send = self._waypoints[self._wp_idx]
            wp_to_send.pose.header.stamp = self.get_clock().now().to_msg()
            self._target_pub.publish(wp_to_send)
    
    def _check_waypoint_reached(self, target_wp: PoseWithStiffness, current_pose: PoseStamped, start_pose: PoseStamped) -> bool:
        target_pos = np.array([
            target_wp.pose.pose.position.x,
            target_wp.pose.pose.position.y,
            target_wp.pose.pose.position.z
        ])
        current_pos = np.array([
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            current_pose.pose.position.z
        ])
        
        target_rot = R.from_quat([
            target_wp.pose.pose.orientation.x,
            target_wp.pose.pose.orientation.y,
            target_wp.pose.pose.orientation.z,
            target_wp.pose.pose.orientation.w
        ])
        current_rot = R.from_quat([
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w
        ])

        pos_error_vec = target_pos - current_pos
        orientation_error_rad = (target_rot * current_rot.inv()).magnitude()

        stiffness_id = target_wp.stiffness_id
        if stiffness_id == 0:
            return (np.linalg.norm(pos_error_vec) < config['POS_CTRL_REACHED_POSITION_THRESHOLD'] and
                    orientation_error_rad < config['POS_CTRL_REACHED_ORIENTATION_THRESHOLD'])
        else:
            start_pos = np.array([
                start_pose.pose.position.x,
                start_pose.pose.position.y,
                start_pose.pose.position.z
            ])
            insertion_dir_vec = target_pos - start_pos
            norm = np.linalg.norm(insertion_dir_vec)
            if norm < 1e-6:
                return (np.linalg.norm(pos_error_vec) < config['ADM_CTRL_REACHED_INSERTION_THRESHOLD'] and
                        orientation_error_rad < config['ADM_CTRL_REACHED_ORIENTATION_THRESHOLD'])

            insertion_dir_unit = insertion_dir_vec / norm
            insertion_error = abs(np.dot(pos_error_vec, insertion_dir_unit))
            lateral_error = np.linalg.norm(pos_error_vec - np.dot(pos_error_vec, insertion_dir_unit) * insertion_dir_unit)

            return (insertion_error < config['ADM_CTRL_REACHED_INSERTION_THRESHOLD'] and
                    lateral_error < config['ADM_CTRL_REACHED_LATERAL_THRESHOLD'] and
                    orientation_error_rad < config['ADM_CTRL_REACHED_ORIENTATION_THRESHOLD'])

    def _log_data(self):
        with self._state_lock:
            if self._current_pose is None or self._current_wrench is None:
                return
            current_pose_locked = self._current_pose
            current_wrench_locked = self._current_wrench
        
        with self._path_lock:
            wp_idx_locked = self._wp_idx

        now_msg = self.get_clock().now().to_msg()
        row = [
            now_msg.sec + now_msg.nanosec * 1e-9, wp_idx_locked,
            current_pose_locked.pose.position.x,
            current_pose_locked.pose.position.y,
            current_pose_locked.pose.position.z,
            current_pose_locked.pose.orientation.x,
            current_pose_locked.pose.orientation.y,
            current_pose_locked.pose.orientation.z,
            current_pose_locked.pose.orientation.w,
            current_wrench_locked.wrench.force.x,
            current_wrench_locked.wrench.force.y,
            current_wrench_locked.wrench.force.z,
            current_wrench_locked.wrench.torque.x,
            current_wrench_locked.wrench.torque.y,
            current_wrench_locked.wrench.torque.z
        ]
        self._log_rows.append(row)

    def _write_csv(self):
        try:
            log_dir = Path.home() / "ros2_logs" / "system1"
            log_dir.mkdir(parents=True, exist_ok=True)
            filename = log_dir / f"log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            
            with open(filename, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "time", "wp_idx",
                    "pos_x", "pos_y", "pos_z",
                    "quat_x", "quat_y", "quat_z", "quat_w",
                    "force_x", "force_y", "force_z",
                    "torque_x", "torque_y", "torque_z"
                ])
                writer.writerows(self._log_rows)
            self.get_logger().info(f"Log saved to {filename}.")
        except Exception as e:
            self.get_logger().error(f"Failed to save log: {e}")

    def _stop_timers(self):
        if self._control_timer: self._control_timer.cancel()
        if self._log_timer: self._log_timer.cancel()
        self._control_timer = None
        self._log_timer = None

    def destroy_node(self):
        self._stop_timers()
        super().destroy_node()
