import threading
import numpy as np
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Int32, String
from scipy.spatial.transform import Rotation as R
from segment_task_pkg.msg import PoseWithStiffness, SegmentTaskCmd

from ..config.config_loader import get_config_loader

# Load YAML configuration
_config_loader = get_config_loader()
common_config = _config_loader.get_common_config()
from .target_pose_corrector import TargetPoseCorrector
from .abstract_drivers import AbstractRobotArm, AbstractGripper


class HardwareInterfaceNode(Node):
    """
    ROS node that interfaces with robot arm, gripper, and force sensor.
    """
    def __init__(
        self,
        robot_arm: AbstractRobotArm,
        gripper: AbstractGripper,
        force_sensor,
        adm_params: dict = None,
        mount_transform: dict = None,
    ):
        super().__init__('hardware_interface')
        self.get_logger().info("Hardware Interface starting up.")

        # Driver instances
        self._robot_arm = robot_arm
        self._gripper = gripper
        self._force_sensor = force_sensor

        # --- Connection & Initialization ---
        self._robot_ok    = self._robot_arm.connect()
        self._gripper_ok  = self._gripper.connect()
        self._force_ok    = self._force_sensor.connect()
        if self._gripper_ok:
            self._gripper.activate()

        # --- Callback group & Locks ---
        self._cg = ReentrantCallbackGroup()
        self._target_lock = threading.Lock()
        self._wrench_lock = threading.Lock()
        self._target_msg: Optional[PoseWithStiffness] = None
        self._latest_wrench: Optional[np.ndarray] = None

        # --- Subscriber ---
        self.create_subscription(
            PoseWithStiffness,
            '/motion_controller/target_pose',
            self._target_cb,
            10,
            callback_group=self._cg,
        )
        self.create_subscription(
            Int32,
            '/motion_controller/gripper_command',
            self._gripper_cmd_cb,
            10,
            callback_group=self._cg,
        )

        # --- Publisher ---
        self._pose_pub = self.create_publisher(PoseStamped,   '/robot_driver/current_pose',   10)
        self._wrench_pub = self.create_publisher(WrenchStamped,'/robot_driver/current_wrench',10)
        self._gripper_result_pub = self.create_publisher(String,'/robot_driver/gripper_result',10)

        # --- Timer implementation ---
        self._servo_timer = self.create_timer(
            1.0/common_config['HARDWARE_INTERFACE_RATE_HZ'],
            self._servo_loop,
            callback_group=self._cg,
        )
        # State publishing at fixed 50Hz (previously 1.0/50.0)
        self._state_timer = self.create_timer(
            1.0 / 50.0,
            self._state_publish_loop,
            callback_group=self._cg,
        )

        # --- Load admittance settings ---
        # Use adm_params passed from __main__.py if available,
        # otherwise use default values
        if adm_params is not None:
            self._adm_settings = adm_params
        else:
            # Set default values
            force_sensor_type = common_config.get('FORCE_SENSOR_TYPE', 'ur_builtin')
            sensor_config = _config_loader.get_sensor_config()
            self._adm_settings = sensor_config.get(force_sensor_type, {}).get('adm_params', {})

        # Same for mount transform
        if mount_transform is not None:
            self._mount_transform = mount_transform
        else:
            force_sensor_type = common_config.get('FORCE_SENSOR_TYPE', 'ur_builtin')
            sensor_config = _config_loader.get_sensor_config()
            self._mount_transform = sensor_config.get('mount_transforms', {}).get(force_sensor_type, {})

        # --- Initialize TargetPoseCorrector ---
        adm = self._adm_settings
        self._adm_calc = TargetPoseCorrector(
            dt=(1.0 / common_config['HARDWARE_INTERFACE_RATE_HZ']),
            adm_params=adm,
        )

        # For wrench bias calculation
        self._wrench_bias = None
        self._wrench_samples = []

        self._last_valid_pose = None
        self._last_valid_wrench = None

    def _log(self, msg: str, level: str = "info"):
        if level == "info":
            self.get_logger().info(msg)
        elif level == "warn":
            self.get_logger().warn(msg)
        else:
            self.get_logger().error(msg)

    def _has_target_changed(
        self,
        old_msg: PoseWithStiffness,
        new_msg: PoseWithStiffness
    ) -> bool:
        # Check for changes in position, rotation angle, and stiffness_id
        old_pos = np.array([
            old_msg.pose.pose.position.x,
            old_msg.pose.pose.position.y,
            old_msg.pose.pose.position.z,
        ])
        new_pos = np.array([
            new_msg.pose.pose.position.x,
            new_msg.pose.pose.position.y,
            new_msg.pose.pose.position.z,
        ])
        if np.linalg.norm(new_pos - old_pos) > 1e-3:
            return True

        old_quat = np.array([
            old_msg.pose.pose.orientation.x,
            old_msg.pose.pose.orientation.y,
            old_msg.pose.pose.orientation.z,
            old_msg.pose.pose.orientation.w,
        ])
        new_quat = np.array([
            new_msg.pose.pose.orientation.x,
            new_msg.pose.pose.orientation.y,
            new_msg.pose.pose.orientation.z,
            new_msg.pose.pose.orientation.w,
        ])
        if (R.from_quat(new_quat) * R.from_quat(old_quat).inv()).magnitude() > 1e-2:
            return True

        return old_msg.stiffness_id != new_msg.stiffness_id

    def _target_cb(self, msg: PoseWithStiffness):
        with self._target_lock:
            if self._target_msg is None or self._has_target_changed(self._target_msg, msg):
                self.get_logger().info("Received new target. Resetting wrench bias.")
                self._wrench_bias = None
                self._wrench_samples.clear()
                self._adm_calc.reset()
            self._target_msg = msg

    def _servo_loop(self):
        # Check if robot connection is active and robot is running
        if not self._robot_ok or not self._robot_arm.is_running():
            return

        # If None is returned due to communication error, continue loop with last successful value
        pose_list = self._robot_arm.get_current_pose()
        if pose_list is None:
            # If pose has never been obtained, warn and abort processing
            if self._last_valid_pose is None:
                self.get_logger().warn("Failed to get current pose and no last valid pose is available.", throttle_duration_sec=1.0)
                return
            # Reuse last successful value
            self.get_logger().warn("Failed to get current pose. Using last valid pose.", throttle_duration_sec=1.0)
            pose_list = self._last_valid_pose
        else:
            # Update value when successfully obtained
            self._last_valid_pose = pose_list

        current_pos = np.array(pose_list[0:3])
        current_rot = R.from_rotvec(np.array(pose_list[3:6]))
        current_pose = {'pos': current_pos, 'quat': current_rot.as_quat()}

        # Similarly, continue loop with last value even if sensor value cannot be obtained
        wrench = self._force_sensor.get_wrench(current_pose)
        if wrench is None:
            if self._last_valid_wrench is None:
                self.get_logger().warn("Failed to read force sensor and no last valid wrench is available.", throttle_duration_sec=1.0)
                return
            self.get_logger().warn("Failed to read force sensor. Using last valid wrench.", throttle_duration_sec=1.0)
            wrench = self._last_valid_wrench
        else:
            self._last_valid_wrench = wrench

        # Store latest wrench information in thread-safe manner
        with self._wrench_lock:
            self._latest_wrench = np.array(wrench, copy=True)

        # Get target value
        with self._target_lock:
            target = self._target_msg
        if target is None:
            return # Do nothing if no target

        # Calculate bias (initial offset) from a certain number of samples immediately after control starts
        if self._wrench_bias is None:
            self._wrench_samples.append(wrench)
            if len(self._wrench_samples) >= self._adm_settings['bias_sample_count']:
                self._wrench_bias = np.mean(self._wrench_samples, axis=0)
                self.get_logger().info(f"Wrench bias calculated: {self._wrench_bias}")
            else:
                # Don't send control commands until bias is calculated
                return

        # Calculate next frame target pose using admittance control
        ref_pos = np.array([target.pose.pose.position.x, target.pose.pose.position.y, target.pose.pose.position.z])
        ref_quat = np.array([target.pose.pose.orientation.x, target.pose.pose.orientation.y, target.pose.pose.orientation.z, target.pose.pose.orientation.w])
        ref_pose = {'pos': ref_pos, 'quat': ref_quat}
        
        cmd = self._adm_calc.update(
            current_pose,
            ref_pose,
            wrench,
            target.stiffness_id,
            self._wrench_bias
        )

        # Send servo command to robot
        cmd_pos = cmd['pos']
        cmd_rvec = R.from_quat(cmd['quat']).as_rotvec()
        servo_cmd = list(cmd_pos) + list(cmd_rvec)
        if not self._robot_arm.servo_to_pose(servo_cmd):
            self.get_logger().error("Failed to send servo command. Stopping servo loop.")
            self._robot_ok = False

    def _state_publish_loop(self):
        if not self._robot_ok or not self._robot_arm.is_running():
            return

        pose_list = self._robot_arm.get_current_pose()
        if pose_list is None:
            return

        with self._wrench_lock:
            wrench = self._latest_wrench
        if wrench is None:
            return

        stamp = self.get_clock().now().to_msg()

        # PoseStamped publish
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = pose_list[:3]
        q = R.from_rotvec(pose_list[3:]).as_quat()
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, \
        pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = q
        self._pose_pub.publish(pose_msg)

        # WrenchStamped publish
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = stamp
        wrench_msg.header.frame_id = "tool0"
        f, t = wrench[:3], wrench[3:]
        wrench_msg.wrench.force.x, wrench_msg.wrench.force.y, wrench_msg.wrench.force.z = f
        wrench_msg.wrench.torque.x, wrench_msg.wrench.torque.y, wrench_msg.wrench.torque.z = t
        self._wrench_pub.publish(wrench_msg)

    def _gripper_cmd_cb(self, msg: Int32):
        cmd = msg.data
        if cmd == SegmentTaskCmd.OPEN_END_EFFECTOR:
            status, info = self._gripper.open()
        elif cmd == SegmentTaskCmd.CLOSE_END_EFFECTOR:
            status, info = self._gripper.close()
        else:
            return
        self._gripper_result_pub.publish(String(data=f"{status}: {info}"))

    def destroy_node(self):
        self.get_logger().info("Hardware Interface shutting down.")
        if hasattr(self, '_servo_timer'): self._servo_timer.cancel()
        if hasattr(self, '_state_timer'): self._state_timer.cancel()
        try:
            self._robot_arm.disconnect()
            self._gripper.disconnect()
        except Exception:
            pass
        super().destroy_node()
