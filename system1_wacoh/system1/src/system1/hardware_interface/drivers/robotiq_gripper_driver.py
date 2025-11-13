from typing import Tuple
from rclpy.node import Node

from ..abstract_drivers import AbstractGripper
from .robotiq_gripper_lib import RobotiqGripper
from ...config.config_loader import get_config_loader

# Load YAML configuration
_config_loader = get_config_loader()
config = _config_loader.get_gripper_config('robotiq')

class RobotiqGripperDriver(AbstractGripper):
    """Concrete class for controlling Robotiq grippers."""

    def __init__(self, logger_callback):
        self._gripper = RobotiqGripper()
        self.log = logger_callback
        self._is_connected = False

    def connect(self) -> bool:
        try:
            self.log(f"Connecting to Gripper ({config['GRIPPER_IP']}:{config['GRIPPER_PORT']})...")
            self._gripper.connect(config['GRIPPER_IP'], config['GRIPPER_PORT'])
            self._is_connected = True
            self.log("Gripper connected successfully.")
            return True
        except Exception as e:
            self.log(f"Gripper connection failed: {e}", level="error")
            self._is_connected = False
            return False

    def disconnect(self) -> None:
        if self._is_connected:
            self._gripper.disconnect()
            self._is_connected = False
            self.log("Gripper connection closed.")

    def activate(self) -> bool:
        if not self._is_connected:
            self.log("Gripper is not connected.", level="error")
            return False

        try:
            if not self._gripper.is_active():
                self.log("Gripper not activated. Executing activation sequence.")
                self._gripper.activate()
            self.log("Gripper activation successful.")
            return True
        except Exception as e:
            self.log(f"Gripper activation failed: {e}", level="error")
            return False

    def move(self, position: int, speed: int = 255, force: int = 50) -> Tuple[str, str]:
        if not self._is_connected or not self._gripper.is_active():
            msg = "Gripper is not connected or activated."
            self.log(msg, level="error")
            return self.GripperStatus.ERROR, msg
        try:
            final_pos, status = self._gripper.move_and_wait_for_pos(position, speed, force)
            msg = f"Move completed (target pos={position}, final pos={final_pos}, status={status.name})"
            self.log(msg)
            return self.GripperStatus.SUCCESS, msg
        except Exception as e:
            msg = f"Gripper move error: {e}"
            self.log(msg, level="error")
            return self.GripperStatus.ERROR, msg

    def open(self) -> Tuple[str, str]:
        try:
            open_position = self._gripper.get_open_position()
        except Exception as e:
            msg = f"Failed to get open position: {e}"
            self.log(msg, level="error")
            return self.GripperStatus.ERROR, msg
        return self.move(open_position, speed=255, force=50)

    def close(self) -> Tuple[str, str]:
        try:
            closed_position = self._gripper.get_closed_position()
        except Exception as e:
            msg = f"Failed to get closed position: {e}"
            self.log(msg, level="error")
            return self.GripperStatus.ERROR, msg
        return self.move(closed_position, speed=255, force=50)

    def get_current_position(self) -> int:
        if not self._is_connected:
            self.log("Gripper is not connected.", level="error")
            return -1
        try:
            current_pos = self._gripper.get_current_position()
            self.log(f"Current gripper position: {current_pos}")
            return current_pos
        except Exception as e:
            self.log(f"Failed to get current position: {e}", level="error")
            return -1

    def auto_calibrate(self, log: bool = True) -> None:
        if not self._is_connected or not self._gripper.is_active():
            self.log("Gripper is not connected or activated. Cannot calibrate.", level="error")
            return
        try:
            self._gripper.auto_calibrate(log=log)
            self.log("Gripper auto-calibration completed.")
        except Exception as e:
            self.log(f"Auto-calibration failed: {e}", level="error")