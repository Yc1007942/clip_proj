from typing import Optional
import numpy as np

import rtde_control
import rtde_receive

from ..abstract_drivers import AbstractRobotArm
from ...config.config_loader import get_config_loader

# Load YAML configuration
_config_loader = get_config_loader()
_common_config = _config_loader.get_common_config()
config = _config_loader.get_robot_config('ur')
HARDWARE_INTERFACE_RATE_HZ = _common_config['HARDWARE_INTERFACE_RATE_HZ']

class UrRobotArmDriver(AbstractRobotArm):
    """Concrete class for controlling UR robot arms."""
    def __init__(self, logger_callback):
        self._rtde_c: Optional[rtde_control.RTDEControlInterface] = None
        self._rtde_r: Optional[rtde_receive.RTDEReceiveInterface] = None
        self.log = logger_callback
        self._is_connected = False

    def connect(self) -> bool:
        if self._is_connected and self._rtde_c and self._rtde_r:
            self.log("RTDE already connected, skipping connect.")
            return True

        self.log(f"Connecting to RTDE ({config['ROBOT_IP']})...")

        try:
            rtde_c = rtde_control.RTDEControlInterface(config['ROBOT_IP'])
            rtde_r = rtde_receive.RTDEReceiveInterface(config['ROBOT_IP'])
        except Exception as e:
            self.log(f"Exception during RTDE connection: {e}", level="error")
            self._is_connected = False
            return False

        try:
            if not rtde_r.isConnected():
                self.log("RTDE connection failed: isConnected() is False", level="error")
                self._is_connected = False
                try: rtde_c.disconnect()
                except: pass
                try: rtde_r.disconnect()
                except: pass
                return False
        except Exception as e:
            self.log(f"Exception during RTDE connection check: {e}", level="error")
            self._is_connected = False
            return False

        self._rtde_c = rtde_c
        self._rtde_r = rtde_r
        self._is_connected = True
        self.log("RTDE connection successful.")
        return True

    def disconnect(self) -> None:
        if self._rtde_c:
            try:
                self._rtde_c.servoStop()
                self._rtde_c.disconnect()
            except Exception:
                pass
        if self._rtde_r:
            try:
                self._rtde_r.disconnect()
            except Exception:
                pass
        self._is_connected = False
        self.log("RTDE connection closed.")

    def is_running(self) -> bool:
        return (
            self._is_connected
            and self._rtde_c is not None
            and self._rtde_c.isProgramRunning()
        )

    def get_current_pose(self) -> Optional[np.ndarray]:
        if not self.is_running() or self._rtde_r is None:
            return None
        try:
            return np.array(self._rtde_r.getActualTCPPose())
        except Exception as e:
            self.log(f"Failed to get pose from RTDE: {e}", level="error")
            return None

    def get_current_wrench(self) -> Optional[np.ndarray]:
        if not self.is_running() or self._rtde_r is None:
            return None
        try:
            return np.array(self._rtde_r.getActualTCPForce())
        except Exception as e:
            self.log(f"Failed to get wrench from RTDE: {e}", level="error")
            return None

    def servo_to_pose(self, pose: list) -> bool:
        if not self.is_running() or self._rtde_c is None:
            return False
        try:
            servo_time = 1.0 / HARDWARE_INTERFACE_RATE_HZ
            self._rtde_c.servoL(
                pose,
                config['SERVO_VEL'],
                config['SERVO_ACC'],
                servo_time,
                config['SERVO_LOOKAHEAD_TIME'],
                config['SERVO_GAIN'],
            )
            return True
        except Exception as e:
            self.log(f"Failed to send servoL command: {e}", level="error")
            self._is_connected = False
            return False