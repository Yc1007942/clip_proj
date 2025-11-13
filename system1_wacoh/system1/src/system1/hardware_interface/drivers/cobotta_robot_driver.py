from typing import Optional
import numpy as np
import pyads

from ..abstract_drivers import AbstractRobotArm
from ...config.config_loader import get_config_loader

# Load YAML configuration
_config_loader = get_config_loader()
config = _config_loader.get_robot_config('cobotta')

class CobottaRobotArmDriver(AbstractRobotArm):
    """Concrete class for controlling COBOTTA robot arms."""

    def __init__(self, logger_callback):
        self.log = logger_callback
        self._plc: Optional[pyads.Connection] = None

    def connect(self) -> bool:
        try:
            self.log(f"Connecting to COBOTTA PLC at {config['PLC_TARGET_IP']} "
                    f"(NetID: {config['PLC_TARGET_NETID']}, Port: {config['PLC_PORT']})...")
            self._plc = pyads.Connection(config['PLC_TARGET_NETID'], config['PLC_PORT'], config['PLC_TARGET_IP'])
            self._plc.open()
            self.log("COBOTTA PLC connection established.")
            return True
        except Exception as e:
            self.log(f"Failed to connect to COBOTTA PLC: {e}", level="error")
            self._plc = None
            return False

    def disconnect(self) -> None:
        if self._plc:
            try:
                self._plc.close()
                self.log("COBOTTA PLC connection closed.")
            except Exception as e:
                self.log(f"Error during disconnect: {e}", level="error")
        self._plc = None

    def is_running(self) -> bool:
        return self._plc is not None and self._plc.is_open

    def get_current_pose(self) -> Optional[np.ndarray]:
        if not self.is_running():
            self.log("Not connected to COBOTTA PLC.", level="error")
            return None
        try:
            pose = self._plc.read_by_name(config['CURRENT_POSE_VAR'],
                                            pyads.PLCTYPE_ARR_REAL(config['CURRENT_POSE_ARRAY_LENGTH']))
            self.log(f"Current pose received: {pose}")
            return np.array(pose)
        except Exception as e:
            self.log(f"Failed to read current pose: {e}", level="error")
            return None

    def get_current_wrench(self) -> Optional[np.ndarray]:
        if not self.is_running():
            self.log("Not connected to COBOTTA PLC.", level="error")
            return None
        try:
            wrench = self._plc.read_by_name(config['CURRENT_WRENCH_VAR'],
                                            pyads.PLCTYPE_ARR_REAL(config['CURRENT_WRENCH_ARRAY_LENGTH']))
            self.log(f"Current wrench received: {wrench}")
            return np.array(wrench)
        except Exception as e:
            self.log(f"Failed to read current wrench: {e}", level="error")
            return None

    def servo_to_pose(self, pose: list) -> bool:
        if not self.is_running():
            self.log("Not connected to COBOTTA PLC.", level="error")
            return False

        if len(pose) != config['COMMAND_ARRAY_LENGTH']:
            self.log(f"Invalid pose length: expected {config['COMMAND_ARRAY_LENGTH']}, got {len(pose)}", level="error")
            return False

        try:
            self._plc.write_by_name(config['TARGET_POSE_VAR'], pose,
                                    pyads.PLCTYPE_ARR_REAL(config['COMMAND_ARRAY_LENGTH']))
            self.log(f"Sent target pose: {pose}")
            return True
        except Exception as e:
            self.log(f"Failed to send target pose: {e}", level="error")
            return False
