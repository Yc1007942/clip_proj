from abc import ABC, abstractmethod
from typing import Optional
import numpy as np

class AbstractRobotArm(ABC):
    """Abstract base class for controlling robot arms."""

    @abstractmethod
    def connect(self) -> bool:
        """Connect to the robot."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the robot."""
        pass

    @abstractmethod
    def is_running(self) -> bool:
        """Check if the robot program is running."""
        pass

    @abstractmethod
    def get_current_pose(self) -> Optional[np.ndarray]:
        """Get the current TCP pose as [x, y, z, rx, ry, rz]."""
        pass

    @abstractmethod
    def get_current_wrench(self) -> Optional[np.ndarray]:
        """Get the current TCP wrench as [fx, fy, fz, tx, ty, tz]."""
        pass

    @abstractmethod
    def servo_to_pose(self, pose: list) -> bool:
        """Perform servo motion to the specified pose."""
        pass


class AbstractGripper(ABC):
    """Abstract base class for controlling grippers."""

    class GripperStatus:
        SUCCESS = "SUCCESS"
        ERROR   = "ERROR"
        TIMEOUT = "TIMEOUT"

    @abstractmethod
    def connect(self) -> bool:
        """Connect to the gripper."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the gripper."""
        pass

    @abstractmethod
    def activate(self) -> bool:
        """Activate the gripper."""
        pass

    @abstractmethod
    def open(self) -> tuple:
        """Open the gripper. Returns: (status, message)"""
        pass

    @abstractmethod
    def close(self) -> tuple:
        """Close the gripper. Returns: (status, message)"""
        pass

    @abstractmethod
    def move(self, position: int, speed: int, force: int) -> tuple:
        """Move to specified position. Returns: (status, message)"""
        pass

    @abstractmethod
    def get_current_position(self) -> int:
        """Get the current gripper position."""
        pass

    @abstractmethod
    def auto_calibrate(self, log: bool = True) -> None:
        """Execute automatic gripper calibration."""
        pass


class AbstractForceSensor(ABC):
    """Abstract base class for handling force sensors."""

    @abstractmethod
    def connect(self) -> bool:
        """Connect to the sensor."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the sensor."""
        pass

    @abstractmethod
    def get_raw_data(self) -> Optional[np.ndarray]:
        """
        Return raw sensor data."""
        pass

    @abstractmethod
    def get_wrench(self, current_pose: dict = None) -> Optional[np.ndarray]:
        """Get force/torque in the base coordinate frame."""
        pass