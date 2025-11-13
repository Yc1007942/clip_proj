import numpy as np
from typing import Optional, Dict
from system1.hardware_interface.abstract_drivers import AbstractForceSensor

class UrForceSensorDriver(AbstractForceSensor):
    """Concrete class for handling UR built-in force sensor."""

    def __init__(self,
                robot_arm,
                logger_callback=lambda msg, level="info": print(f"[URFTS][{level}] {msg}")):
        self.robot_arm = robot_arm
        self.log       = logger_callback

    def connect(self) -> bool:
        self.log("connect called", level="debug")
        return True

    def disconnect(self) -> None:
        self.log("disconnect called", level="debug")
        pass

    def get_raw_data(self) -> Optional[np.ndarray]:
        return self.robot_arm.get_current_wrench()

    def get_wrench(self, current_pose: Dict = None) -> Optional[np.ndarray]:
        raw = self.get_raw_data()
        if raw is None:
            self.log("UR force sensor raw data is None", level="warn")
            return None
        return np.array(raw, dtype=float).copy()
