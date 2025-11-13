import pytest
from unittest.mock import MagicMock
import numpy as np

from system1.hardware_interface.drivers.ur_force_sensor_driver import UrForceSensorDriver
from system1.hardware_interface.abstract_drivers import AbstractRobotArm

@pytest.fixture
def mock_robot_arm():
    """Fixture to create an AbstractRobotArm mock"""
    return MagicMock(spec=AbstractRobotArm)

@pytest.fixture
def sensor_driver(mock_robot_arm):
    """Fixture to create a UrForceSensorDriver instance"""
    logger_callback = MagicMock()
    driver = UrForceSensorDriver(robot_arm=mock_robot_arm, logger_callback=logger_callback)
    driver.log = logger_callback
    return driver

def test_connect_disconnect(sensor_driver):
    """Test that connect and disconnect are called"""
    assert sensor_driver.connect() is True
    sensor_driver.log.assert_called_with("connect called", level="debug")

    sensor_driver.disconnect()
    sensor_driver.log.assert_called_with("disconnect called", level="debug")

def test_get_wrench_success(sensor_driver, mock_robot_arm):
    """Test when wrench data is successfully obtained"""
    expected_wrench = np.array([1.0, 2.0, 3.0, 0.1, 0.2, 0.3])
    mock_robot_arm.get_current_wrench.return_value = expected_wrench

    wrench = sensor_driver.get_wrench()

    np.testing.assert_array_equal(wrench, expected_wrench)
    mock_robot_arm.get_current_wrench.assert_called_once()

def test_get_wrench_fail(sensor_driver, mock_robot_arm):
    """Test when wrench data acquisition fails (returns None)"""
    mock_robot_arm.get_current_wrench.return_value = None

    wrench = sensor_driver.get_wrench()

    assert wrench is None
    sensor_driver.log.assert_called_with("UR force sensor raw data is None", level="warn")
