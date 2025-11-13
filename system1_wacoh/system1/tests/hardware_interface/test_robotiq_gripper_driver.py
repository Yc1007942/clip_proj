import pytest
from unittest.mock import MagicMock
from pathlib import Path
import yaml

_CONFIG_DIR = Path(__file__).resolve().parents[2] / "src" / "system1" / "config"
config = yaml.safe_load((_CONFIG_DIR / "robotiq_gripper.yaml").read_text(encoding="utf-8"))

from system1.hardware_interface.drivers.robotiq_gripper_driver import RobotiqGripperDriver
from system1.hardware_interface.abstract_drivers import AbstractGripper



@pytest.fixture
def mock_gripper(monkeypatch):
    """Fixture to create a RobotiqGripper mock"""
    inst = MagicMock()
    monkeypatch.setattr(
        'system1.hardware_interface.drivers.robotiq_gripper_driver.RobotiqGripper',
        MagicMock(return_value=inst)
    )
    return inst

@pytest.fixture
def gripper_driver(mock_gripper):
    """Fixture to create a RobotiqGripperDriver instance"""
    logger = MagicMock()
    drv = RobotiqGripperDriver(logger_callback=logger)
    drv.log = logger
    return drv

def test_connect_success(gripper_driver, mock_gripper):
    """Test successful connection"""
    assert gripper_driver.connect() is True

    mock_gripper.connect.assert_called_once_with(config['GRIPPER_IP'], config['GRIPPER_PORT'])

    gripper_driver.log.assert_any_call(f"Connecting to Gripper ({config['GRIPPER_IP']}:{config['GRIPPER_PORT']})...")
    gripper_driver.log.assert_any_call("Gripper connected successfully.")
    assert gripper_driver._is_connected is True

def test_connect_fail(gripper_driver, mock_gripper):
    """Test connection failure"""
    mock_gripper.connect.side_effect = Exception("Connection Error")
    assert gripper_driver.connect() is False

    gripper_driver.log.assert_called_with(
        "Gripper connection failed: Connection Error",
        level="error"
    )
    assert gripper_driver._is_connected is False

def test_activate_not_connected(gripper_driver):
    """Test activation when gripper is not connected"""
    gripper_driver._is_connected = False
    ok = gripper_driver.activate()
    assert ok is False
    gripper_driver.log.assert_called_with("Gripper is not connected.", level="error")

def test_activate_when_inactive(gripper_driver, mock_gripper):
    """Test activation when gripper is inactive"""
    gripper_driver._is_connected = True
    mock_gripper.is_active.return_value = False

    ok = gripper_driver.activate()
    assert ok is True
    mock_gripper.activate.assert_called_once()
    gripper_driver.log.assert_called_with("Gripper activation successful.")

def test_activate_when_already_active(gripper_driver, mock_gripper):
    """Test activation when gripper is already active"""
    gripper_driver._is_connected = True
    mock_gripper.is_active.return_value = True

    ok = gripper_driver.activate()
    assert ok is True
    mock_gripper.activate.assert_not_called()
    gripper_driver.log.assert_called_with("Gripper activation successful.")

def test_open(gripper_driver, mock_gripper):
    """Test open command"""
    gripper_driver._is_connected = True
    mock_gripper.get_open_position.return_value = 10
    mock_gripper.is_active.return_value = True
    mock_gripper.move_and_wait_for_pos.return_value = (10, MagicMock(name='STATUS'))

    status, msg = gripper_driver.open()
    assert status == AbstractGripper.GripperStatus.SUCCESS

    mock_gripper.get_open_position.assert_called_once()
    mock_gripper.move_and_wait_for_pos.assert_called_once_with(10, 255, 50)

def test_close(gripper_driver, mock_gripper):
    """Test close command"""
    gripper_driver._is_connected = True
    mock_gripper.get_closed_position.return_value = 20
    mock_gripper.is_active.return_value = True
    mock_gripper.move_and_wait_for_pos.return_value = (20, MagicMock(name='STATUS'))

    status, msg = gripper_driver.close()
    assert status == AbstractGripper.GripperStatus.SUCCESS

    mock_gripper.get_closed_position.assert_called_once()
    mock_gripper.move_and_wait_for_pos.assert_called_once_with(20, 255, 50)