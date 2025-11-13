import pytest
import numpy as np
from unittest.mock import MagicMock, patch
from pathlib import Path
import yaml

_CONFIG_DIR = Path(__file__).resolve().parents[2] / "src" / "system1" / "config"
config = yaml.safe_load((_CONFIG_DIR / "cobotta_robot.yaml").read_text(encoding="utf-8"))

@pytest.fixture(autouse=True)
def setup_config():
    config.update({
        'PLC_TARGET_IP': "192.168.0.10",
        'PLC_TARGET_NETID': "5.6.7.8.1.1",
        'PLC_PORT': 851,
        'CURRENT_POSE_VAR': "currentPose",
        'CURRENT_POSE_ARRAY_LENGTH': 6,
        'CURRENT_WRENCH_VAR': "currentWrench",
        'CURRENT_WRENCH_ARRAY_LENGTH': 6,
        'COMMAND_ARRAY_LENGTH': 6,
        'TARGET_POSE_VAR': "targetPose",
    })
    yield

# Import CobottaRobotArmDriver main body
from system1.hardware_interface.drivers.cobotta_robot_driver import CobottaRobotArmDriver

@pytest.fixture
def mock_plc(monkeypatch):
    """
    Replace pyads.Connection and
    replace PLCTYPE_ARR_REAL with a simple primitive
    """
    plc = MagicMock()
    plc.is_open = False
    # Connection(...) -> returns plc
    monkeypatch.setattr(
        'system1.hardware_interface.drivers.cobotta_robot_driver.pyads.Connection',
        lambda netid, port, ip: plc
    )
    # PLCTYPE_ARR_REAL(n) -> f"ARR_REAL({n})"
    monkeypatch.setattr(
        'system1.hardware_interface.drivers.cobotta_robot_driver.pyads.PLCTYPE_ARR_REAL',
        lambda length: f"ARR_REAL({length})"
    )
    return plc

@pytest.fixture
def driver(mock_plc):
    """
    Create driver by passing MagicMock to logger_callback
    """
    logger = MagicMock()
    drv = CobottaRobotArmDriver(logger_callback=logger)
    # Since driver internally calls self.log(...),
    # explicitly replace it
    drv.log = logger
    return drv

def test_connect_success(driver, mock_plc):
    result = driver.connect()
    assert result is True
    mock_plc.open.assert_called_once()
    driver.log.assert_called_with("COBOTTA PLC connection established.")
    assert driver._plc is mock_plc

def test_connect_failure(driver, mock_plc):
    mock_plc.open.side_effect = Exception("conn fail")
    result = driver.connect()
    assert result is False
    driver.log.assert_called_with("Failed to connect to COBOTTA PLC: conn fail", level="error")
    assert driver._plc is None

def test_disconnect_when_not_connected(driver):
    # Should not throw exception even when _plc is None
    driver._plc = None
    driver.disconnect()
    assert driver._plc is None

def test_disconnect_when_connected(driver, mock_plc):
    driver._plc = mock_plc
    driver.disconnect()
    mock_plc.close.assert_called_once()
    driver.log.assert_called_with("COBOTTA PLC connection closed.")
    assert driver._plc is None

def test_is_running(driver, mock_plc):
    # If _plc=None, return False
    assert driver.is_running() is False

    # If _plc=mock_plc but is_open=False, return False
    driver._plc = mock_plc
    mock_plc.is_open = False
    assert driver.is_running() is False

    # If is_open=True, return True
    mock_plc.is_open = True
    assert driver.is_running() is True

def test_get_current_pose_not_connected(driver):
    out = driver.get_current_pose()
    assert out is None
    driver.log.assert_called_with("Not connected to COBOTTA PLC.", level="error")

def test_get_current_pose_success(driver, mock_plc):
    driver._plc = mock_plc
    mock_plc.is_open = True
    arr = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    mock_plc.read_by_name.return_value = arr

    pose = driver.get_current_pose()
    np.testing.assert_array_equal(pose, np.array(arr))
    mock_plc.read_by_name.assert_called_once_with(
           config['CURRENT_POSE_VAR'],
           f"ARR_REAL({config['CURRENT_POSE_ARRAY_LENGTH']})"
    )
    driver.log.assert_any_call(f"Current pose received: {arr}")

def test_get_current_pose_failure(driver, mock_plc):
    driver._plc = mock_plc
    mock_plc.is_open = True
    mock_plc.read_by_name.side_effect = Exception("read fail")

    out = driver.get_current_pose()
    assert out is None
    driver.log.assert_called_with("Failed to read current pose: read fail", level="error")

def test_get_current_wrench_not_connected(driver):
    out = driver.get_current_wrench()
    assert out is None
    driver.log.assert_called_with("Not connected to COBOTTA PLC.", level="error")

def test_get_current_wrench_success(driver, mock_plc):
    driver._plc = mock_plc
    mock_plc.is_open = True
    arr = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    mock_plc.read_by_name.return_value = arr

    wrench = driver.get_current_wrench()
    np.testing.assert_array_equal(wrench, np.array(arr))
    mock_plc.read_by_name.assert_called_once_with(
           config['CURRENT_WRENCH_VAR'],
           f"ARR_REAL({config['CURRENT_WRENCH_ARRAY_LENGTH']})"
    )
    driver.log.assert_any_call(f"Current wrench received: {arr}")

def test_get_current_wrench_failure(driver, mock_plc):
    driver._plc = mock_plc
    mock_plc.is_open = True
    mock_plc.read_by_name.side_effect = Exception("read fail")

    out = driver.get_current_wrench()
    assert out is None
    driver.log.assert_called_with("Failed to read current wrench: read fail", level="error")

def test_servo_to_pose_not_connected(driver):
    ok = driver.servo_to_pose([0]*config['COMMAND_ARRAY_LENGTH'])
    assert ok is False
    driver.log.assert_called_with("Not connected to COBOTTA PLC.", level="error")

def test_servo_to_pose_invalid_length(driver, mock_plc):
    driver._plc = mock_plc
    mock_plc.is_open = True
    bad = [0]*(config['COMMAND_ARRAY_LENGTH'] + 1)
    ok = driver.servo_to_pose(bad)
    assert ok is False
    driver.log.assert_called_with(
    f"Invalid pose length: expected {config['COMMAND_ARRAY_LENGTH']}, got {len(bad)}",
        level="error"
    )

def test_servo_to_pose_success(driver, mock_plc):
    driver._plc = mock_plc
    mock_plc.is_open = True
    pose = [0.1]*config['COMMAND_ARRAY_LENGTH']

    ok = driver.servo_to_pose(pose)
    assert ok is True
    mock_plc.write_by_name.assert_called_once_with(
           config['TARGET_POSE_VAR'],
        pose,
    f"ARR_REAL({config['COMMAND_ARRAY_LENGTH']})"
    )
    driver.log.assert_called_with(f"Sent target pose: {pose}")
