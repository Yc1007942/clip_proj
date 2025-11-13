import pytest
from unittest.mock import MagicMock, patch

@pytest.fixture
def mock_rtde_c():
    return MagicMock()

@pytest.fixture
def mock_rtde_r():
    return MagicMock()

@pytest.fixture
def ur_driver(mock_rtde_c, mock_rtde_r):
    # Replace rtde_control and rtde_receive classes with mocks using patch
    with patch('system1.hardware_interface.drivers.ur_robot_driver.rtde_control.RTDEControlInterface', return_value=mock_rtde_c), \
        patch('system1.hardware_interface.drivers.ur_robot_driver.rtde_receive.RTDEReceiveInterface', return_value=mock_rtde_r):
        from system1.hardware_interface.drivers.ur_robot_driver import UrRobotArmDriver
        logger_cb = MagicMock()
        driver = UrRobotArmDriver(logger_callback=logger_cb)
        driver.log = logger_cb
        yield driver

def test_connect_success(ur_driver, mock_rtde_r):
    mock_rtde_r.isConnected.return_value = True

    result = ur_driver.connect()
    assert result is True
    assert ur_driver._is_connected is True

    assert ur_driver.log.call_count == 2
    args1, kwargs1 = ur_driver.log.call_args_list[0]
    assert "Connecting to RTDE" in args1[0]
    args2, kwargs2 = ur_driver.log.call_args_list[1]
    assert args2 == ("RTDE connection successful.",)
    assert kwargs2 == {}

def test_connect_fail(ur_driver, mock_rtde_r):
    mock_rtde_r.isConnected.return_value = False

    result = ur_driver.connect()
    assert result is False
    assert ur_driver._is_connected is False

    assert ur_driver.log.call_count == 2
    args, kwargs = ur_driver.log.call_args_list[-1]
    assert args == (f"RTDE connection failed: isConnected() is False",)
    assert kwargs.get("level") == "error"


def test_disconnect(ur_driver, mock_rtde_c, mock_rtde_r):
    ur_driver._rtde_c = mock_rtde_c
    ur_driver._rtde_r = mock_rtde_r
    ur_driver._is_connected = True

    ur_driver.disconnect()

    mock_rtde_c.servoStop.assert_called_once()
    mock_rtde_c.disconnect.assert_called_once()
    mock_rtde_r.disconnect.assert_called_once()

    ur_driver.log.assert_called_with("RTDE connection closed.")
    assert ur_driver._is_connected is False

def test_servo_to_pose_success(ur_driver, mock_rtde_c):
    ur_driver._is_connected = True
    ur_driver._rtde_c = mock_rtde_c
    mock_rtde_c.isProgramRunning.return_value = True

    pose = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    ok = ur_driver.servo_to_pose(pose)
    assert ok is True

    mock_rtde_c.servoL.assert_called_once()
    args, kwargs = mock_rtde_c.servoL.call_args
    assert args[0] == pose

def test_servo_to_pose_fail_on_exception(ur_driver, mock_rtde_c):
    ur_driver._is_connected = True
    ur_driver._rtde_c = mock_rtde_c
    mock_rtde_c.isProgramRunning.return_value = True
    mock_rtde_c.servoL.side_effect = Exception("test exception")

    pose = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    ok = ur_driver.servo_to_pose(pose)
    assert ok is False

    ur_driver.log.assert_called_with("Failed to send servoL command: test exception", level="error")
    assert ur_driver._is_connected is False
