import pytest
from unittest.mock import MagicMock, patch
import numpy as np

mock_wacoh_lib = MagicMock()

modules = {
    'system1.hardware_interface.drivers.wacoh_force_sensor_lib': MagicMock(WacohForceSensor=MagicMock(return_value=mock_wacoh_lib))
}

mock_config = {
    'SENSOR_MOUNT_TRANSFORMS': {
        'wacoh': {
            "rotation": [0, 0, 0, 1]
        }
    },
    'WACOH_SENSOR_PARAMS': {
        "port": "/dev/ttyUSB_Test",
        "baudrate": 921600,
        "timeout": 1.0,
        "bias_sample_count": 5
    },
    'FORCE_SENSOR_TYPE': 'wacoh'
}

with patch.dict('sys.modules', modules), \
    patch('system1.hardware_interface.drivers.wacoh_force_sensor_driver.SENSOR_MOUNT_TRANSFORMS', mock_config['SENSOR_MOUNT_TRANSFORMS']), \
    patch('system1.hardware_interface.drivers.wacoh_force_sensor_driver.WACOH_SENSOR_PARAMS', mock_config['WACOH_SENSOR_PARAMS']), \
    patch('system1.hardware_interface.drivers.wacoh_force_sensor_driver.FORCE_SENSOR_TYPE', mock_config['FORCE_SENSOR_TYPE']):

    from system1.hardware_interface.drivers.wacoh_force_sensor_driver import WacohForceSensorDriver

@pytest.fixture
def wacoh_driver():
    mock_wacoh_lib.reset_mock()
    logger_callback = MagicMock()

    driver = WacohForceSensorDriver(logger_callback=logger_callback)
    driver.sensor = mock_wacoh_lib
    driver.log = logger_callback

    return driver

def test_connect_success(wacoh_driver):
    mock_wacoh_lib.open.return_value = True
    mock_wacoh_lib.init_sensor.return_value = {"err": False, "data": [10, 10, 10, 10, 10, 10]}

    assert wacoh_driver.connect() is True
    mock_wacoh_lib.open.assert_called_once()
    mock_wacoh_lib.init_sensor.assert_called_once()
    np.testing.assert_array_equal(wacoh_driver.zeros, np.array([10]*6))
    wacoh_driver.log.assert_any_call("Zero bias = [10, 10, 10, 10, 10, 10]")

def test_connect_fail_on_open(wacoh_driver):
    mock_wacoh_lib.open.return_value = False

    assert wacoh_driver.connect() is False
    wacoh_driver.log.assert_called_with("Failed to open serial port", "error")

def test_get_wrench(wacoh_driver):
    wacoh_driver.zeros = np.array([10]*6, dtype=int)

    raw_data = {"err": False, "data": [10, 10, 10, 10, 10, 10]}
    mock_wacoh_lib.get_sensor.return_value = raw_data

    with patch.object(mock_wacoh_lib, 'snsvalue_to_force', return_value=[5.0, -5.0, 10.0, 0.5, -0.5, 1.0]):
        wrench_tool = wacoh_driver.get_wrench()
        np.testing.assert_allclose(wrench_tool, [5.0, -5.0, 10.0, 0.5, -0.5, 1.0])

        current_pose = {'quat': [0, 0, np.sin(np.pi/4), np.cos(np.pi/4)]}
        wrench_base = wacoh_driver.get_wrench(current_pose)
        expected_wrench_base = [5.0, 5.0, 10.0, 0.5, 0.5, 1.0]
        np.testing.assert_allclose(wrench_base, expected_wrench_base, atol=1e-6)