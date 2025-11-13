import pytest
from unittest.mock import MagicMock, patch
import numpy as np
import rclpy

from std_msgs.msg import Int32
from segment_task_pkg.msg import PoseWithStiffness, SegmentTaskCmd
from system1.hardware_interface.abstract_drivers import AbstractRobotArm, AbstractGripper

# Automatically initialize and shutdown rclpy
@pytest.fixture(scope="session", autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def hardware_node():
    """
    Mock all AbstractRobotArm, AbstractGripper, and force_sensor
    to create HardwareInterfaceNode.
    """
    from system1.hardware_interface.hardware_interface_node import HardwareInterfaceNode

    # Mocks for robot arm and gripper
    mock_robot_arm   = MagicMock(spec=AbstractRobotArm)
    mock_gripper     = MagicMock(spec=AbstractGripper)
    # Mock for force sensor
    mock_force_sensor = MagicMock()

    # Configure behavior of each mock
    mock_robot_arm.connect.return_value         = True
    mock_robot_arm.is_running.return_value      = True
    mock_robot_arm.get_current_pose.return_value = np.zeros(6)
    # servo_to_pose return value is not specifically used, but set to True since False stops the loop
    mock_robot_arm.servo_to_pose.return_value   = True

    mock_gripper.connect.return_value = True

    mock_force_sensor.connect.return_value   = True
    mock_force_sensor.get_wrench.return_value = np.zeros(6)

    # Create instance
    node = HardwareInterfaceNode(
        robot_arm=mock_robot_arm,
        gripper=mock_gripper,
        force_sensor=mock_force_sensor,
    )

    # Store for easy access within tests
    node.mock_robot_arm     = mock_robot_arm
    node.mock_gripper       = mock_gripper
    node.mock_force_sensor  = mock_force_sensor

    yield node

    # tear-down
    node.destroy_node()


def test_target_cb_resets_bias(hardware_node):
    """Verify that _target_cb resets wrench bias and clears samples when receiving a new target"""
    # Set dummy bias beforehand
    hardware_node._wrench_bias = np.ones(6)
    hardware_node._wrench_samples = [np.zeros(6), np.zeros(6)]

    # Send new PoseWithStiffness
    new_target = PoseWithStiffness()
    new_target.pose.pose.position.x = 1.0  # Reset action occurs if position changes
    hardware_node._target_cb(new_target)

    # Verify reset occurred
    assert hardware_node._wrench_bias is None
    assert hardware_node._wrench_samples == []

    # adm_calc is also reset(), so initial offset should remain at 0
    # (Check adm_pos_off attribute of TargetPoseCorrector if available)
    assert (hardware_node._adm_calc.adm_pos_off == 0).all()


def test_servo_loop_calls_drivers(hardware_node):
    """Verify that _servo_loop calls robot_arm.get_current_pose, force_sensor.get_wrench,
        adm_calc.update, and servo_to_pose"""
    # Wrap to count adm_calc.update calls
    with patch.object(hardware_node, '_adm_calc', wraps=hardware_node._adm_calc) as mock_adm:
        # First reset internally with _target_cb
        tgt = PoseWithStiffness()
        tgt.pose.pose.position.x = 0.1
        hardware_node._target_cb(tgt)

        # Set bias in advance
        hardware_node._wrench_bias = np.zeros(6)

        # Execute servo loop
        hardware_node._servo_loop()

        # robot_arm.get_current_pose() called once
        hardware_node.mock_robot_arm.get_current_pose.assert_called_once()

        # force_sensor.get_wrench() called once
        hardware_node.mock_force_sensor.get_wrench.assert_called_once()
        # current_pose dict should be passed as call argument
        args, _ = hardware_node.mock_force_sensor.get_wrench.call_args
        assert isinstance(args[0], dict) and 'pos' in args[0] and 'quat' in args[0]

        # adm_calc.update() called once
        mock_adm.update.assert_called_once()

        # servo_to_pose() called once
        hardware_node.mock_robot_arm.servo_to_pose.assert_called_once()


def test_gripper_cmd_cb(hardware_node):
    """Verify that _gripper_cmd_cb calls gripper.open/close in response to OPEN/CLOSE commands"""
    # Expect open/close to return (status, message)
    hardware_node.mock_gripper.open.return_value  = (True, "ok")
    hardware_node.mock_gripper.close.return_value = (True, "ok")

    # OPEN
    open_msg = Int32(data=SegmentTaskCmd.OPEN_END_EFFECTOR)
    hardware_node._gripper_cmd_cb(open_msg)
    hardware_node.mock_gripper.open.assert_called_once()

    # CLOSE
    close_msg = Int32(data=SegmentTaskCmd.CLOSE_END_EFFECTOR)
    hardware_node._gripper_cmd_cb(close_msg)
    hardware_node.mock_gripper.close.assert_called_once()