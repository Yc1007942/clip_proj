import pytest
import numpy as np
from scipy.spatial.transform import Rotation as R

from system1.hardware_interface.target_pose_corrector import TargetPoseCorrector

# Define test parameters as a fixture
@pytest.fixture
def adm_params():
    stiffness = np.diag([1500, 1500, 1500, 20, 20, 20])
    damping = np.diag([50, 50, 50, 5, 5, 5])
    return {
        "inertia": np.diag([1.0]*6),
        "stiffness_profiles": {
            0: stiffness,
            1: stiffness,
        },
        "damping_profiles": {
            0: damping,
            1: damping,
        },
        "force_deadband": 1.0,
        "torque_deadband": 0.05,
        "pos_ctrl_linear_step": 0.05,
        "pos_ctrl_angular_step": 0.10,
        "adm_ctrl_linear_step": 0.01,
        "adm_ctrl_angular_step": 0.025,
        "max_pos_offset": 0.05,
        "max_rot_offset": 0.2,
    }

@pytest.fixture
def corrector(adm_params):
    """Fixture to create a TargetPoseCorrector instance"""
    return TargetPoseCorrector(
        dt=1.0 / 50.0,
        adm_params=adm_params,
    )

def test_reset(corrector):
    """Test that the reset method clears internal state to zero"""
    corrector.adm_pos_off = np.array([0.1, 0.2, 0.3])
    corrector.adm_rot_off = np.array([0.4, 0.5, 0.6])
    corrector.adm_vel_off = np.array([0.7, 0.8, 0.9, 1.0, 1.1, 1.2])
    
    corrector.reset()
    
    np.testing.assert_allclose(corrector.adm_pos_off, np.zeros(3))
    np.testing.assert_allclose(corrector.adm_rot_off, np.zeros(3))
    np.testing.assert_allclose(corrector.adm_vel_off, np.zeros(6))

def test_position_control_mode(corrector):
    """Test operation in position control mode (stiffness_id=0)"""
    current_pose = {'pos': np.array([0.0, 0.0, 0.0]), 'quat': np.array([0, 0, 0, 1])}
    target_pose = {'pos': np.array([0.1, 0.0, 0.0]), 'quat': np.array([0, 0, 0, 1])}
    wrench_base = np.array([10.0, 0, 0, 0, 0, 0]) # Force should be ignored
    stiffness_id = 0

    # Update multiple times to confirm approach to target
    next_pose = current_pose
    for _ in range(10):
        next_pose = corrector.update(next_pose, target_pose, wrench_base, stiffness_id, np.zeros(6))

    # Confirm position approaches target
    final_pos = next_pose['pos']
    assert np.linalg.norm(target_pose['pos'] - final_pos) < 1e-3

    # Confirm admittance offset is reset
    np.testing.assert_allclose(corrector.adm_pos_off, np.zeros(3))

def test_admittance_control_with_force(corrector, adm_params):
    """Test operation in admittance control mode with external force"""
    current_pose = {'pos': np.array([0.0, 0.0, 0.0]), 'quat': np.array([0, 0, 0, 1])}
    target_pose = {'pos': np.array([0.1, 0.0, 0.0]), 'quat': np.array([0, 0, 0, 1])}
    # Apply force larger than deadband in Z-axis direction
    wrench_base = np.array([0, 0, 10.0, 0, 0, 0])
    stiffness_id = 1

    # Update multiple times
    next_pose = current_pose
    for _ in range(10):
        next_pose = corrector.update(next_pose, target_pose, wrench_base, stiffness_id, np.zeros(6))

    # Confirm position offset occurs due to external force
    # In this test, confirm offset occurs in Z direction
    assert corrector.adm_pos_off[2] > 0.001

    # Should also progress in X direction
    assert next_pose['pos'][0] > 0
    # Should also be displaced in Z direction according to the force
    assert next_pose['pos'][2] > 0