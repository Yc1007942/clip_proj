import pytest
import numpy as np
from scipy.spatial.transform import Rotation as R
from types import SimpleNamespace
from pathlib import Path
import yaml

from geometry_msgs.msg import PoseStamped, Pose
from segment_task_pkg.msg import PoseWithStiffness

_CONFIG_DIR = Path(__file__).resolve().parents[1] / "src" / "system1" / "config"
with open(_CONFIG_DIR / "common.yaml", "r", encoding="utf-8") as _cfg_file:
    config = SimpleNamespace(**yaml.safe_load(_cfg_file))

# MotionController is a ROS node, so here we test only the logic that doesn't depend on ROS
# To test the _check_waypoint_reached method, we implement the method's logic directly
# as a test function without instantiating the class.
# This is because the original class requires ROS initialization.

def check_waypoint_reached_logic(target_wp: PoseWithStiffness, current_pose: PoseStamped, start_pose: PoseStamped) -> bool:
    """Reproduce the logic of MotionController._check_waypoint_reached"""
    target_pos = np.array([
        target_wp.pose.pose.position.x,
        target_wp.pose.pose.position.y,
        target_wp.pose.pose.position.z
    ])
    current_pos = np.array([
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z
    ])
    
    target_rot = R.from_quat([
        target_wp.pose.pose.orientation.x,
        target_wp.pose.pose.orientation.y,
        target_wp.pose.pose.orientation.z,
        target_wp.pose.pose.orientation.w
    ])
    current_rot = R.from_quat([
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w
    ])

    pos_error_vec = target_pos - current_pos
    orientation_error_rad = (target_rot * current_rot.inv()).magnitude()

    stiffness_id = target_wp.stiffness_id
    if stiffness_id == 0:
        return (np.linalg.norm(pos_error_vec) < config.POS_CTRL_REACHED_POSITION_THRESHOLD and
                orientation_error_rad < config.POS_CTRL_REACHED_ORIENTATION_THRESHOLD)
    else:
        start_pos = np.array([
            start_pose.pose.position.x,
            start_pose.pose.position.y,
            start_pose.pose.position.z
        ])
        insertion_dir_vec = target_pos - start_pos
        if np.linalg.norm(insertion_dir_vec) < 1e-6:
            return False

        insertion_dir_unit = insertion_dir_vec / np.linalg.norm(insertion_dir_vec)
        insertion_error = abs(np.dot(pos_error_vec, insertion_dir_unit))
        lateral_error = np.linalg.norm(pos_error_vec - np.dot(pos_error_vec, insertion_dir_unit) * insertion_dir_unit)

        return (insertion_error < config.ADM_CTRL_REACHED_INSERTION_THRESHOLD and
                lateral_error < config.ADM_CTRL_REACHED_LATERAL_THRESHOLD and
                orientation_error_rad < config.ADM_CTRL_REACHED_ORIENTATION_THRESHOLD)

@pytest.fixture
def sample_poses():
    start = PoseStamped()
    start.pose.position.x = 0.0
    start.pose.position.y = 0.0
    start.pose.position.z = 0.0
    
    target = PoseWithStiffness()
    target.pose.pose.position.x = 0.5
    target.pose.pose.position.y = 0.0
    target.pose.pose.position.z = 0.0
    target.pose.pose.orientation.w = 1.0

    current = PoseStamped()
    current.pose.position.x = 0.5
    current.pose.position.y = 0.0
    current.pose.position.z = 0.0
    current.pose.orientation.w = 1.0
    
    return start, target, current

def test_pos_ctrl_reached(sample_poses):
    """Test arrival determination in position control mode (success case)"""
    start, target, current = sample_poses
    target.stiffness_id = 0

    # Slightly inside the threshold
    current.pose.position.x = target.pose.pose.position.x - (config.POS_CTRL_REACHED_POSITION_THRESHOLD * 0.9)

    assert check_waypoint_reached_logic(target, current, start) == True

def test_pos_ctrl_not_reached(sample_poses):
    """Test arrival determination in position control mode (failure case)"""
    start, target, current = sample_poses
    target.stiffness_id = 0

    # Slightly outside the threshold
    current.pose.position.x = target.pose.pose.position.x - (config.POS_CTRL_REACHED_POSITION_THRESHOLD * 1.1)

    assert check_waypoint_reached_logic(target, current, start) == False

def test_adm_ctrl_reached(sample_poses):
    """Test arrival determination in admittance control mode (success case)"""
    start, target, current = sample_poses
    target.stiffness_id = 1

    # Set errors in insertion direction, lateral direction, and orientation within thresholds
    # Insertion direction (X) error
    current.pose.position.x = target.pose.pose.position.x + (config.ADM_CTRL_REACHED_INSERTION_THRESHOLD * 0.9)
    # Lateral direction (Y) error
    current.pose.position.y = target.pose.pose.position.y + (config.ADM_CTRL_REACHED_LATERAL_THRESHOLD * 0.9)
    # Orientation error
    q = R.from_euler('z', config.ADM_CTRL_REACHED_ORIENTATION_THRESHOLD * 0.9).as_quat()
    current.pose.orientation.x, current.pose.orientation.y, current.pose.orientation.z, current.pose.orientation.w = q

    assert check_waypoint_reached_logic(target, current, start) == True

def test_adm_ctrl_not_reached_by_lateral_error(sample_poses):
    """Test arrival determination in admittance control mode (failure due to lateral error)"""
    start, target, current = sample_poses
    target.stiffness_id = 1

    # Set lateral error outside threshold
    current.pose.position.y = target.pose.pose.position.y + (config.ADM_CTRL_REACHED_LATERAL_THRESHOLD * 1.1)

    assert check_waypoint_reached_logic(target, current, start) == False