#!/usr/bin/env python3
"""
Waypoint Cartesian Controller
=============================

Provides an interactive CLI for storing and executing waypoints defined purely in TCP Cartesian pose
(x, y, z, rx, ry, rz) where rotation is axis-angle expressed as a Rodrigues vector (same format as
UR's RTDE pose / get_pos output). Internally, motions are executed by:

1. Continuously reading current joint states and TCP pose feedback (`/joint_states`, `/tcp_pose_broadcaster/pose`).
2. Performing damped least-squares (DLS) iterative inverse kinematics from the current joint state to the target pose.
3. Splitting the joint-space path (interpolated in Cartesian space) into a short joint trajectory and
   sending it through the scaled joint trajectory action server for smooth execution.
4. After reaching a waypoint, verifying actual TCP pose error and (optionally) applying a micro-correction step.

This does NOT use MoveIt's planning pipeline, but attempts to mimic straight-line (linear) Cartesian interpolation
in task space similar to MoveIt Cartesian path service. It does NOT perform collision checking.

JSON Waypoint Format (Cartesion):
{
  "name": "pick_above",
  "pose": [x, y, z, rx, ry, rz],
  "gripper": 30,           # optional (0-100)
  "wait_time": 1.0         # optional seconds
}

Top-level JSON file structure used for sequences:
{
  "name": "set_cartesian_A",
  "waypoints": [ { waypoint_obj }, ... ]
}

Commands:
    save <name> [gripper] [wait]                  - Save current TCP pose as waypoint
    pos                                           - Print current pose & gripper
    execute <file.json> [eef_step] [delay]        - Execute waypoints sequentially (legacy per-waypoint planning)
    execute_blend <file.json> [eef_step] [radius] [speed] - Blended multi-waypoint path (Cartesian interp + IK)
    create_set <file.json>                        - Initialize empty set file
    add_to_set <file.json> <name> [gripper] [wait]- Append current pose
    load <file.json>                              - Show summary
    gripper <value 0-100>                         - Move gripper
    params                                        - Show current tuning parameters
    quit                                          - Exit

Limitations:
  * No collision checking
  * Singularities handled by damping only
  * Assumes tool0 frame pose broadcaster matches RTDE pose semantics
  * Orientation interpolation linear in rotation vector (approximation)

Author: Generated helper utility.
"""

import math
import json
import time
from typing import List, Optional, Tuple
import argparse

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8
import numpy as np

# ------------------ Configuration Defaults (can edit here or override via CLI) ------------------ #
DEFAULT_EEF_STEP = 0.01         # meters for linear interpolation
DEFAULT_BLEND_RADIUS = 0.015    # meters (path distance window) for blending between waypoints
DEFAULT_SPEED_SCALE = 1.0       # 0-1 scaling for nominal segment timing (heuristic)
DEFAULT_MICRO_POS_TOL = 5e-4    # 0.5 mm
DEFAULT_MICRO_ROT_TOL = 2e-3    # ~0.1 deg
DEFAULT_MAX_MICRO_ATTEMPTS = 3
DEFAULT_MIN_SEG_POINTS = 2
DEFAULT_MAX_IK_ITERS = 220
DEFAULT_IK_DAMPING = 0.01

# ------------------ Utility Functions ------------------ #

def rotvec_to_quat(rx: float, ry: float, rz: float) -> Tuple[float, float, float, float]:
    angle = math.sqrt(rx*rx + ry*ry + rz*rz)
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    ax = rx / angle
    ay = ry / angle
    az = rz / angle
    s = math.sin(angle/2.0)
    return (ax*s, ay*s, az*s, math.cos(angle/2.0))

def quat_to_rotvec(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    # Ensure normalization
    n = math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    qx /= n; qy /= n; qz /= n; qw /= n
    angle = 2.0 * math.atan2(math.sqrt(qx*qx+qy*qy+qz*qz), qw)
    if angle < 1e-12:
        return (0.0, 0.0, 0.0)
    s = math.sin(angle/2.0)
    ax = qx / s
    ay = qy / s
    az = qz / s
    return (ax*angle, ay*angle, az*angle)

def interpolate_rotvec(r1: np.ndarray, r2: np.ndarray, t: float) -> np.ndarray:
    # Simple linear interpolation in rotation vector space (approx)
    return r1 + t*(r2 - r1)

# ------------------ UR5e Kinematics (simplified) ------------------ #
# DH parameters (UR standard) for UR5e: a, d, alpha
UR5_DH = [
    (0.0,       0.1625,  math.pi/2.0),   # Base to shoulder
    (-0.425,    0.0,     0.0),           # Shoulder to elbow  
    (-0.3922,   0.0,     0.0),           # Elbow to wrist_1
    (0.0,       0.1333,  math.pi/2.0),   # Wrist_1 to wrist_2
    (0.0,       0.0997, -math.pi/2.0),   # Wrist_2 to wrist_3
    (0.0,       0.0996,  0.0),           # Wrist_3 to tool0
]

def dh_transform(a, d, alpha, theta):
    ca = math.cos(alpha); sa = math.sin(alpha)
    ct = math.cos(theta); st = math.sin(theta)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0.0,    sa,     ca,    d],
        [0.0,   0.0,    0.0,  1.0]
    ])

def fk_ur5(q: List[float]) -> Tuple[np.ndarray, np.ndarray]:
    T = np.eye(4)
    for i, (a,d,alpha) in enumerate(UR5_DH):
        T = T @ dh_transform(a, d, alpha, q[i])
    pos = T[0:3, 3]
    # Convert rotation to rotvec
    R = T[0:3, 0:3]
    angle = math.acos(max(-1.0, min(1.0, (np.trace(R)-1)/2)))
    if angle < 1e-9:
        rotvec = np.zeros(3)
    else:
        rx = (R[2,1]-R[1,2])/(2*math.sin(angle))
        ry = (R[0,2]-R[2,0])/(2*math.sin(angle))
        rz = (R[1,0]-R[0,1])/(2*math.sin(angle))
        rotvec = np.array([rx, ry, rz]) * angle
    return pos, rotvec

def jacobian_numeric(q: List[float], eps: float = 1e-5) -> np.ndarray:
    # 6x6 numerical jacobian (pos + rotvec differential approx)
    J = np.zeros((6,6))
    pos0, rot0 = fk_ur5(q)
    for i in range(6):
        q_pert = list(q)
        q_pert[i] += eps
        pos1, rot1 = fk_ur5(q_pert)
        dp = (pos1 - pos0)/eps
        dR = (rot1 - rot0)/eps
        J[0:3, i] = dp
        J[3:6, i] = dR
    return J

def damped_least_squares_ik(q_start: List[float], target_pos: np.ndarray, target_rotvec: np.ndarray,
                             max_iters: int = 200, pos_tol: float = 1e-4, rot_tol: float = 1e-3,
                             damping: float = 0.01) -> Optional[List[float]]:
    q = np.array(q_start, dtype=float)
    for _ in range(max_iters):
        cur_pos, cur_rot = fk_ur5(q)
        dp = target_pos - cur_pos
        dr = target_rotvec - cur_rot
        if np.linalg.norm(dp) < pos_tol and np.linalg.norm(dr) < rot_tol:
            return q.tolist()
        J = jacobian_numeric(q)
        JT = J.T
        A = J @ JT + (damping**2)*np.eye(6)
        dq = JT @ np.linalg.solve(A, np.hstack((dp, dr)))
        q += dq
    return None

# ------------------ Main Node ------------------ #
class CartesianWaypointController(Node):
    def __init__(self, args=None):
        super().__init__('waypoint_cartesian_controller')
        self.action_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(PoseStamped, '/tcp_pose_broadcaster/pose', self.pose_cb, 10)
        self.gripper_pub = self.create_publisher(UInt8, '/lebai_gripper/cmd/position', 10)
        self.gripper_state_sub = self.create_subscription(UInt8, '/lebai_gripper/state/position', self.gripper_state_cb, 10)

        self.current_joints = None
        self.current_pose = None  # geometry_msgs/Pose
        self.current_gripper = None

        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        # Parameter set (from CLI or defaults)
        self.params = {
            'eef_step': DEFAULT_EEF_STEP if not args or args.eef_step is None else args.eef_step,
            'blend_radius': DEFAULT_BLEND_RADIUS if not args or args.blend_radius is None else args.blend_radius,
            'speed_scale': DEFAULT_SPEED_SCALE if not args or args.speed_scale is None else args.speed_scale,
            'micro_pos_tol': DEFAULT_MICRO_POS_TOL,
            'micro_rot_tol': DEFAULT_MICRO_ROT_TOL,
            'micro_attempts': DEFAULT_MAX_MICRO_ATTEMPTS,
            'ik_damping': DEFAULT_IK_DAMPING,
            'ik_max_iters': DEFAULT_MAX_IK_ITERS,
            'min_seg_points': DEFAULT_MIN_SEG_POINTS,
        }
        print("[Cartesian] Controller initialized (blending capable)")

    def joint_cb(self, msg: JointState):
        if len(msg.name) >= 6:
            jd = dict(zip(msg.name, msg.position))
            self.current_joints = [jd.get(n, 0.0) for n in self.joint_names]

    def pose_cb(self, msg: PoseStamped):
        self.current_pose = msg.pose

    def gripper_state_cb(self, msg: UInt8):
        self.current_gripper = msg.data

    def wait_ready(self, timeout=10.0):
        print(" Waiting for action server...")
        if not self.action_client.wait_for_server(timeout_sec=timeout):
            print(" Action server not available")
            return False
        start = time.time()
        while (self.current_joints is None or self.current_pose is None) and (time.time()-start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.current_joints is None or self.current_pose is None:
            print(" Failed to receive joint states or pose")
            return False
        print(" Ready.")
        return True

    # -------------- Waypoint Persistence -------------- #
    def save_waypoint(self, name: str, gripper: Optional[int] = None, wait: float = 0.0):
        if self.current_pose is None:
            print(" No current pose available")
            return None
        # Convert current pose quaternion to rotvec (rx, ry, rz)
        rx, ry, rz = quat_to_rotvec(self.current_pose.orientation.x, self.current_pose.orientation.y,
                                    self.current_pose.orientation.z, self.current_pose.orientation.w)
        pose_arr = [self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z, rx, ry, rz]
        wp = {"name": name, "pose": pose_arr}
        if gripper is not None:
            wp["gripper"] = gripper
        if wait and wait > 0:
            wp["wait_time"] = wait
        print(f" Saved waypoint {name}: {pose_arr}")
        return wp

    def create_set(self, filename: str):
        data = {"name": filename, "waypoints": []}
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        print(f" Created empty set {filename}")

    def add_to_set(self, filename: str, name: str, gripper: Optional[int], wait: float):
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
        except FileNotFoundError:
            print(" File not found. Use create_set first.")
            return
        wp = self.save_waypoint(name, gripper, wait)
        if wp:
            data.setdefault('waypoints', []).append(wp)
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            print(f" Added waypoint {name} to {filename}")

    def load_set(self, filename: str):
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            print(f" Loaded {filename}: {len(data.get('waypoints', []))} waypoints")
            return data
        except Exception as e:
            print(f" Failed to load set: {e}")
            return None

    # -------------- Motion Generation -------------- #
    def execute_set(self, data: dict, eef_step: float = 0.01, inter_delay: float = 0.2):
        wps = data.get('waypoints', [])
        if not wps:
            print(" No waypoints")
            return False
        print(f" Executing {len(wps)} Cartesian waypoints (eef_step={eef_step}m)")
        for i, wp in enumerate(wps, 1):
            pose = wp.get('pose')
            if not pose or len(pose) != 6:
                print(f" Invalid pose in waypoint {wp.get('name','?')}, skipping")
                continue
            target_pos = np.array(pose[0:3])
            target_rot = np.array(pose[3:6])
            grip = wp.get('gripper')
            wait_time = wp.get('wait_time', 0.0)
            print(f"  [{i}/{len(wps)}] {wp.get('name','wp')} -> {pose}")
            if grip is not None:
                self.set_gripper(grip, 0.2)
            if not self.move_cartesian(target_pos, target_rot, eef_step=eef_step):
                print("   Failed to reach waypoint, aborting sequence")
                return False
            if wait_time > 0:
                time.sleep(wait_time)
            if inter_delay > 0 and i < len(wps):
                time.sleep(inter_delay)
        print(" Sequence complete")
        return True

    def move_cartesian(self, target_pos: np.ndarray, target_rot: np.ndarray, eef_step: float = 0.01,
                       max_attempts: int = 3, micro_tolerance_pos: float = 5e-4, micro_tolerance_rot: float = 2e-3,
                       start_q_override: Optional[List[float]] = None):
        if self.current_joints is None or self.current_pose is None:
            print(" Not ready")
            return False
        # Current start
        start_q = self.current_joints[:] if start_q_override is None else start_q_override[:]
        rx, ry, rz = quat_to_rotvec(self.current_pose.orientation.x, self.current_pose.orientation.y,
                                    self.current_pose.orientation.z, self.current_pose.orientation.w)
        start_pos = np.array([self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z])
        start_rot = np.array([rx, ry, rz])
        # Path length in Cartesian (position only) for step segmentation
        dist = np.linalg.norm(target_pos - start_pos)
        steps = max(1, int(dist / eef_step))
        qs = [start_q]
        last_q = start_q
        for s in range(1, steps+1):
            t = s/steps
            interp_pos = start_pos + t*(target_pos - start_pos)
            interp_rot = interpolate_rotvec(start_rot, target_rot, t)
            q_sol = damped_least_squares_ik(last_q, interp_pos, interp_rot)
            if q_sol is None:
                print(f"    IK failed at segment {s}/{steps}")
                return False
            qs.append(q_sol)
            last_q = q_sol
        # Build trajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        time_accum = 0.0
        base_dt = 0.25 * max(0.05, dist / max(steps,1))
        dt = base_dt / max(1e-6, self.params['speed_scale'])
        for idx, q in enumerate(qs):
            pt = JointTrajectoryPoint()
            pt.positions = q
            pt.velocities = [0.0]*6
            pt.time_from_start.sec = int(time_accum)
            pt.time_from_start.nanosec = int((time_accum - int(time_accum))*1e9)
            goal.trajectory.points.append(pt)
            time_accum += dt
        # Send
        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if gh is None or not gh.accepted:
            print("  Trajectory goal rejected")
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result()
        if res is None or res.result.error_code != 0:
            print("  Trajectory execution failed")
            return False
        # Refresh feedback
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
        # Verify
        if self.current_pose is not None:
            cur_pos = np.array([self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z])
            crx, cry, crz = quat_to_rotvec(self.current_pose.orientation.x, self.current_pose.orientation.y,
                                           self.current_pose.orientation.z, self.current_pose.orientation.w)
            cur_rot = np.array([crx, cry, crz])
            perr = np.linalg.norm(target_pos - cur_pos)
            rerr = np.linalg.norm(target_rot - cur_rot)
            print(f"   Final pose error pos={perr*1000:.2f} mm rot={rerr:.4f} rad")
            # Micro-correction if out of tolerance
            attempt = 0
            while (perr > micro_tolerance_pos or rerr > micro_tolerance_rot) and attempt < max_attempts:
                attempt += 1
                print(f"    Micro-correction attempt {attempt}")
                q_sol = damped_least_squares_ik(self.current_joints, target_pos, target_rot)
                if q_sol is None:
                    break
                micro_goal = FollowJointTrajectory.Goal()
                micro_goal.trajectory.joint_names = self.joint_names
                sp = JointTrajectoryPoint()
                sp.positions = self.current_joints
                sp.time_from_start.sec = 0; sp.time_from_start.nanosec = 0
                ep = JointTrajectoryPoint()
                ep.positions = q_sol
                ep.time_from_start.sec = 0
                ep.time_from_start.nanosec = int(0.4 * 1e9)
                micro_goal.trajectory.points = [sp, ep]
                fut = self.action_client.send_goal_async(micro_goal)
                rclpy.spin_until_future_complete(self, fut)
                g2 = fut.result()
                if g2 and g2.accepted:
                    rf = g2.get_result_async(); rclpy.spin_until_future_complete(self, rf)
                for _ in range(6):
                    rclpy.spin_once(self, timeout_sec=0.05)
                cur_pos = np.array([self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z])
                crx, cry, crz = quat_to_rotvec(self.current_pose.orientation.x, self.current_pose.orientation.y,
                                                self.current_pose.orientation.z, self.current_pose.orientation.w)
                cur_rot = np.array([crx, cry, crz])
                perr = np.linalg.norm(target_pos - cur_pos)
                rerr = np.linalg.norm(target_rot - cur_rot)
                print(f"     -> error now pos={perr*1000:.2f} mm rot={rerr:.4f} rad")
            return perr < 3*micro_tolerance_pos and rerr < 3*micro_tolerance_rot
        return True

    def set_gripper(self, value: int, wait: float = 0.3):
        if value < 0 or value > 100:
            print(" Gripper value 0-100")
            return
        msg = UInt8(); msg.data = value
        self.gripper_pub.publish(msg)
        time.sleep(wait)

    # ----------- Blended Multi-Waypoint Execution (single composite trajectory) ----------- #
    def execute_set_blended(self, data: dict, eef_step: Optional[float] = None, blend_radius: Optional[float] = None):
        wps = data.get('waypoints', [])
        if len(wps) < 2:
            print(" Need at least 2 waypoints for blended execution")
            return False
        eef_step = eef_step if eef_step is not None else self.params['eef_step']
        blend_radius = blend_radius if blend_radius is not None else self.params['blend_radius']
        print(f" Blended execution: {len(wps)} waypoints | eef_step={eef_step} | blend_radius={blend_radius}")

        # Build per-segment joint waypoint lists first.
        segment_joint_paths: List[List[List[float]]] = []
        previous_final_q = self.current_joints[:]
        for i in range(len(wps)-1):
            a = wps[i]; b = wps[i+1]
            pose_a = np.array(a['pose'][0:3]); rot_a = np.array(a['pose'][3:6])
            pose_b = np.array(b['pose'][0:3]); rot_b = np.array(b['pose'][3:6])
            # Interpolate path samples
            dist = np.linalg.norm(pose_b - pose_a)
            steps = max(1, int(dist / eef_step))
            joint_seq = [previous_final_q]
            last_q = previous_final_q
            ik_fail = False
            for s in range(1, steps+1):
                t = s/steps
                ipos = pose_a + t*(pose_b - pose_a)
                irot = interpolate_rotvec(rot_a, rot_b, t)
                q_sol = damped_least_squares_ik(last_q, ipos, irot,
                                                max_iters=self.params['ik_max_iters'], damping=self.params['ik_damping'])
                if q_sol is None:
                    print(f"  IK fail segment {i} step {s}/{steps}")
                    ik_fail = True
                    break
                joint_seq.append(q_sol)
                last_q = q_sol
            if ik_fail:
                return False
            previous_final_q = joint_seq[-1]
            segment_joint_paths.append(joint_seq)
        # Blend: remove overlapping tail/head based on blend radius (distance in cartesian position approximated via joint index near end).
        # Simple heuristic: drop last N points of each segment except final, where N = ceil(blend_radius / eef_step)
        trim_points = max(1, int(blend_radius / max(1e-6, eef_step)))
        composite: List[List[float]] = []
        for si, seg in enumerate(segment_joint_paths):
            if si < len(segment_joint_paths)-1 and len(seg) > trim_points:
                composite.extend(seg[:-trim_points])
            else:
                composite.extend(seg)
        # Build single trajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        time_accum = 0.0
        # Estimate per-point dt based on eef_step and speed_scale
        base_dt = 0.15 * (eef_step / 0.01)
        dt = base_dt / max(1e-6, self.params['speed_scale'])
        for q in composite:
            pt = JointTrajectoryPoint()
            pt.positions = q
            pt.velocities = [0.0]*6
            pt.time_from_start.sec = int(time_accum)
            pt.time_from_start.nanosec = int((time_accum - int(time_accum))*1e9)
            goal.trajectory.points.append(pt)
            time_accum += dt
        print(f"  Composite points: {len(goal.trajectory.points)}")
        # Send once
        fut = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh or not gh.accepted:
            print("  Blended trajectory goal rejected")
            return False
        rf = gh.get_result_async(); rclpy.spin_until_future_complete(self, rf)
        rres = rf.result()
        if not rres or rres.result.error_code != 0:
            print("  Blended execution failed")
            return False
        print("  Blended execution complete")
        return True

    def print_current(self):
        if self.current_pose is None:
            print(" No pose")
            return
        rx, ry, rz = quat_to_rotvec(self.current_pose.orientation.x, self.current_pose.orientation.y,
                                    self.current_pose.orientation.z, self.current_pose.orientation.w)
        print(f"Pose: [{self.current_pose.position.x:.4f}, {self.current_pose.position.y:.4f}, {self.current_pose.position.z:.4f}, {rx:.4f}, {ry:.4f}, {rz:.4f}]  Gripper: {self.current_gripper}")

# ------------------ CLI Loop ------------------ #
def parse_args():
    parser = argparse.ArgumentParser(description="Cartesian Waypoint Controller (with blending)")
    parser.add_argument('--eef-step', type=float, help='Interpolation step (m)')
    parser.add_argument('--blend-radius', type=float, help='Blend radius (m) for blended execution')
    parser.add_argument('--speed-scale', type=float, help='Speed scaling (0-1, affects timing heuristic)')
    parser.add_argument('--file', type=str, help='If provided with --auto-execute, will run on startup')
    parser.add_argument('--auto-execute', action='store_true', help='Run blended execution then exit (non-interactive)')
    parser.add_argument('--mode', choices=['blend','sequential'], default='blend', help='Execution mode for auto-execute')
    return parser.parse_args()

def main():
    args = parse_args()
    rclpy.init()
    node = CartesianWaypointController(args)
    if not node.wait_ready():
        return

    # Non-interactive mode path
    if args.auto_execute and args.file:
        data = node.load_set(args.file)
        if data:
            if args.mode == 'blend':
                node.execute_set_blended(data, eef_step=args.eef_step, blend_radius=args.blend_radius)
            else:
                node.execute_set(data, eef_step=(args.eef_step or node.params['eef_step']))
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()
        return

    print("\n[Cartesian] Commands: save, pos, execute, execute_blend, create_set, add_to_set, load, gripper, params, quit")
    while True:
        try:
            cmd = input("cartesian> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not cmd:
            continue
        parts = cmd.split()
        op = parts[0].lower()
        if op == 'quit' or op == 'q':
            break
        elif op == 'pos':
            node.print_current()
        elif op == 'save':
            if len(parts) < 2:
                print("Usage: save <name> [gripper] [wait]")
                continue
            name = parts[1]
            grip = int(parts[2]) if len(parts) >=3 else None
            wait = float(parts[3]) if len(parts) >=4 else 0.0
            wp = node.save_waypoint(name, grip, wait)
            if wp:
                print(json.dumps(wp, indent=2))
        elif op == 'create_set':
            if len(parts) < 2:
                print("Usage: create_set <file.json>")
                continue
            node.create_set(parts[1])
        elif op == 'add_to_set':
            if len(parts) < 3:
                print("Usage: add_to_set <file.json> <name> [gripper] [wait]")
                continue
            file = parts[1]; name = parts[2]
            grip = int(parts[3]) if len(parts) >=4 else None
            wait = float(parts[4]) if len(parts) >=5 else 0.0
            node.add_to_set(file, name, grip, wait)
        elif op == 'load':
            if len(parts) < 2:
                print("Usage: load <file.json>")
                continue
            node.load_set(parts[1])
        elif op == 'gripper':
            if len(parts) < 2:
                print("Usage: gripper <0-100>")
                continue
            node.set_gripper(int(parts[1]))
        elif op == 'execute':
            if len(parts) < 2:
                print("Usage: execute <file.json> [eef_step] [delay]")
                continue
            file = parts[1]
            eef_step = float(parts[2]) if len(parts) >=3 else 0.01
            delay = float(parts[3]) if len(parts) >=4 else 0.2
            data = node.load_set(file)
            if data:
                node.execute_set(data, eef_step=eef_step, inter_delay=delay)
        elif op == 'execute_blend':
            if len(parts) < 2:
                print("Usage: execute_blend <file.json> [eef_step] [blend_radius] [speed]")
                continue
            file = parts[1]
            eef_step = float(parts[2]) if len(parts) >=3 else node.params['eef_step']
            blend_radius = float(parts[3]) if len(parts) >=4 else node.params['blend_radius']
            speed = float(parts[4]) if len(parts) >=5 else node.params['speed_scale']
            node.params['speed_scale'] = max(1e-3, min(5.0, speed))
            data = node.load_set(file)
            if data:
                node.execute_set_blended(data, eef_step=eef_step, blend_radius=blend_radius)
        elif op == 'params':
            print(json.dumps(node.params, indent=2))
        else:
            print(" Unknown command")
    try:
        node.destroy_node()
    except Exception:
        pass
    rclpy.shutdown()
    print(" Exited Cartesian controller.")

if __name__ == '__main__':
    main()
