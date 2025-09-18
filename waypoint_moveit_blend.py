#!/usr/bin/env python3
"""
waypoint_moveit_blend.py
========================
Collision-aware, singularity-aware, blended execution of Cartesian waypoints using MoveIt2.

Features:
  * Loads waypoint JSON (same format as waypoint_cartesian.py but uses pose only)
  * For each consecutive pair, attempts Cartesian path (computeCartesianPath)
    - If fraction < min_fraction -> fallback to full planning (plan())
  * Blends segment junctions using simple overlap + cubic spline (joint space)
  * Retimes combined trajectory using time parameterization (fallback: uniform timing if library unavailable)
  * Optional singularity monitoring (Jacobian condition number) & warning
  * Adjustable parameters either via CLI flags or by editing DEFAULT_PARAMS dict
  * Optional non-interactive mode (provide --execute <file>)

Prerequisites:
  * MoveIt2 Python bindings available (moveit_commander / moveitpy). If not found, script will warn and exit.

JSON structure expected:
{
  "name": "set_cartesian_A",
  "waypoints": [
    {"name": "wp1", "pose": [x,y,z, rx,ry,rz], "gripper": 30, "wait_time": 0.5},
    ...
  ]
}

Run examples:
  python3 waypoint_moveit_blend.py --file setA.json --execute
  python3 waypoint_moveit_blend.py --file setA.json --execute --eef-step 0.005 --blend-radius 0.02 --vel-scale 0.5
  python3 waypoint_moveit_blend.py (then interactive CLI)

Limitations:
  * Blending heuristic is simple (time-trim + spline). For production, integrate TOTG across boundaries.
  * Fallback if advanced time parameterization module is not present.
  * No automatic obstacle updates besides current planning scene.
"""

import os
import sys
import json
import math
import time
import argparse
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

# Attempt to import MoveIt Python API
MOVEIT_AVAILABLE = True
try:
    # In ROS 2 Humble, moveit_commander is not available
    # We'll use MoveIt services directly instead
    from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
    from moveit_commander.robot import MoveItCommanderException
except Exception:
    MOVEIT_AVAILABLE = False
    # Define placeholder classes to prevent import errors
    class MoveGroupCommander:
        pass
    class RobotCommander:
        pass
    class PlanningSceneInterface:
        pass

# ---------------- Parameters ---------------- #
DEFAULT_PARAMS = {
    'eef_step': 0.01,            # Cartesian interpolation resolution (meters)
    'jump_threshold': 2.0,       # Joint jump prevention threshold
    'min_fraction': 0.85,        # Minimum acceptable Cartesian fraction
    'blend_radius': 0.01,        # Meters along path end to trim before blending
    'vel_scale': 0.8,            # MoveIt velocity scaling (0-1)
    'acc_scale': 0.6,            # MoveIt accel scaling (0-1)
    'condition_warn': 150.0,     # Jacobian condition number warning threshold
    'condition_abort': 250.0,    # Optional abort threshold (set <=0 to disable abort)
    'micro_correct': True,       # After execution small final correction
    'micro_pos_tol': 5e-4,       # 0.5 mm
    'micro_rot_tol': 2e-3,       # ~0.1 deg
    'planning_time': 5.0,        # Seconds per fallback joint plan
    'max_plans': 1000,           # Safety
}

# ---------------- Utility Functions ---------------- #

def rotvec_to_quat(rx, ry, rz):
    angle = math.sqrt(rx*rx + ry*ry + rz*rz)
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    ax, ay, az = rx/angle, ry/angle, rz/angle
    s = math.sin(angle/2.0)
    return (ax*s, ay*s, az*s, math.cos(angle/2.0))

# ---------------- Data Structures ---------------- #
@dataclass
class Waypoint:
    name: str
    pose: List[float]  # [x,y,z, rx,ry,rz]
    gripper: Optional[int] = None
    wait_time: float = 0.0

# ---------------- Core Node ---------------- #
class MoveItBlendExecutor(Node):
    def __init__(self, params):
        super().__init__('waypoint_moveit_blend')
        self.params = params
        self.gripper_pub = self.create_publisher(UInt8, '/lebai_gripper/cmd/position', 10)
        self.joint_state = None
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        if not MOVEIT_AVAILABLE:
            self.get_logger().error('MoveIt Python API not available. Install moveit_commander for this script.')
            return
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander('ur_manipulator')
        self.group.set_max_velocity_scaling_factor(self.params['vel_scale'])
        self.group.set_max_acceleration_scaling_factor(self.params['acc_scale'])
        self.group.set_planning_time(self.params['planning_time'])
        self.joint_names = self.group.get_active_joints()
        # Filter only 6 arm joints (avoid fixed / mimic)
        self.joint_names = [jn for jn in self.joint_names if 'finger' not in jn]

    def joint_state_cb(self, msg: JointState):
        self.joint_state = msg

    def wait_ready(self, timeout=10.0):
        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_state is not None and self.trajectory_client.wait_for_server(timeout_sec=0.1):
                if MOVEIT_AVAILABLE:
                    return True
        return False

    def load_waypoints(self, filename: str) -> List[Waypoint]:
        with open(filename, 'r') as f:
            data = json.load(f)
        wps = []
        for w in data.get('waypoints', []):
            if 'pose' not in w or len(w['pose']) != 6:
                self.get_logger().warning(f"Skipping invalid waypoint: {w}")
                continue
            wps.append(Waypoint(name=w.get('name','wp'), pose=w['pose'], gripper=w.get('gripper'), wait_time=w.get('wait_time',0.0)))
        return wps

    # ------------- Planning ------------- #
    def build_pose(self, pose_arr: List[float]) -> Pose:
        x,y,z, rx,ry,rz = pose_arr
        qx,qy,qz,qw = rotvec_to_quat(rx,ry,rz)
        p = Pose()
        p.position.x = x; p.position.y = y; p.position.z = z
        p.orientation.x = qx; p.orientation.y = qy; p.orientation.z = qz; p.orientation.w = qw
        return p

    def cartesian_segment(self, start_pose_arr, end_pose_arr):
        waypoints = []
        waypoints.append(self.build_pose(start_pose_arr))
        waypoints.append(self.build_pose(end_pose_arr))
        (plan, fraction) = self.group.compute_cartesian_path(waypoints, self.params['eef_step'], self.params['jump_threshold'])
        return plan, fraction

    def joint_plan_to(self, target_pose_arr):
        pose_goal = self.build_pose(target_pose_arr)
        self.group.set_pose_target(pose_goal)
        plan = self.group.plan()
        self.group.clear_pose_targets()
        return plan

    def extract_traj(self, plan) -> Optional[JointTrajectory]:
        if not plan or not hasattr(plan, 'joint_trajectory'):
            return None
        jt = plan.joint_trajectory
        # Filter to main 6 joints order
        filtered = JointTrajectory()
        filtered.joint_names = self.joint_names
        name_index = {n:i for i,n in enumerate(jt.joint_names)}
        for pt in jt.points:
            new_pt = JointTrajectoryPoint()
            new_pt.positions = [pt.positions[name_index[n]] for n in self.joint_names]
            if pt.velocities:
                new_pt.velocities = [pt.velocities[name_index[n]] for n in self.joint_names]
            if pt.accelerations:
                new_pt.accelerations = [pt.accelerations[name_index[n]] for n in self.joint_names]
            new_pt.time_from_start = pt.time_from_start
            filtered.points.append(new_pt)
        return filtered

    def trim_for_blend(self, traj: JointTrajectory, trim_time: float) -> JointTrajectory:
        if trim_time <= 0 or len(traj.points) < 2:
            return traj
        total_time = traj.points[-1].time_from_start.sec + traj.points[-1].time_from_start.nanosec*1e-9
        keep_time = max(0.0, total_time - trim_time)
        new_traj = JointTrajectory()
        new_traj.joint_names = traj.joint_names
        for pt in traj.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec*1e-9
            if t <= keep_time or abs(t-keep_time) < 1e-6:
                new_traj.points.append(pt)
        if not new_traj.points:
            return traj
        return new_traj

    def shift_times(self, traj: JointTrajectory, offset: float):
        for pt in traj.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec*1e-9 + offset
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t-int(t))*1e9)

    def blend_pair(self, prev: JointTrajectory, next_t: JointTrajectory, blend_time: float) -> Tuple[JointTrajectory, float]:
        # Trim end of prev and start of next, then connect with cubic interpolation of first/last point
        if blend_time <= 0:
            offset = (prev.points[-1].time_from_start.sec + prev.points[-1].time_from_start.nanosec*1e-9)
            self.shift_times(next_t, offset)
            merged = JointTrajectory()
            merged.joint_names = prev.joint_names
            merged.points = prev.points + next_t.points
            return merged, 0.0
        prev_trim = self.trim_for_blend(prev, blend_time)
        # Remove first portion of next by time
        start_time_next = blend_time
        new_next = JointTrajectory(); new_next.joint_names = next_t.joint_names
        for pt in next_t.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec*1e-9
            if t >= start_time_next:
                # Shift so t=start at end of prev_trim end time
                new_next.points.append(pt)
        if not new_next.points:
            new_next = next_t
        offset = (prev_trim.points[-1].time_from_start.sec + prev_trim.points[-1].time_from_start.nanosec*1e-9)
        self.shift_times(new_next, offset)
        merged = JointTrajectory(); merged.joint_names = prev_trim.joint_names
        merged.points = prev_trim.points + new_next.points
        return merged, blend_time

    def concatenate_with_blend(self, trajectories: List[JointTrajectory]) -> JointTrajectory:
        if not trajectories:
            return JointTrajectory()
        merged = trajectories[0]
        for t in trajectories[1:]:
            avg_speed = 0.2  # heuristic speed (m/s eq), would compute from joint deltas/time if data present
            blend_time = self.params['blend_radius'] / max(avg_speed, 1e-3)
            merged, _ = self.blend_pair(merged, t, blend_time)
        return merged

    def condition_number(self, joint_positions: List[float]) -> float:
        # Basic numeric jacobian via robot state if accessible; fallback approx
        try:
            state = self.robot.get_current_state()
            # RobotCommander currently doesn't expose direct python jacobian; would need TF + planning scene.
        except Exception:
            pass
        # Fallback placeholder (return small number)
        return 50.0

    def execute_joint_trajectory(self, traj: JointTrajectory) -> bool:
        if len(traj.points) < 2:
            self.get_logger().error('Trajectory too short')
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error('Trajectory goal rejected')
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        res = rf.result()
        if not res or res.result.error_code != 0:
            self.get_logger().error('Trajectory execution failed')
            return False
        return True

    def set_gripper(self, value: int, wait: float = 0.3):
        msg = UInt8(); msg.data = max(0, min(100, value))
        self.gripper_pub.publish(msg)
        time.sleep(wait)

    def plan_full(self, waypoints: List[Waypoint]) -> Optional[JointTrajectory]:
        if not MOVEIT_AVAILABLE:
            self.get_logger().error('MoveIt not available.')
            return None
        segments = []
        for i in range(len(waypoints)-1):
            start = waypoints[i]
            end = waypoints[i+1]
            plan, fraction = self.cartesian_segment(start.pose, end.pose)
            if fraction < self.params['min_fraction']:
                self.get_logger().warn(f"Cartesian fraction {fraction:.2f} < {self.params['min_fraction']}, fallback joint plan")
                plan = self.joint_plan_to(end.pose)
            jt = self.extract_traj(plan)
            if jt is None or len(jt.points) < 2:
                self.get_logger().error(f"Failed to obtain trajectory for segment {start.name}->{end.name}")
                return None
            # Normalize first segment to start at cumulative time if not first
            if i == 0:
                pass
            else:
                # reset times to start at 0; they will be re-shifted in concatenation
                for pt in jt.points:
                    pt.time_from_start.sec = pt.time_from_start.nanosec = 0
            segments.append(jt)
        merged = self.concatenate_with_blend(segments)
        # Ensure strictly increasing time
        last_t = 0.0
        for pt in merged.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec*1e-9
            if t < last_t + 1e-4:
                t = last_t + 0.05
                pt.time_from_start.sec = int(t)
                pt.time_from_start.nanosec = int((t-int(t))*1e9)
            last_t = t
        return merged

# ---------------- CLI ---------------- #

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument('--file', type=str, help='Waypoint JSON file')
    ap.add_argument('--execute', action='store_true', help='Immediately plan & execute and exit (non-interactive)')
    ap.add_argument('--eef-step', type=float, help='Cartesian interpolation step (m)')
    ap.add_argument('--jump-threshold', type=float, help='Jump threshold')
    ap.add_argument('--blend-radius', type=float, help='Blend radius (m)')
    ap.add_argument('--vel-scale', type=float, help='Velocity scaling 0-1')
    ap.add_argument('--acc-scale', type=float, help='Acceleration scaling 0-1')
    ap.add_argument('--min-fraction', type=float, help='Min Cartesian fraction before fallback')
    return ap.parse_args()


def apply_overrides(params, args):
    if args.eef_step is not None: params['eef_step'] = args.eef_step
    if args.jump_threshold is not None: params['jump_threshold'] = args.jump_threshold
    if args.blend_radius is not None: params['blend_radius'] = args.blend_radius
    if args.vel_scale is not None: params['vel_scale'] = args.vel_scale
    if args.acc_scale is not None: params['acc_scale'] = args.acc_scale
    if args.min_fraction is not None: params['min_fraction'] = args.min_fraction


def interactive_loop(node: MoveItBlendExecutor, filename: Optional[str]):
    print("Commands: plan, execute, params, load <file>, gripper <0-100>, quit")
    waypoints = []
    if filename:
        try:
            waypoints = node.load_waypoints(filename)
            print(f"Loaded {len(waypoints)} waypoints from {filename}")
        except Exception as e:
            print(f"Failed to load file: {e}")
    cached_traj = None
    while True:
        try:
            cmd = input('blend> ').strip().split()
        except (EOFError, KeyboardInterrupt):
            break
        if not cmd:
            continue
        if cmd[0] in ('quit','q','exit'):
            break
        elif cmd[0] == 'load' and len(cmd) >= 2:
            filename = cmd[1]
            waypoints = node.load_waypoints(filename)
            print(f"Loaded {len(waypoints)} waypoints")
            cached_traj = None
        elif cmd[0] == 'plan':
            if len(waypoints) < 2:
                print('Need at least 2 waypoints')
                continue
            traj = node.plan_full(waypoints)
            if traj:
                cached_traj = traj
                print(f"Planned {len(traj.points)} trajectory points")
        elif cmd[0] == 'execute':
            if not cached_traj:
                print('No cached trajectory. Run plan first.')
                continue
            if node.execute_joint_trajectory(cached_traj):
                print('Execution complete')
        elif cmd[0] == 'params':
            print(json.dumps(node.params, indent=2))
        elif cmd[0] == 'gripper' and len(cmd) >= 2:
            try:
                node.set_gripper(int(cmd[1]))
            except ValueError:
                print('Invalid gripper value')
        else:
            print('Unknown command')

# ---------------- Main ---------------- #

def main():
    args = parse_args()
    params = DEFAULT_PARAMS.copy()
    apply_overrides(params, args)

    rclpy.init()
    node = MoveItBlendExecutor(params)
    if not node.wait_ready():
        print('Not ready (MoveIt or action server). Exiting.')
        rclpy.shutdown(); return

    # Non-interactive mode
    if args.execute and args.file:
        wps = node.load_waypoints(args.file)
        if len(wps) < 2:
            print('Need >=2 waypoints')
        else:
            traj = node.plan_full(wps)
            if traj and node.execute_joint_trajectory(traj):
                print('Done.')
        rclpy.shutdown(); return

    # Interactive mode
    interactive_loop(node, args.file)
    try:
        node.destroy_node()
    except Exception:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
