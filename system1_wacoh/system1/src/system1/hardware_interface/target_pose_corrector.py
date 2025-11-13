import numpy as np
from typing import Optional
from scipy.spatial.transform import Rotation as R, Slerp
from segment_task_pkg.msg import StiffnessId


class TargetPoseCorrector:
    """Class that performs admittance control and position interpolation to calculate the next servo target pose to send to the robot."""

    def __init__(self, dt: float, adm_params: dict):
        self.dt = dt

        # Expand parameters
        self.inertia_inv = np.linalg.inv(adm_params["inertia"])
        self.stiffness_profiles = adm_params["stiffness_profiles"]
        self.damping_profiles = adm_params["damping_profiles"]
        self.force_deadband = adm_params["force_deadband"]
        self.torque_deadband = adm_params["torque_deadband"]
        self.bias_sample_count = adm_params.get("bias_sample_count", 0)

        # Step sizes for position control and admittance control
        self.pos_ctrl_linear_step  = adm_params["pos_ctrl_linear_step"]
        self.pos_ctrl_angular_step = adm_params["pos_ctrl_angular_step"]
        self.adm_ctrl_linear_step  = adm_params["adm_ctrl_linear_step"]
        self.adm_ctrl_angular_step = adm_params["adm_ctrl_angular_step"]

        # Maximum offsets for saturation
        self.max_pos_offset = adm_params["max_pos_offset"]
        self.max_rot_offset = adm_params["max_rot_offset"]

        # Off-diagonal coupling for stiffness_id == ADM_CONTROL_2
        self.k_task2_pz_ry = adm_params.get("k_task2_pz_ry", 0.0)
        self.k_task2_py_rz = adm_params.get("k_task2_py_rz", 0.0)

        # Internal state (tool coordinate frame)
        self.adm_pos_off = np.zeros(3)
        self.adm_rot_off = np.zeros(3)  # Rotation vector representation
        self.adm_vel_off = np.zeros(6)

    @staticmethod
    def normalize_quat(q: np.ndarray) -> np.ndarray:
        norm = np.linalg.norm(q)
        return q / norm if norm > 1e-9 else np.array([0.0, 0.0, 0.0, 1.0])

    def _get_task_frame_rotation(self,
                                    target_pos: np.ndarray,
                                    current_pos: np.ndarray) -> R:
        diff = target_pos - current_pos
        if np.linalg.norm(diff) < 1e-6:
            return R.identity()

        x_axis = diff / np.linalg.norm(diff)
        z_ref = np.array([0.0, 0.0, 1.0])
        y_axis = np.cross(z_ref, x_axis)
        if np.linalg.norm(y_axis) < 1e-6:
            # If parallel, change Y axis reference
            y_axis = np.cross([0.0, 1.0, 0.0], x_axis)
        y_axis /= np.linalg.norm(y_axis)
        z_axis = np.cross(x_axis, y_axis)

        # Create matrix so each column is an axis vector of the new coordinate frame
        mat = np.vstack([x_axis, y_axis, z_axis]).T
        return R.from_matrix(mat)

    def reset(self):
        """Clear internal offset state to zero."""
        self.adm_pos_off.fill(0.0)
        self.adm_rot_off.fill(0.0)
        self.adm_vel_off.fill(0.0)

    def update(self,
                current_pose: dict,
                target_pose: dict,
                wrench_base: np.ndarray,
                stiffness_id: int,
                wrench_bias_base: Optional[np.ndarray] = None
                ) -> dict:

        # 1) Select step size
        if stiffness_id == StiffnessId.POSITION_CONTROL:
            lin_step = self.pos_ctrl_linear_step
            ang_step = self.pos_ctrl_angular_step
        else:
            lin_step = self.adm_ctrl_linear_step
            ang_step = self.adm_ctrl_angular_step

        # 2) Interpolate the "ideal" next step position/rotation
        p_cur = current_pose['pos']
        q_cur = R.from_quat(self.normalize_quat(current_pose['quat']))
        p_tgt = target_pose['pos']
        q_tgt = R.from_quat(self.normalize_quat(target_pose['quat']))

        # Position interpolation
        err_p = p_tgt - p_cur
        dist = np.linalg.norm(err_p)
        alpha = 1.0 if dist < 1e-6 else min(1.0, lin_step / dist)
        next_pos_ideal = p_cur + alpha * err_p

        # Rotation interpolation
        angle = (q_tgt * q_cur.inv()).magnitude()
        beta = 1.0 if angle < 1e-6 else min(1.0, ang_step / angle)
        slerp = Slerp([0.0, 1.0], R.from_quat([q_cur.as_quat(), q_tgt.as_quat()]))
        next_rot_ideal = slerp([beta])[0]

        # If position mode, clear offset and return
        if stiffness_id == StiffnessId.POSITION_CONTROL:
            self.reset()
            return {
                'pos': next_pos_ideal,
                'quat': self.normalize_quat(next_rot_ideal.as_quat())
            }

        # 3) Admittance control: wrench transformation to tool frame + deadband
        w = np.array(wrench_base, dtype=float)
        if wrench_bias_base is not None:
            w -= np.array(wrench_bias_base, dtype=float)

        f_tool = q_cur.inv().apply(w[:3])
        t_tool = q_cur.inv().apply(w[3:])
        f_tool[np.abs(f_tool) < self.force_deadband]   = 0.0
        t_tool[np.abs(t_tool) < self.torque_deadband] = 0.0
        wrench_tool = np.hstack([f_tool, t_tool])

        # Get stiffness and damping matrices
        K = self.stiffness_profiles.get(stiffness_id)
        D = self.damping_profiles.get(stiffness_id)
        if K is None or D is None:
            # If profile is undefined, use position-control equivalent
            self.reset()
            return {
                'pos': next_pos_ideal,
                'quat': self.normalize_quat(next_rot_ideal.as_quat())
            }
        # Offset vector
        offset_pr = np.hstack([self.adm_pos_off, self.adm_rot_off])

        # Off-diagonal stiffness mode
        if stiffness_id == StiffnessId.ADMITTANCE_CONTROL_2:
            # --- Build task frame rotation matrix ---
            R_task = self._get_task_frame_rotation(p_tgt, p_cur)
            R_t2c  = R_task.inv() * q_cur     # base→task→tool
            R_c2t  = R_t2c.inv()

            f_tool = wrench_tool[:3]
            t_tool = wrench_tool[3:]
            f_task = R_t2c.apply(f_tool)
            t_task = R_t2c.apply(t_tool)
            w_task = np.hstack([f_task, t_task])

            off_lin = offset_pr[:3]
            off_ang = offset_pr[3:]
            off_lin_t = R_t2c.apply(off_lin)
            off_ang_t = R_t2c.apply(off_ang)
            off_task = np.hstack([off_lin_t, off_ang_t])

            vel_lin = self.adm_vel_off[:3]
            vel_ang = self.adm_vel_off[3:]
            vel_lin_t = R_t2c.apply(vel_lin)
            vel_ang_t = R_t2c.apply(vel_ang)
            vel_task = np.hstack([vel_lin_t, vel_ang_t])

            # Inject coupling into stiffness matrix
            K2 = K.copy()
            K2[2, 4] = K2[4, 2] = self.k_task2_pz_ry
            K2[1, 5] = K2[5, 1] = self.k_task2_py_rz

            # Acceleration in task coordinate frame
            acc_task = self.inertia_inv @ (w_task - D @ vel_task - K2 @ off_task)

            # Convert acceleration back to tool(base) coordinate frame
            acc_lin_tool = R_c2t.apply(acc_task[:3])
            acc_ang_tool = R_c2t.apply(acc_task[3:])
            acc_tool = np.hstack([acc_lin_tool, acc_ang_tool])

        else:
            # Standard admittance
            acc_tool = self.inertia_inv @ (wrench_tool - D @ self.adm_vel_off - K @ offset_pr)
        # 4) Integrate offsets and apply saturation
        self.adm_vel_off += acc_tool * self.dt
        self.adm_pos_off += self.adm_vel_off[:3] * self.dt
        self.adm_rot_off += self.adm_vel_off[3:] * self.dt

        # Apply magnitude limits
        pos_norm = np.linalg.norm(self.adm_pos_off)
        if pos_norm > self.max_pos_offset:
            self.adm_pos_off *= (self.max_pos_offset / pos_norm)

        rot_norm = np.linalg.norm(self.adm_rot_off)
        if rot_norm > self.max_rot_offset:
            self.adm_rot_off *= (self.max_rot_offset / rot_norm)

        # 5) Final target = ideal target + offset
        final_pos  = next_pos_ideal + next_rot_ideal.apply(self.adm_pos_off)
        final_rot  = next_rot_ideal * R.from_rotvec(self.adm_rot_off)

        return {
            'pos': final_pos,
            'quat': self.normalize_quat(final_rot.as_quat())
        }