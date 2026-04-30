import os
import sys
import time
import torch
import numpy as np
from geo_gp_interfaces.msg import PromptTrajectory, PredictedTrajectory
from geometry_msgs.msg import Pose, Vector3

os.environ["DISPLAY"] = ""
sys.path.append("/home/user/geo-gp")

from config import Config
from geometry.metrics import geom_mse
from geometry.resample import resample_trajectory_6d_equal_dt
from gp.model import rollout_reference_6d
from skills.skill_library import SkillLibrary
from skills.skill_loader import load_skills_from_models
from utils.misc import (
    moving_average_centered_pos,
    moving_average_centered_6d,
    smooth_prediction_by_twist_6d,
)


class Predictor:
    def __init__(self, logger, config_path, model_dir):
        # Load config
        self.cfg = Config(config_path)
        self.k = self.cfg.sampling["k_hist"]
        self.sample_hz = self.cfg.sampling["sample_hz"]
        self.default_speed = self.cfg.sampling["default_speed"]
        self.smooth_win = self.cfg.sampling["smooth_win"]

        self.rollout_horizon = self.cfg.prediction["rollout_horizon"]
        self.mse_thresh = self.cfg.prediction["mse_thresh"]
        self.goal_stop_eps = self.cfg.prediction["goal_stop_eps"]
        self.max_start_jump = self.cfg.prediction["max_start_jump"]
        self.drop_k = self.cfg.prediction["drop_k"]
        self.max_retries = self.cfg.prediction["max_retries"]

        # Logger
        self.logger = logger
        self.logger.info("Loading skills...")

        # Load skill library
        self.skill_library = SkillLibrary()

        skills = load_skills_from_models(model_dir, mode="6d")

        for s in skills:
            self.skill_library.add_skill(s)

        self.logger.info(str(self.skill_library))

    @staticmethod
    def polyline_length(pts: np.ndarray) -> float:
        """
        Calculate the length of a polyline.

        Args:
            pts (np.ndarray): A numpy array of shape (N, 3) containing the (x, y, z) coordinates of the polyline.

        Returns:
            float: The length of the polyline.
        """
        pts = np.asarray(pts)
        
        if pts.shape[0] < 2:
            return 0.0

        segs = pts[1:] - pts[:-1]

        return float(np.linalg.norm(segs, axis=1).sum())

    def estimate_prompt_speed(self, prompt_msg: PromptTrajectory) -> float:
        """
        Estimate the speed of the prompt trajectory.

        Args:
            prompt_msg (PromptTrajectory): The input prompt trajectory.

        Returns:
            float: The estimated speed of the prompt trajectory.
        """
        prompt_pts, _, _ = self.prompt_to_numpy(prompt_msg)

        if prompt_pts.shape[0] < 2:
            return self.default_speed

        if len(prompt_msg.time_from_start) == len(prompt_msg.poses):
            duration = float(prompt_msg.time_from_start[-1] - prompt_msg.time_from_start[0])
            if duration > 1e-6:
                length = self.polyline_length(prompt_pts)
                if length > 1e-6:
                    return max(length / duration, 1e-3)

        return self.default_speed

    def prompt_to_numpy(self, msg: PromptTrajectory):
        """
        Convert a ROS PromptTrajectory message to a numpy array.

        Args:
            msg (PromptTrajectory): The input ROS PromptTrajectory message.

        Returns:
            tuple[np.ndarray, np.ndarray, np.ndarray | None]:
                positions array of shape (N, 3), quaternions array of shape (N, 4)
                in [w, x, y, z], and optional force array of shape (N, 3).
        """
        pts = []
        quats = []
        forces = []

        for p in msg.poses:
            pts.append([
                p.position.x,
                p.position.y,
                p.position.z
            ])
            quats.append([
                p.orientation.w,
                p.orientation.x,
                p.orientation.y,
                p.orientation.z,
            ])

        if len(msg.forces) == len(msg.poses):
            for f in msg.forces:
                forces.append([f.x, f.y, f.z])

        force_arr = np.array(forces) if forces else None
        return np.array(pts), np.array(quats), force_arr

    def numpy_to_predicted(
        self,
        ref_msg: PromptTrajectory,
        pts: np.ndarray,
        quats: np.ndarray = None,
        forces: np.ndarray = None,
        *,
        target_speed=None,
        skill_name="",
        success=True,
        confidence=1.0,
    ):
        """
        Convert a numpy array of predicted points to a ROS PredictedTrajectory message.

        Args:
            ref_msg (PromptTrajectory): The reference PromptTrajectory message to copy the header from.
            pts (np.ndarray): A numpy array of shape (N, 3) containing the (x, y, z) coordinates of the predicted trajectory.
            target_speed (float): The target speed of the predicted trajectory.
            skill_name (str): The name of the skill used to generate the predicted trajectory.
            success (bool): Whether the prediction was successful.
            confidence (float): The confidence in the prediction.

        Returns:
            PredictedTrajectory: A ROS PredictedTrajectory message containing the predicted trajectory.
        """
        out = PredictedTrajectory()

        # Header
        out.header = ref_msg.header
        out.header.frame_id = "world"

        out.skill_name = skill_name
        out.success = success
        out.confidence = confidence

        # Poses
        for i, p in enumerate(pts):
            pose = Pose()
            pose.position.x = float(p[0])
            pose.position.y = float(p[1])
            pose.position.z = float(p[2])

            if quats is not None and i < len(quats):
                q = quats[i]
                pose.orientation.w = float(q[0])
                pose.orientation.x = float(q[1])
                pose.orientation.y = float(q[2])
                pose.orientation.z = float(q[3])
            else:
                pose.orientation = ref_msg.poses[-1].orientation

            out.poses.append(pose)
            if forces is not None and i < len(forces):
                force = Vector3()
                force.x = float(forces[i][0])
                force.y = float(forces[i][1])
                force.z = float(forces[i][2])
                out.forces.append(force)

        if target_speed is None:
            target_speed = self.default_speed
        target_speed = max(float(target_speed), 1e-3)

        out.time_from_start = []
        if len(pts) > 0:
            t = 0.0
            out.time_from_start.append(t)
            for i in range(1, len(pts)):
                seg_len = float(np.linalg.norm(pts[i] - pts[i - 1]))
                dt = seg_len / target_speed
                t += dt
                out.time_from_start.append(t)

        return out

    def predict(self, prompt_msg: PromptTrajectory, predict_force: bool = True):
        """
        Predict the trajectory based on the prompt path.

        Args:
            prompt_msg (PromptTrajectory): The input prompt trajectory.

        Returns:
            PredictedTrajectory: The predicted trajectory.
        """
        t_predict_start = time.perf_counter()
        input_type = "spherical"
        output_type = "delta"

        target_speed = self.estimate_prompt_speed(prompt_msg)
        self.logger.info(f"Target execution speed from prompt: {target_speed:.4f} m/s")

        # 1) ROS Path → numpy
        probe, probe_quat, probe_force = self.prompt_to_numpy(prompt_msg)
        if not predict_force:
            probe_force = None

        if probe_force is not None:
            probe_eq, probe_quat_eq, probe_force_eq = resample_trajectory_6d_equal_dt(
                probe,
                probe_quat,
                sample_hz=self.sample_hz,
                speed=self.default_speed,
                points_force=probe_force)
        else:
            probe_eq, probe_quat_eq = resample_trajectory_6d_equal_dt(
                probe,
                probe_quat,
                sample_hz=self.sample_hz,
                speed=self.default_speed)
            probe_force_eq = None

        if len(probe_eq) < (self.k + 2):
            self.logger.info("[Predict] Not enough probe points")
            return self.numpy_to_predicted(
                prompt_msg,
                probe,
                probe_quat,
                probe_force,
                target_speed=target_speed,
                skill_name="",
                success=False,
                confidence=0.0
            )

        probe_eq = moving_average_centered_pos(probe_eq, self.smooth_win)
        probe_quat_eq = moving_average_centered_6d(probe_quat_eq, self.smooth_win)
        if probe_force_eq is not None:
            probe_force_eq = moving_average_centered_6d(probe_force_eq, self.smooth_win)

        # 2) Skill matching
        t_match_start = time.perf_counter()
        skill, (R, s, t, j_end) = self.skill_library.match(probe_eq, margin_pts=30, step=1)
        match_ms = (time.perf_counter() - t_match_start) * 1000.0

        ref_eq = skill.ref_eq
        model = skill.model

        self.logger.info(f"Matched skill: {skill.name} | matching={match_ms:.2f} ms")

        # 3) Transform to ref frame
        probe_in_ref = ((probe_eq - t) / s) @ R
        probe_goal = s * (ref_eq[-1] @ R.T) + t

        # 4) Rollout
        t_rollout_start = time.perf_counter()
        preds = None
        preds_quat = None
        preds_force = None

        for attempt in range(self.max_retries):
            cur_pos = probe_in_ref.copy()
            cur_quat = probe_quat_eq.copy()[:cur_pos.shape[0]]
            cur_force = None if probe_force_eq is None else probe_force_eq.copy()[:cur_pos.shape[0]]
            preds_world_pos = []
            preds_world_quat = []
            preds_world_force = []
            failed = False

            for step in range(self.rollout_horizon):
                tp = torch.tensor(cur_pos, dtype=torch.float32)
                tq = torch.tensor(cur_quat, dtype=torch.float32)
                tf = None if cur_force is None else torch.tensor(cur_force, dtype=torch.float32)

                preds_ref_pos, preds_quat, preds_ref_force, _, _, _, vars_ref = rollout_reference_6d(
                    model,
                    tp,
                    tq,
                    start_t=cur_pos.shape[0] - 1,
                    h=1,
                    k=self.k,
                    input_type=input_type,
                    output_type=output_type,
                    R_ref_probe=R,
                    traj_force=tf,
                )

                next_ref_pos = preds_ref_pos[-1].numpy()

                # Ref → Probe frame
                next_world_pos = s * (next_ref_pos @ R.T) + t
                next_world_quat = preds_quat[-1].numpy()

                preds_world_pos.append(next_world_pos)
                preds_world_quat.append(next_world_quat)
                if preds_ref_force is not None:
                    next_world_force = preds_ref_force[-1].numpy()
                    preds_world_force.append(next_world_force)

                cur_pos = np.vstack([cur_pos, next_ref_pos])
                cur_quat = np.vstack([cur_quat, next_world_quat[None, :]])
                if cur_force is not None and preds_ref_force is not None:
                    cur_force = np.vstack([cur_force, next_world_force[None, :]])

                # Truncation
                d = np.linalg.norm(next_world_pos - probe_goal)
                if d < self.goal_stop_eps and np.max(vars_ref) > 1e-3:
                    self.logger.info(
                        f"[Predict] Reached goal at step {step}, d={d:.4f}"
                    )
                    break

            # Geometric drift check
            mse_full = geom_mse(cur_pos, ref_eq, min(len(cur_pos), len(ref_eq)))
            self.logger.info(f"[GeomCheck] full mse = {mse_full:.4f}")

            if mse_full > self.mse_thresh:
                self.logger.info("[Recover] Geometric drift detected, retry...")
                failed = True

            if not failed:
                preds_world_pos = np.asarray(preds_world_pos)
                preds_world_quat = np.asarray(preds_world_quat)
                preds_world_force = (
                    np.asarray(preds_world_force) if len(preds_world_force) else None
                )

                probe_end = probe_eq[-1]
                dists = np.linalg.norm(preds_world_pos - probe_end, axis=1)
                candidate_idxs = np.where(dists < self.max_start_jump)[0]

                if len(candidate_idxs) == 0:
                    self.logger.info(
                        "[Recover] No prediction point close to probe end, retry..."
                    )
                    failed = True
                else:
                    i_start = int(candidate_idxs[0])
                    preds = preds_world_pos[i_start:]
                    preds_quat = preds_world_quat[i_start:]
                    preds_force = (
                        preds_world_force[i_start:] if preds_world_force is not None else None
                    )
                    break

            # Drop tail
            if probe_in_ref.shape[0] <= (self.k + self.drop_k):
                break

            probe_in_ref = probe_in_ref[:-self.drop_k]
            self.logger.info(
                f"[Recover] Dropping last {self.drop_k} probe points, retry {attempt + 1}"
            )

        if preds is None:
            self.logger.info("[Predict] All retries failed. No prediction output.")
            return self.numpy_to_predicted(
                prompt_msg,
                probe_eq,
                probe_quat_eq,
                probe_force_eq,
                target_speed=target_speed,
                skill_name=skill.name,
                success=False,
                confidence=0.0,
            )

        # 5) Smoothing
        preds, preds_quat = smooth_prediction_by_twist_6d(
            probe_pos=probe_eq,
            probe_quat=probe_quat_eq,
            pred_pos=preds,
            pred_quat=preds_quat,
            win=self.smooth_win,
            blend_first_step_pos=0.5,
            blend_first_step_rot=0.5,
        )
        if preds_force is not None:
            preds_force = moving_average_centered_6d(preds_force, self.smooth_win)

        rollout_ms = (time.perf_counter() - t_rollout_start) * 1000.0
        total_ms = (time.perf_counter() - t_predict_start) * 1000.0
        preds_force_shape = preds_force.shape if preds_force is not None else None
        self.logger.info(f"[Predict] Done. preds={preds.shape}, preds_force={preds_force_shape}")
        self.logger.info(
            f"[Timing] matching={match_ms:.2f} ms | rollout={rollout_ms:.2f} ms | total={total_ms:.2f} ms"
        )

        # 6) Numpy → ROS Path
        return self.numpy_to_predicted(
            prompt_msg,
            preds,
            preds_quat,
            preds_force,
            target_speed=target_speed,
            skill_name=skill.name,
            success=True,
            confidence=1.0,
        )
