import os
import sys
import torch
import numpy as np
from geo_gp_interfaces.msg import PromptTrajectory, PredictedTrajectory
from geometry_msgs.msg import Pose

os.environ["DISPLAY"] = ""
sys.path.append("/home/user/geo-gp")

from config import Config
from geometry.metrics import geom_mse
from geometry.resample import resample_trajectory_3d_equal_dt
from gp.model import rollout_reference_3d
from skills.skill_library import SkillLibrary
from skills.skill_loader import load_skills_from_models
from utils.misc import moving_average_centered_pos, smooth_prediction_by_velocity


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

        skills = load_skills_from_models(model_dir, mode="3d")

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
        prompt_pts = self.prompt_to_numpy(prompt_msg)

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
            np.ndarray: A numpy array of shape (N, 3) containing the (x, y, z) coordinates of the prompt trajectory.
        """
        pts = []

        for p in msg.poses:
            pts.append([
                p.position.x,
                p.position.y,
                p.position.z
            ])

        return np.array(pts)

    def numpy_to_predicted(
        self,
        ref_msg: PromptTrajectory,
        pts: np.ndarray,
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
        for p in pts:
            pose = Pose()
            pose.position.x = float(p[0])
            pose.position.y = float(p[1])
            pose.position.z = float(p[2])

            pose.orientation = ref_msg.poses[-1].orientation

            out.poses.append(pose)

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

    def predict(self, prompt_msg: PromptTrajectory):
        """
        Predict the trajectory based on the prompt path.

        Args:
            prompt_msg (PromptTrajectory): The input prompt trajectory.

        Returns:
            PredictedTrajectory: The predicted trajectory.
        """
        target_speed = self.estimate_prompt_speed(prompt_msg)
        self.logger.info(f"Target execution speed from prompt: {target_speed:.4f} m/s")

        # 1) ROS Path → numpy
        probe = self.prompt_to_numpy(prompt_msg)

        probe_eq = resample_trajectory_3d_equal_dt(
            probe,
            sample_hz=self.sample_hz,
            speed=self.default_speed)

        if len(probe_eq) < (self.k + 2):
            self.logger.info("[Predict] Not enough probe points")
            return self.numpy_to_predicted(
                prompt_msg,
                probe,
                target_speed=target_speed,
                skill_name="",
                success=False,
                confidence=0.0
            )

        probe_eq = moving_average_centered_pos(probe_eq, self.smooth_win)

        # 2) Skill matching
        skill, (R, s, t, j_end) = self.skill_library.match(probe_eq)

        ref_eq = skill.ref_eq
        model = skill.model

        self.logger.info(f"Matched skill: {skill.name}")

        # 3) Transform to ref frame
        probe_in_ref = ((probe_eq - t) / s) @ R
        probe_goal = s * (ref_eq[-1] @ R.T) + t

        # 4) Rollout
        preds = None

        for attempt in range(self.max_retries):
            cur_hist = probe_in_ref.copy()
            preds_world = []
            failed = False

            for step in range(self.rollout_horizon):
                preds_ref, _, _, vars_ref = rollout_reference_3d(
                    model,
                    torch.tensor(cur_hist, dtype=torch.float32),
                    start_t=cur_hist.shape[0] - 1,
                    h=1,
                    k=self.k,
                    input_type="spherical",
                    output_type="delta",
                )

                next_ref = preds_ref[-1].numpy()

                # Ref → Probe/World frame
                next_world = s * (next_ref @ R.T) + t
                preds_world.append(next_world)

                cur_hist = np.vstack([cur_hist, next_ref])

                # Goal stopping
                d = np.linalg.norm(next_world - probe_goal)

                if d < self.goal_stop_eps and np.max(vars_ref) > 1e-3:
                    self.logger.info(f"[Predict] Reached goal at step {step}")
                    break

            # Drift check
            mse_full = geom_mse(cur_hist, ref_eq, min(len(cur_hist), len(ref_eq)))

            if mse_full > self.mse_thresh:
                self.logger.info("[Recover] Drift detected, retry...")
                failed = True

            if not failed:
                preds_world = np.asarray(preds_world)

                probe_end = probe_eq[-1]
                dists = np.linalg.norm(preds_world - probe_end, axis=1)
                candidate_idxs = np.where(dists < self.max_start_jump)[0]

                if len(candidate_idxs) == 0:
                    self.logger.info("[Recover] No valid start point")
                    failed = True
                else:
                    i_start = int(candidate_idxs[0])
                    preds = preds_world[i_start:]
                    break

            # Retry: drop tail
            if probe_in_ref.shape[0] <= (self.k + self.drop_k):
                break

            probe_in_ref = probe_in_ref[:-self.drop_k]
            self.logger.info(f"[Recover] Retry {attempt+1}")

        if preds is None:
            self.logger.info("[Predict] Failed")
            return self.numpy_to_predicted(prompt_msg,
                probe_eq,
                target_speed=target_speed,
                skill_name=skill.name,
                success=False,
                confidence=0.0)

        # 5) Smoothing
        preds = smooth_prediction_by_velocity(
            probe=probe_eq,
            pred=preds,
            win=self.smooth_win,
            blend_first_step=0.5,
        )

        self.logger.info(f"[Predict] Done: {preds.shape}")

        # 6) Numpy → ROS Path
        return self.numpy_to_predicted(prompt_msg,
            preds,
            target_speed=target_speed,
            skill_name=skill.name,
            success=True,
            confidence=1.0)
