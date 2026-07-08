import csv
import json
import os
import sys
import time
import math
import torch
import numpy as np
from geo_gp_interfaces.msg import PromptTrajectory, PredictedTrajectory
from geometry_msgs.msg import Pose, Vector3

os.environ["DISPLAY"] = ""
sys.path.append("/home/user/geo-gp")

from config import Config
from geometry.metrics import geom_mse
from geometry.frame6d import estimate_rotation_scale_3d_search_by_count
from geometry.resample import resample_trajectory_6d_equal_dt
from gp.model import rollout_reference_6d
from skills.skill_library import SkillLibrary
from skills.skill_loader import load_skills_from_models
from utils.misc import (
    moving_average_centered_pos,
    moving_average_centered_6d,
    smooth_prediction_by_twist_6d,
)


def save_predicted_trajectory_to_csv(filepath: str, pred: PredictedTrajectory) -> None:
    """
    Save a predicted trajectory to a CSV file.

    Args:
        filepath (str): The path to the CSV file to save the predicted trajectory to.
        pred (PredictedTrajectory): The predicted trajectory to save.
    """
    has_force = len(pred.forces) == len(pred.poses) and len(pred.poses) > 0
    header = ["time", "x", "y", "z", "qx", "qy", "qz", "qw"]
    if has_force:
        header.extend(["fx", "fy", "fz"])

    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i, pose in enumerate(pred.poses):
            t = pred.time_from_start[i] if i < len(pred.time_from_start) else float(i)
            row = [
                round(float(t), 4),
                round(float(pose.position.x), 6),
                round(float(pose.position.y), 6),
                round(float(pose.position.z), 6),
                round(float(pose.orientation.x), 6),
                round(float(pose.orientation.y), 6),
                round(float(pose.orientation.z), 6),
                round(float(pose.orientation.w), 6),
            ]
            if has_force:
                force = pred.forces[i]
                row.extend([
                    round(float(force.x), 6),
                    round(float(force.y), 6),
                    round(float(force.z), 6),
                ])
            writer.writerow(row)


def save_prompt_trajectory_to_csv(filepath: str, prompt: PromptTrajectory) -> None:
    """
    Save a prompt trajectory to a CSV file.

    Args:
        filepath (str): The path to the CSV file to save the prompt trajectory to.
        prompt (PromptTrajectory): The prompt trajectory to save.
    """
    has_force = len(prompt.forces) == len(prompt.poses) and len(prompt.poses) > 0
    header = ["time", "x", "y", "z", "qx", "qy", "qz", "qw"]
    if has_force:
        header.extend(["fx", "fy", "fz"])

    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i, pose in enumerate(prompt.poses):
            t = prompt.time_from_start[i] if i < len(prompt.time_from_start) else float(i)
            row = [
                round(float(t), 4),
                round(float(pose.position.x), 6),
                round(float(pose.position.y), 6),
                round(float(pose.position.z), 6),
                round(float(pose.orientation.x), 6),
                round(float(pose.orientation.y), 6),
                round(float(pose.orientation.z), 6),
                round(float(pose.orientation.w), 6),
            ]
            if has_force:
                force = prompt.forces[i]
                row.extend([
                    round(float(force.x), 6),
                    round(float(force.y), 6),
                    round(float(force.z), 6),
                ])
            writer.writerow(row)


def save_similarity_transform_to_json(filepath: str, ctx: dict) -> None:
    """
    Save skill-matching similarity transform (R, s, t) and related metadata to JSON.

    Args:
        filepath (str): The path to the JSON file to save the similarity transform to.
        ctx (dict): The context dictionary containing the similarity transform and related metadata.
    """
    R = np.asarray(ctx["R"], dtype=np.float64)
    t_vec = np.asarray(ctx["t"], dtype=np.float64).reshape(-1)
    payload = {
        "skill_name": ctx["skill"].name,
        "match_ms": round(float(ctx["match_ms"]), 3),
        "j_end": int(ctx["j_end"]),
        "s": round(float(ctx["s"]), 8),
        "t": [round(float(v), 8) for v in t_vec.tolist()],
        "R": [[round(float(v), 8) for v in row] for row in R.tolist()],
    }
    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
        f.write("\n")


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
        self.skill_confidence_temperature = float(
            self.cfg.prediction.get(
                "skill_confidence_temperature",
                max(self.mse_thresh, 1e-6),
            )
        )
        self.skill_confidence_mse_temperature = float(
            self.cfg.prediction.get(
                "skill_confidence_mse_temperature",
                1e-3,
            )
        )
        self.skill_confidence_ratio_temperature = float(
            self.cfg.prediction.get("skill_confidence_ratio_temperature", 0.2)
        )
        self.skill_confidence_min = float(
            self.cfg.prediction.get("skill_confidence_min", 0.5)
        )
        self.goal_stop_eps = self.cfg.prediction["goal_stop_eps"]
        self.max_start_jump = self.cfg.prediction["max_start_jump"]
        self.drop_k = self.cfg.prediction["drop_k"]
        self.max_retries = self.cfg.prediction["max_retries"]
        self.last_prediction_context = None

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
        skill_confidence=1.0,
        variance_mean=0.0,
        variance_means=None,
        chunk_error=0.0,
        progress=0.0,
    ):
        """
        Convert a numpy array of predicted points to a ROS PredictedTrajectory message.

        Args:
            ref_msg (PromptTrajectory): The reference PromptTrajectory message to copy the header from.
            pts (np.ndarray): A numpy array of shape (N, 3) containing the (x, y, z) coordinates of the predicted trajectory.
            quats (np.ndarray): A numpy array of shape (N, 4) containing the quaternions of the predicted trajectory.
            forces (np.ndarray): A numpy array of shape (N, 3) containing the forces of the predicted trajectory.
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
        out.skill_confidence = skill_confidence
        out.variance_mean = variance_mean
        if variance_means is not None:
            out.variance_means = [float(v) for v in variance_means]
        out.chunk_error = chunk_error
        out.progress = progress

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

    @staticmethod
    def per_point_variance_means(vars_ref):
        """
        Compute the mean variance for each point in the predicted trajectory.

        Args:
            vars_ref (np.ndarray): A numpy array of shape (N, 3) containing the variance for each point in the predicted trajectory.

        Returns:
            list[float]: A list of mean variances for each point in the predicted trajectory.
        """
        arr = np.asarray(vars_ref, dtype=np.float64)
        if arr.size == 0:
            return []
        if arr.ndim == 1:
            return [float(v) for v in arr]
        return [float(v) for v in np.mean(arr, axis=1)]

    def match_skill_with_confidence(self, probe_eq: np.ndarray):
        """
        Match the input probe trajectory to the skill library 
        and return the best matching skill along with its alignment parameters and confidence.

        Args:
            probe_eq (np.ndarray): The input probe trajectory as a numpy array of shape (N, 3).

        Returns:
            tuple: A tuple containing the best matching skill, 
            its alignment parameters (R, s, t, j_end), the confidence in the match, 
            and the RMSE of the match.
        """
        if len(self.skill_library.skills) == 0:
            raise RuntimeError("SkillLibrary is empty")

        def skill_family(name: str) -> str:
            parts = name.rsplit("_", 1)
            if len(parts) == 2 and parts[1].isdigit():
                return parts[0]
            return name

        matches_by_family = {}
        for skill in self.skill_library.skills:
            ref_eq = skill.ref_eq
            R, s, t, j_end, rmse = estimate_rotation_scale_3d_search_by_count(
                ref_eq,
                probe_eq,
                margin_pts=1000,
                step=15,
            )[:5]
            rmse = float(rmse)
            family = skill_family(skill.name)
            self.logger.info(
                f"[SkillMatch] {skill.name} family={family} rmse={rmse:.6f}"
            )
            match = (skill, (R, s, t, j_end), rmse)
            if family not in matches_by_family or rmse < matches_by_family[family][2]:
                matches_by_family[family] = match

        ranked = sorted(matches_by_family.items(), key=lambda item: item[1][2])
        best_family, (best_skill, best_align, best_rmse) = ranked[0]
        best_mse = best_rmse ** 2

        if len(ranked) > 1:
            second_family, (_, _, second_rmse) = ranked[1]
            second_mse = second_rmse ** 2
            mse_gap = max(0.0, second_mse - best_mse)
            mse_ratio = second_mse / max(best_mse, 1e-12)
            c_gap = 1.0 - math.exp(
                -mse_gap / max(self.skill_confidence_mse_temperature, 1e-12)
            )
            c_ratio = 1.0 - math.exp(
                -max(0.0, mse_ratio - 1.0)
                / max(self.skill_confidence_ratio_temperature, 1e-12)
            )
            c_skill = float(math.sqrt(max(0.0, c_gap * c_ratio)))
        else:
            second_family = None
            second_rmse = math.inf
            second_mse = math.inf
            mse_gap = math.inf
            mse_ratio = math.inf
            c_gap = 1.0
            c_ratio = 1.0
            c_skill = 1.0

        self.logger.info(
            f"[SkillMatch] selected={best_skill.name} family={best_family} | "
            f"best_rmse={best_rmse:.6f} best_mse={best_mse:.8f} | "
            f"second_family={second_family} second_rmse={second_rmse:.6f} "
            f"second_mse={second_mse:.8f} | "
            f"mse_gap={mse_gap:.8f} mse_ratio={mse_ratio:.3f} | "
            f"c_gap={c_gap:.3f} c_ratio={c_ratio:.3f} c_skill={c_skill:.3f}"
        )
        return best_skill, best_align, c_skill, best_rmse

    def prepare_prediction_context(self, prompt_msg: PromptTrajectory, predict_force: bool):
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
            self.last_prediction_context = {"ok": False}
            return {
                "ok": False,
                "predicted": self.numpy_to_predicted(
                    prompt_msg,
                    probe,
                    probe_quat,
                    probe_force,
                    target_speed=target_speed,
                    skill_name="",
                    success=False,
                    confidence=0.0,
                ),
            }

        probe_eq = moving_average_centered_pos(probe_eq, self.smooth_win)
        probe_quat_eq = moving_average_centered_6d(probe_quat_eq, self.smooth_win)
        if probe_force_eq is not None:
            probe_force_eq = moving_average_centered_6d(probe_force_eq, self.smooth_win)

        # 2) Skill matching
        t_match_start = time.perf_counter()
        skill, (R, s, t, j_end), c_skill, skill_rmse = self.match_skill_with_confidence(probe_eq)
        match_ms = (time.perf_counter() - t_match_start) * 1000.0

        ref_eq = skill.ref_eq
        model = skill.model

        self.logger.info(f"Matched skill: {skill.name} | matching={match_ms:.2f} ms")
        if c_skill < self.skill_confidence_min:
            self.logger.info(
                f"[Predict] Skill confidence {c_skill:.3f} below min {self.skill_confidence_min:.3f}; skipping prediction"
            )
            self.last_prediction_context = {
                "ok": False,
                "reason": "low_skill_confidence",
                "skill": skill,
                "skill_confidence": c_skill,
                "skill_rmse": skill_rmse,
            }
            return {
                "ok": False,
                "reason": "low_skill_confidence",
                "predicted": self.numpy_to_predicted(
                    prompt_msg,
                    np.empty((0, 3), dtype=np.float64),
                    None,
                    None,
                    target_speed=target_speed,
                    skill_name=skill.name,
                    success=False,
                    confidence=c_skill,
                    skill_confidence=c_skill,
                    chunk_error=skill_rmse,
                ),
            }

        # 3) Transform to ref frame
        probe_in_ref = ((probe_eq - t) / s) @ R
        probe_goal = s * (ref_eq[-1] @ R.T) + t
        self.last_prediction_context = {
            "ok": True,
            "prompt_msg": prompt_msg,
            "target_speed": target_speed,
            "input_type": "spherical",
            "output_type": "delta",
            "match_ms": match_ms,
            "skill": skill,
            "R": R,
            "s": s,
            "t": t,
            "j_end": int(j_end),
            "skill_confidence": c_skill,
            "skill_rmse": skill_rmse,
            "ref_eq": ref_eq,
            "probe_eq": probe_eq,
            "probe_quat_eq": probe_quat_eq,
            "probe_force_eq": probe_force_eq,
            "probe_in_ref": probe_in_ref,
            "probe_goal": probe_goal,
        }
        return self.last_prediction_context.copy()

    def predict_from_context(self, ctx, rollout_horizon_override=None):
        t_predict_start = time.perf_counter()
        skill = ctx["skill"]
        R = ctx["R"]
        s = ctx["s"]
        t = ctx["t"]
        ref_eq = ctx["ref_eq"]
        probe_eq = ctx["probe_eq"]
        probe_quat_eq = ctx["probe_quat_eq"]
        probe_force_eq = ctx["probe_force_eq"]
        probe_in_ref = ctx["probe_in_ref"]
        probe_goal = ctx["probe_goal"]
        target_speed = ctx["target_speed"]
        input_type = ctx["input_type"]
        output_type = ctx["output_type"]
        match_ms = ctx["match_ms"]
        model = skill.model

        # 4) Rollout
        t_rollout_start = time.perf_counter()
        preds = None
        preds_quat = None
        preds_force = None
        selected_vars_ref = None
        accepted_chunk_error = self.mse_thresh

        rollout_h = self.rollout_horizon
        if rollout_horizon_override is not None:
            rollout_h = max(1, int(rollout_horizon_override))

        for attempt in range(self.max_retries):
            cur_pos = probe_in_ref.copy()
            cur_quat = probe_quat_eq.copy()[:cur_pos.shape[0]]
            cur_force = None if probe_force_eq is None else probe_force_eq.copy()[:cur_pos.shape[0]]
            failed = False
            hist_len = cur_pos.shape[0]

            tp = torch.tensor(cur_pos, dtype=torch.float32)
            tq = torch.tensor(cur_quat, dtype=torch.float32)
            tf = None if cur_force is None else torch.tensor(cur_force, dtype=torch.float32)

            preds_ref_pos, preds_quat, preds_ref_force, _, _, _, vars_ref = rollout_reference_6d(
                model,
                tp,
                tq,
                start_t=cur_pos.shape[0] - 1,
                h=rollout_h,
                k=self.k,
                input_type=input_type,
                output_type=output_type,
                R_ref_probe=R,
                traj_force=tf,
            )

            preds_ref_pos_np = preds_ref_pos.numpy()
            preds_world_pos = s * (preds_ref_pos_np @ R.T) + t
            preds_world_quat = preds_quat.numpy()
            preds_world_force = None
            if preds_ref_force is not None:
                preds_world_force = preds_ref_force.numpy()

            # Truncation logic
            if preds_world_pos.shape[0] > 0:
                dists_goal = np.linalg.norm(preds_world_pos - probe_goal, axis=1)
                vars_ref_arr = np.asarray(vars_ref)
                step_unc = vars_ref_arr if vars_ref_arr.ndim == 1 else np.max(vars_ref_arr, axis=1)
                stop_idx = np.where((dists_goal < self.goal_stop_eps) & (step_unc > 1e-3))[0]
                if stop_idx.size > 0:
                    i_stop = int(stop_idx[0])
                    self.logger.info(
                        f"[Predict] Reached goal at step {i_stop}, d={dists_goal[i_stop]:.4f}"
                    )
                    preds_ref_pos_np = preds_ref_pos_np[: i_stop + 1]
                    preds_world_pos = preds_world_pos[: i_stop + 1]
                    preds_world_quat = preds_world_quat[: i_stop + 1]
                    vars_ref = vars_ref[: i_stop + 1]
                    if preds_world_force is not None:
                        preds_world_force = preds_world_force[: i_stop + 1]

            # Geometric drift check on the predicted chunk only
            ref_tail_start = min(hist_len, len(ref_eq))
            ref_tail = ref_eq[ref_tail_start:]
            compare_len = min(len(preds_ref_pos_np), len(ref_tail))
            if compare_len <= 0:
                self.logger.info("[GeomCheck] No valid reference tail for chunk check, retry...")
                failed = True
            else:
                mse_chunk = geom_mse(
                    preds_ref_pos_np[:compare_len],
                    ref_tail[:compare_len],
                    compare_len,
                )
                self.logger.info(f"[GeomCheck] chunk mse = {mse_chunk:.5f} (len={compare_len})")

            # Single history update
            cur_pos = np.vstack([cur_pos, preds_ref_pos_np])

            if not failed and mse_chunk > self.mse_thresh:
                self.logger.info("[Recover] Geometric drift detected, retry...")
                failed = True

            if not failed:
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
                    preds_force = preds_world_force[i_start:] if preds_world_force is not None else None
                    selected_vars_ref = np.asarray(vars_ref)[i_start:]
                    accepted_chunk_error = float(mse_chunk)
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
                ctx["prompt_msg"],
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
        variance_means = self.per_point_variance_means(selected_vars_ref)
        variance_mean = float(np.mean(variance_means)) if variance_means else 0.0
        progress = float(np.clip(float(ctx["j_end"]) / max(float(len(ref_eq)), 1.0), 0.0, 1.0))
        c_skill = float(ctx.get("skill_confidence", 0.0))
        return self.numpy_to_predicted(
            ctx["prompt_msg"],
            preds,
            preds_quat,
            preds_force,
            target_speed=target_speed,
            skill_name=skill.name,
            success=True,
            confidence=c_skill,
            skill_confidence=c_skill,
            variance_mean=variance_mean,
            variance_means=variance_means,
            chunk_error=accepted_chunk_error,
            progress=progress,
        )

    def predict(
        self,
        prompt_msg: PromptTrajectory,
        predict_force: bool = True,
        rollout_horizon_override: int = None,
    ):
        ctx = self.prepare_prediction_context(prompt_msg, predict_force)
        if not ctx["ok"]:
            reason = ctx.get("reason", "not_enough_probe_points")
            self.logger.info(f"[Predict] Context not ready: {reason}")
            return ctx["predicted"]
        return self.predict_from_context(
            ctx,
            rollout_horizon_override=rollout_horizon_override,
        )

    def build_progressive_chunk_sizes(self, first_chunk_horizon: int, rollout_step: int):
        first = max(1, int(first_chunk_horizon))
        step = max(1, int(rollout_step))
        max_h = max(1, int(self.rollout_horizon))
        chunk_sizes = []
        remaining = max_h
        next_size = first
        while remaining > 0:
            size = min(next_size, remaining)
            chunk_sizes.append(size)
            remaining -= size
            next_size = step
        return chunk_sizes

    def iter_progressive_predictions(
        self,
        prompt_msg: PromptTrajectory,
        predict_force: bool,
        first_chunk_horizon: int,
        rollout_step: int,
    ):
        ctx = self.prepare_prediction_context(prompt_msg, predict_force)
        if not ctx["ok"]:
            reason = ctx.get("reason", "not_enough_probe_points")
            self.logger.info(f"[Predict] Context not ready: {reason}")
            yield 1, 1, ctx["predicted"]
            return

        skill = ctx["skill"]
        model = skill.model
        R = ctx["R"]
        s = ctx["s"]
        t = ctx["t"]
        ref_eq = ctx["ref_eq"]
        probe_eq = ctx["probe_eq"]
        probe_quat_eq = ctx["probe_quat_eq"]
        probe_force_eq = ctx["probe_force_eq"]
        probe_goal = ctx["probe_goal"]
        target_speed = ctx["target_speed"]
        prompt_msg_ctx = ctx["prompt_msg"]

        cur_pos = ctx["probe_in_ref"].copy()
        cur_quat = probe_quat_eq.copy()[:cur_pos.shape[0]]
        cur_force = None if probe_force_eq is None else probe_force_eq.copy()[:cur_pos.shape[0]]

        accepted_world_pos = None
        accepted_world_quat = None
        accepted_world_force = None
        accepted_vars_ref = None

        chunk_sizes = self.build_progressive_chunk_sizes(first_chunk_horizon, rollout_step)
        total = len(chunk_sizes)

        for idx, chunk_h in enumerate(chunk_sizes, start=1):
            t_chunk_start = time.perf_counter()
            reached_goal_in_chunk = False
            tp = torch.tensor(cur_pos, dtype=torch.float32)
            tq = torch.tensor(cur_quat, dtype=torch.float32)
            tf = None if cur_force is None else torch.tensor(cur_force, dtype=torch.float32)

            preds_ref_pos, preds_quat, preds_ref_force, _, _, _, vars_ref = rollout_reference_6d(
                model,
                tp,
                tq,
                start_t=cur_pos.shape[0] - 1,
                h=chunk_h,
                k=self.k,
                input_type=ctx["input_type"],
                output_type=ctx["output_type"],
                R_ref_probe=R,
                traj_force=tf,
            )

            preds_ref_pos_np = preds_ref_pos.numpy()
            preds_world_pos = s * (preds_ref_pos_np @ R.T) + t
            preds_world_quat = preds_quat.numpy()
            preds_world_force = None
            if preds_ref_force is not None:
                preds_world_force = preds_ref_force.numpy()

            if preds_world_pos.shape[0] > 0:
                dists_goal = np.linalg.norm(preds_world_pos - probe_goal, axis=1)
                vars_ref_arr = np.asarray(vars_ref)
                step_unc = vars_ref_arr if vars_ref_arr.ndim == 1 else np.max(vars_ref_arr, axis=1)
                stop_idx = np.where((dists_goal < self.goal_stop_eps) & (step_unc > 1e-3))[0]
                if stop_idx.size > 0:
                    i_stop = int(stop_idx[0])
                    reached_goal_in_chunk = True
                    preds_ref_pos_np = preds_ref_pos_np[: i_stop + 1]
                    preds_world_pos = preds_world_pos[: i_stop + 1]
                    preds_world_quat = preds_world_quat[: i_stop + 1]
                    if preds_world_force is not None:
                        preds_world_force = preds_world_force[: i_stop + 1]

            if preds_ref_pos_np.shape[0] == 0:
                self.logger.info("[Predict] Empty chunk prediction, stopping progressive rollout")
                break

            total_preds_ref = np.vstack([cur_pos, preds_ref_pos_np])
            compare_len = min(len(total_preds_ref), len(ref_eq))
            if (total_preds_ref.shape[0] > ref_eq.shape[0] + 500):
                self.logger.info("Chunk prediction is too long, stopping progressive rollout")
                break
            mse_chunk = geom_mse(
                total_preds_ref,
                ref_eq[:compare_len],
                compare_len,
            )
            self.logger.info(f"[GeomCheck] chunk mse = {mse_chunk:.5f} (len={compare_len})")
            if mse_chunk > self.mse_thresh:
                self.logger.info("[Predict] Chunk check failed, stopping progressive rollout")
                break

            cur_tip_world = s * (cur_pos[-1] @ R.T) + t
            jump_dist = np.linalg.norm(preds_world_pos - cur_tip_world, axis=1)
            candidate_idxs = np.where(jump_dist < self.max_start_jump)[0]
            if len(candidate_idxs) == 0:
                self.logger.info("[Predict] No continuous chunk start found, stop")
                break
            i_start = int(candidate_idxs[0])

            new_ref_pos = preds_ref_pos_np[i_start:]
            new_world_pos = preds_world_pos[i_start:]
            new_world_quat = preds_world_quat[i_start:]
            new_world_force = preds_world_force[i_start:] if preds_world_force is not None else None
            new_vars_ref = np.asarray(vars_ref)[i_start:i_start + len(new_world_pos)]
            if new_world_pos.shape[0] == 0:
                self.logger.info("[Predict] Filtered chunk became empty, stop")
                break

            if accepted_world_pos is None:
                accepted_world_pos = new_world_pos
                accepted_world_quat = new_world_quat
                accepted_world_force = new_world_force
                accepted_vars_ref = new_vars_ref
            else:
                accepted_world_pos = np.vstack([accepted_world_pos, new_world_pos])
                accepted_world_quat = np.vstack([accepted_world_quat, new_world_quat])
                accepted_vars_ref = np.concatenate([accepted_vars_ref, new_vars_ref])
                if accepted_world_force is not None and new_world_force is not None:
                    accepted_world_force = np.vstack([accepted_world_force, new_world_force])
                elif accepted_world_force is None:
                    accepted_world_force = new_world_force

            cur_pos = np.vstack([cur_pos, new_ref_pos])
            cur_quat = np.vstack([cur_quat, new_world_quat])
            if cur_force is not None and new_world_force is not None:
                cur_force = np.vstack([cur_force, new_world_force])

            pred_msg = self.numpy_to_predicted(
                prompt_msg_ctx,
                accepted_world_pos,
                accepted_world_quat,
                accepted_world_force,
                target_speed=target_speed,
                skill_name=skill.name,
                success=True,
                confidence=float(ctx.get("skill_confidence", 0.0)),
                skill_confidence=float(ctx.get("skill_confidence", 0.0)),
                variance_mean=(
                    float(np.mean(self.per_point_variance_means(accepted_vars_ref)))
                    if accepted_vars_ref is not None and np.asarray(accepted_vars_ref).size > 0
                    else 0.0
                ),
                variance_means=self.per_point_variance_means(accepted_vars_ref),
                chunk_error=float(mse_chunk),
                progress=float(np.clip(float(ctx["j_end"]) / max(float(len(ref_eq)), 1.0), 0.0, 1.0)),
            )
            chunk_ms = (time.perf_counter() - t_chunk_start) * 1000.0
            self.logger.info(
                f"[Timing] chunk {idx}/{total} | predict_ms={chunk_ms:.2f}"
            )
            yield idx, total, pred_msg
            if reached_goal_in_chunk:
                self.logger.info(
                    "[Predict] Reached goal in current chunk, stop further progressive prediction"
                )
                break
