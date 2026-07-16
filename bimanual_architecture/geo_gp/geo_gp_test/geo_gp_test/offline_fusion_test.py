"""Offline CSV-based test node for Geo-GP trajectory fusion."""

import csv
import json
import math
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Vector3
from rclpy.node import Node

from geo_gp_interfaces.msg import PromptTrajectory
from geo_gp_prediction.predictor import Predictor
from geometry.frame6d import estimate_rotation_scale_3d_search_by_count
from geometry.resample import resample_by_arclen_fraction
from geo_gp_fusion.policies.weighted_blending import blend_pose, sample_timed_pose
from geo_gp_fusion.policies.optimal_blending import optimal_blend_pose, pose_conflict
from geo_gp_fusion.policies.nash_blending import nash_blend_pose


POSE_FIELDS = ('time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')


@dataclass
class TimedPose:
    """Pose sample loaded from a CSV trajectory.

    Attributes:
        time: Sample time in seconds from the trajectory start.
        pose: Cartesian pose at ``time``.
        alpha: Optional prediction authority used to produce this pose.
    """

    time: float
    pose: Pose
    alpha: float = None
    autonomy_level: float = None


@dataclass
class FusionConfig:
    """Offline copy of the online fuser confidence parameters.

    Attributes:
        confidence_gain: Gain applied to the computed GP confidence.
        min_prediction_weight: Lower bound for GP authority.
        max_prediction_weight: Upper bound for GP authority.
        network_k_delay: Exponential decay gain for network delay confidence.
        network_delay_max: Delay value treated as the worst nominal delay.
        network_k_jitter: Exponential decay gain for network jitter confidence.
        network_jitter_max: Jitter value treated as the worst nominal jitter.
        network_w_delay: Relative weight of delay in network confidence.
        network_w_jitter: Relative weight of jitter in network confidence.
        network_gamma: Global exponent applied to network confidence.
        gp_skill_min: Minimum skill confidence required to trust the GP.
        gp_k_sigma: Exponential decay gain for GP variance confidence.
        gp_k_chunk: Exponential decay gain for chunk error confidence.
        gp_error_fail: Chunk error scale considered a failed local match.
        gp_k_progress: Logistic gain for progress confidence.
        gp_progress_midpoint: Progress value where logistic confidence is 0.5.
        gp_w_sigma: Relative weight of GP variance confidence.
        gp_w_chunk: Relative weight of chunk error confidence.
        gp_w_progress: Relative weight of progress confidence.
        gp_gamma: Global exponent applied to GP confidence.
        authority_eps: Small denominator guard for authority computation.
        skill_confidence: Offline skill confidence value used for all samples.
        point_variance: Offline conservative point variance used for all samples.
        chunk_error: Offline chunk error value used for all samples.
        progress: Offline progress value used for all samples.
        network_delay: Offline network delay value used for all samples.
        network_jitter: Offline network jitter value used for all samples.
    """

    confidence_gain: float
    min_prediction_weight: float
    max_prediction_weight: float
    network_k_delay: float
    network_delay_max: float
    network_k_jitter: float
    network_jitter_max: float
    network_w_delay: float
    network_w_jitter: float
    network_gamma: float
    gp_skill_min: float
    gp_k_sigma: float
    gp_k_chunk: float
    gp_error_fail: float
    gp_k_progress: float
    gp_progress_midpoint: float
    gp_w_sigma: float
    gp_w_chunk: float
    gp_w_progress: float
    gp_gamma: float
    authority_eps: float
    skill_confidence: float
    point_variance: float
    chunk_error: float
    progress: float
    network_delay: float
    network_jitter: float
    rate: float
    leader_timeout_sec: float
    fusion_policy: str
    optimal_lambda_s: float
    optimal_lambda_c: float
    nash_human_effort: float
    nash_gp_effort: float
    nash_human_agreement: float
    nash_gp_agreement: float
    nash_agreement_ratio: float
    nash_rotation_weight: float
    autonomy_alpha_time_constant: float
    autonomy_alpha_rate_limit: float


class AutonomyPassivityController:
    """Pose-only equivalent of the controller-side autonomy alpha conditioner.

    The C++ energy tank constrains Cartesian wrenches, which cannot be
    reconstructed from pose-only CSV files. Its alpha conditioner is reproduced
    here so the saved autonomy level matches the requested controller behavior.

    Args:
        alpha_time_constant: Time constant in seconds for alpha smoothing.
        alpha_rate_limit: Maximum alpha change per second. Non-positive values
            disable rate limiting.
    """

    def __init__(self, alpha_time_constant=0.05, alpha_rate_limit=2.0):
        self.alpha_time_constant = float(alpha_time_constant)
        self.alpha_rate_limit = float(alpha_rate_limit)
        self.alpha = 0.0

    def update_alpha(self, alpha_requested, dt):
        """Filter and rate-limit a requested autonomy level.

        Args:
            alpha_requested: Raw requested autonomy in the range ``[0, 1]``.
            dt: Time step in seconds since the previous update.

        Returns:
            Passivity-controller-conditioned autonomy level in ``[0, 1]``.
        """
        requested = clamp(
            float(alpha_requested) if math.isfinite(float(alpha_requested)) else 0.0,
            0.0,
            1.0,
        )
        safe_dt = float(dt) if math.isfinite(float(dt)) and dt > 0.0 else 0.0
        target = requested
        if self.alpha_time_constant > 0.0 and safe_dt > 0.0:
            beta = safe_dt / (self.alpha_time_constant + safe_dt)
            target = self.alpha + beta * (requested - self.alpha)
        if self.alpha_rate_limit > 0.0 and safe_dt > 0.0:
            delta = self.alpha_rate_limit * safe_dt
            target = clamp(target, self.alpha - delta, self.alpha + delta)
        self.alpha = clamp(target, 0.0, 1.0)
        return self.alpha


@dataclass
class PredictionMetrics:
    """Prediction confidence values reconstructed for one trial.

    Attributes:
        skill_name: Skill/reference model selected for the saved prediction.
        skill_confidence: Skill matching confidence from the predictor.
        trajectory_variance: Mean conservative point variance over the reconstructed prediction.
        point_variances: Per-point conservative GP variance values aligned with prediction time.
        chunk_error: Geometric chunk error accepted by the predictor.
        progress: Matched progress along the reference trajectory.
        source: Human-readable source of the metrics.
    """

    skill_name: str
    skill_confidence: float
    trajectory_variance: float
    point_variances: list
    chunk_error: float
    progress: float
    source: str


def clamp(value, low, high):
    """Clamp a numeric value to a closed interval.

    Args:
        value: Input numeric value.
        low: Minimum allowed value.
        high: Maximum allowed value.

    Returns:
        ``value`` clipped to the inclusive range ``[low, high]``.
    """
    return max(low, min(high, value))


def normalized_weights(*weights):
    """Normalize non-negative finite weights.

    Args:
        *weights: Raw component weights.

    Returns:
        A list of normalized weights that sums to 1.0. If every input is zero
        or invalid, all components receive equal weight.
    """
    clean = [max(0.0, float(w)) if math.isfinite(float(w)) else 0.0 for w in weights]
    total = sum(clean)
    if total <= 1e-12:
        return [1.0 / len(clean)] * len(clean)
    return [w / total for w in clean]


def expand_path(path_text):
    """Expand user syntax in a path-like string.

    Args:
        path_text: Path string that may start with ``~``.

    Returns:
        Expanded ``Path`` object.
    """
    return Path(path_text).expanduser()


def pose_from_row(row):
    """Convert a CSV row to a timed ROS pose sample.

    Args:
        row: Dictionary row containing ``time,x,y,z,qx,qy,qz,qw`` fields.

    Returns:
        Timed pose sample parsed from the row.

    Raises:
        KeyError: If a required field is absent.
        ValueError: If any required field cannot be converted to ``float``.
    """
    pose = Pose()
    pose.position.x = float(row['x'])
    pose.position.y = float(row['y'])
    pose.position.z = float(row['z'])
    pose.orientation.x = float(row['qx'])
    pose.orientation.y = float(row['qy'])
    pose.orientation.z = float(row['qz'])
    pose.orientation.w = float(row['qw'])
    return TimedPose(float(row['time']), pose)


def read_prompt_trajectory(csv_path):
    """Read a prompt CSV into a ``PromptTrajectory`` message.

    Args:
        csv_path: Path to ``prompt_success_*.csv``.

    Returns:
        Prompt trajectory message containing poses, times, and optional force
        vectors when ``fx,fy,fz`` columns are present.

    Raises:
        ValueError: If required pose columns are missing or no samples exist.
    """
    prompt = PromptTrajectory()
    with csv_path.open('r', newline='') as csv_file:
        reader = csv.DictReader(csv_file)
        missing = [field for field in POSE_FIELDS if field not in reader.fieldnames]
        if missing:
            raise ValueError(f'{csv_path} is missing columns: {missing}')
        has_force = all(field in reader.fieldnames for field in ('fx', 'fy', 'fz'))
        for row in reader:
            sample = pose_from_row(row)
            prompt.time_from_start.append(sample.time)
            prompt.poses.append(sample.pose)
            if has_force:
                force = Vector3()
                force.x = float(row['fx'])
                force.y = float(row['fy'])
                force.z = float(row['fz'])
                prompt.forces.append(force)
    if not prompt.poses:
        raise ValueError(f'{csv_path} contains no prompt samples')
    return prompt


def read_trajectory(csv_path):
    """Read a pose trajectory CSV.

    Args:
        csv_path: Path to a trajectory CSV file. The file may contain extra
            columns, but it must include the pose fields in ``POSE_FIELDS``.

    Returns:
        List of timed pose samples in file order.

    Raises:
        ValueError: If required columns are missing or the file has no samples.
    """
    samples = []
    with csv_path.open('r', newline='') as csv_file:
        reader = csv.DictReader(csv_file)
        missing = [field for field in POSE_FIELDS if field not in reader.fieldnames]
        if missing:
            raise ValueError(f'{csv_path} is missing columns: {missing}')
        for row in reader:
            samples.append(pose_from_row(row))
    if not samples:
        raise ValueError(f'{csv_path} contains no trajectory samples')
    return samples


def write_trajectory(csv_path, samples):
    """Write a fused pose trajectory CSV.

    Args:
        csv_path: Destination CSV path.
        samples: Timed fused poses to write.

    The output uses the prediction trajectory format plus the requested fusion
    authority and the passivity-conditioned autonomy level:
    ``time,x,y,z,qx,qy,qz,qw,alpha,autonomy_level``.
    """
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    has_alpha = any(sample.alpha is not None for sample in samples)
    has_autonomy_level = any(sample.autonomy_level is not None for sample in samples)
    fieldnames = POSE_FIELDS + (('alpha',) if has_alpha else ()) + (
        ('autonomy_level',) if has_autonomy_level else ()
    )
    with csv_path.open('w', newline='') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            pose = sample.pose
            row = {
                'time': f'{sample.time:.6f}',
                'x': f'{pose.position.x:.6f}',
                'y': f'{pose.position.y:.6f}',
                'z': f'{pose.position.z:.6f}',
                'qx': f'{pose.orientation.x:.6f}',
                'qy': f'{pose.orientation.y:.6f}',
                'qz': f'{pose.orientation.z:.6f}',
                'qw': f'{pose.orientation.w:.6f}',
            }
            if has_alpha:
                row['alpha'] = f'{sample.alpha:.6f}' if sample.alpha is not None else ''
            if has_autonomy_level:
                row['autonomy_level'] = (
                    f'{sample.autonomy_level:.6f}'
                    if sample.autonomy_level is not None else ''
                )
            writer.writerow(row)



def find_trial_dirs(input_path, recursive):
    """Find directories containing prediction and leader execution CSV files.

    Args:
        input_path: File or directory supplied by the user.
        recursive: Whether to search recursively below ``input_path``.

    Returns:
        Sorted list of trial directories. A trial directory is any directory with
        at least one ``prediction_success_*.csv`` and one
        ``leader_execution_*.csv``.
    """
    if input_path.is_file():
        return [input_path.parent]
    if not recursive:
        return [input_path]

    trial_dirs = set()
    for prediction_path in input_path.rglob('prediction_success_*.csv'):
        if list(prediction_path.parent.glob('leader_execution_*.csv')):
            trial_dirs.add(prediction_path.parent)
    return sorted(trial_dirs)


def companion_file(directory, prediction_path, prefix, suffix):
    """Find an artifact that shares a prediction timestamp.

    Args:
        directory: Trial directory to search.
        prediction_path: Prediction CSV path that provides the timestamp.
        prefix: Companion file prefix, such as ``prompt_success_``.
        suffix: Companion file suffix, such as ``.csv`` or ``.json``.

    Returns:
        Matching companion path when present, otherwise the newest matching file
        for the requested prefix and suffix.
    """
    stamp = prediction_path.name.replace('prediction_success_', '', 1).replace('.csv', '')
    exact = directory / f'{prefix}{stamp}{suffix}'
    if exact.exists():
        return exact
    return newest_file(directory, f'{prefix}*{suffix}')


def newest_file(directory, pattern):
    """Return the newest matching file in a directory by filename.

    Args:
        directory: Directory to search.
        pattern: Glob pattern to match.

    Returns:
        Last lexicographic match, or ``None`` if no files match.
    """
    matches = sorted(directory.glob(pattern))
    return matches[-1] if matches else None


def saved_transform_rmse(ref_eq, probe_eq, transform):
    """Compute matching RMSE for a saved similarity transform.

    Args:
        ref_eq: Equal-distance reference trajectory from the saved skill.
        probe_eq: Equal-distance prompt trajectory reconstructed offline.
        transform: Similarity transform payload saved beside the prediction CSV.

    Returns:
        RMSE between the prompt and the saved transformed reference prefix.
    """
    j_end = int(transform['j_end'])
    ref_prefix = np.asarray(ref_eq[:j_end], dtype=np.float64)
    probe_eq = np.asarray(probe_eq, dtype=np.float64)
    ref_resampled = resample_by_arclen_fraction(ref_prefix, probe_eq.shape[0])
    rotation = np.asarray(transform['R'], dtype=np.float64)
    scale = float(transform['s'])
    translation = np.asarray(transform['t'], dtype=np.float64)
    transformed = scale * (ref_resampled @ rotation.T) + translation
    return float(np.sqrt(np.mean(np.sum((probe_eq - transformed) ** 2, axis=1))))


def metrics_from_predicted(predicted, fallback_skill_name='', source='reference_model'):
    """Extract fusion metrics from a reconstructed prediction message.

    Args:
        predicted: ``PredictedTrajectory`` message returned by ``Predictor``.
        fallback_skill_name: Skill name used if the message does not provide one.
        source: Label describing how the metrics were obtained.

    Returns:
        PredictionMetrics used by the offline fuser.
    """
    point_variances = [float(v) for v in predicted.variance_means]
    trajectory_variance = (
        float(predicted.variance_mean)
        if math.isfinite(float(predicted.variance_mean))
        else 0.0
    )
    return PredictionMetrics(
        skill_name=predicted.skill_name or fallback_skill_name,
        skill_confidence=float(predicted.skill_confidence),
        trajectory_variance=trajectory_variance,
        point_variances=point_variances,
        chunk_error=float(predicted.chunk_error),
        progress=float(predicted.progress),
        source=source,
    )


def fixed_metrics(config):
    """Create fallback metrics from fixed ROS parameters.

    Args:
        config: Fusion configuration containing fixed confidence fields.

    Returns:
        PredictionMetrics mirroring the previous fixed-parameter behavior.
    """
    return PredictionMetrics(
        skill_name='',
        skill_confidence=config.skill_confidence,
        trajectory_variance=config.point_variance,
        point_variances=[],
        chunk_error=config.chunk_error,
        progress=config.progress,
        source='fixed_parameters',
    )


def fused_output_path(trial_dir, input_root, output_dir, prediction_path):
    """Build the destination path for one fused trajectory.

    Args:
        trial_dir: Directory containing the input trial CSV files.
        input_root: Root path used to preserve relative layout under
            ``output_dir``.
        output_dir: Optional output root. If ``None``, output is written beside
            the input CSV files.
        prediction_path: Prediction CSV whose timestamp is reused for the fused
            CSV filename.

    Returns:
        Destination path named ``fused_success_*.csv``.
    """
    suffix = prediction_path.name.replace('prediction_success_', 'fused_success_', 1)
    if output_dir is None:
        return trial_dir / suffix

    try:
        relative_dir = trial_dir.relative_to(input_root)
    except ValueError:
        relative_dir = Path(trial_dir.name)
    return output_dir / relative_dir / suffix


class OfflineFusionTestNode(Node):
    """Run offline Geo-GP fusion against saved CSV trajectories.

    The node reads saved prediction and leader execution trajectories, computes
    the same scalar GP authority used by ``OnlineFuserNode``, samples the
    prediction at each leader timestamp, blends both poses, and writes a fused
    CSV trajectory.
    """

    def __init__(self):
        """Declare ROS parameters for offline fusion."""
        super().__init__('offline_fusion_test')
        self.declare_parameter('input_path', '~/geo-gp/data/06-02/preds')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('recursive', True)
        self.declare_parameter('overwrite', True)
        self.declare_parameter('leader_time_relative', False)
        self.declare_parameter('use_reference_metrics', True)
        self.declare_parameter('config_path', '/home/user/geo-gp/config/default.yaml')
        self.declare_parameter('model_dir', '/home/user/geo-gp/data/06-02/models/6d')
        self.declare_parameter('predict_force', True)
        self.declare_parameter('confidence_gain', 1.0)
        self.declare_parameter('min_prediction_weight', 0.0)
        self.declare_parameter('max_prediction_weight', 1.0)
        self.declare_parameter('network_k_delay', 3.0)
        self.declare_parameter('network_delay_max', 0.2)
        self.declare_parameter('network_k_jitter', 3.0)
        self.declare_parameter('network_jitter_max', 0.05)
        self.declare_parameter('network_w_delay', 0.5)
        self.declare_parameter('network_w_jitter', 0.5)
        self.declare_parameter('network_gamma', 1.0)
        self.declare_parameter('gp_skill_min', 0.5)
        self.declare_parameter('gp_k_sigma', 2.0)
        self.declare_parameter('gp_k_chunk', 1.0)
        self.declare_parameter('gp_error_fail', 0.01)
        self.declare_parameter('gp_k_progress', 10.0)
        self.declare_parameter('gp_progress_midpoint', 0.25)
        self.declare_parameter('gp_w_sigma', 0.45)
        self.declare_parameter('gp_w_chunk', 0.40)
        self.declare_parameter('gp_w_progress', 0.15)
        self.declare_parameter('gp_gamma', 1.0)
        self.declare_parameter('authority_eps', 1e-6)
        self.declare_parameter('skill_confidence', 1.0)
        self.declare_parameter('point_variance', 0.0)
        self.declare_parameter('chunk_error', 0.0)
        self.declare_parameter('progress', 1.0)
        self.declare_parameter('network_delay', 0.0)
        self.declare_parameter('network_jitter', 0.0)
        # Keep these defaults aligned with OnlineFuserNode
        self.declare_parameter('rate', 200.0)
        self.declare_parameter('leader_timeout_sec', 0.1)
        self.declare_parameter('fusion_policy', 'weighted_blending')
        self.declare_parameter('optimal_lambda_s', 0.10)
        self.declare_parameter('optimal_lambda_c', 0.05)
        self.declare_parameter('nash_human_effort', 0.2)
        self.declare_parameter('nash_gp_effort', 0.5)
        self.declare_parameter('nash_human_agreement', 0.05)
        self.declare_parameter('nash_gp_agreement', 0.10)
        self.declare_parameter('nash_agreement_ratio', 0.7)
        self.declare_parameter('nash_rotation_weight', 1.0)
        # Same defaults as AutonomyPassivityParameters in the C++ controller
        self.declare_parameter('autonomy_alpha_time_constant', 0.05)
        self.declare_parameter('autonomy_alpha_rate_limit', 2.0)
        self._filtered_prediction_weight = 0.0
        self._previous_optimal_weight = 0.0
        self._last_fused_pose = None

    def config(self):
        """Read ROS parameters into a fusion configuration.

        Returns:
            FusionConfig populated from the node's declared ROS parameters.
        """
        return FusionConfig(
            confidence_gain=self.get_float('confidence_gain'),
            min_prediction_weight=self.get_float('min_prediction_weight'),
            max_prediction_weight=self.get_float('max_prediction_weight'),
            network_k_delay=self.get_float('network_k_delay'),
            network_delay_max=self.get_float('network_delay_max'),
            network_k_jitter=self.get_float('network_k_jitter'),
            network_jitter_max=self.get_float('network_jitter_max'),
            network_w_delay=self.get_float('network_w_delay'),
            network_w_jitter=self.get_float('network_w_jitter'),
            network_gamma=self.get_float('network_gamma'),
            gp_skill_min=self.get_float('gp_skill_min'),
            gp_k_sigma=self.get_float('gp_k_sigma'),
            gp_k_chunk=self.get_float('gp_k_chunk'),
            gp_error_fail=self.get_float('gp_error_fail'),
            gp_k_progress=self.get_float('gp_k_progress'),
            gp_progress_midpoint=self.get_float('gp_progress_midpoint'),
            gp_w_sigma=self.get_float('gp_w_sigma'),
            gp_w_chunk=self.get_float('gp_w_chunk'),
            gp_w_progress=self.get_float('gp_w_progress'),
            gp_gamma=self.get_float('gp_gamma'),
            authority_eps=self.get_float('authority_eps'),
            skill_confidence=self.get_float('skill_confidence'),
            point_variance=self.get_float('point_variance'),
            chunk_error=self.get_float('chunk_error'),
            progress=self.get_float('progress'),
            network_delay=self.get_float('network_delay'),
            network_jitter=self.get_float('network_jitter'),
            rate=self.get_float('rate'),
            leader_timeout_sec=self.get_float('leader_timeout_sec'),
            fusion_policy=self.normalize_fusion_policy(self.get_string('fusion_policy')),
            optimal_lambda_s=self.get_float('optimal_lambda_s'),
            optimal_lambda_c=self.get_float('optimal_lambda_c'),
            nash_human_effort=self.get_float('nash_human_effort'),
            nash_gp_effort=self.get_float('nash_gp_effort'),
            nash_human_agreement=self.get_float('nash_human_agreement'),
            nash_gp_agreement=self.get_float('nash_gp_agreement'),
            nash_agreement_ratio=self.get_float('nash_agreement_ratio'),
            nash_rotation_weight=self.get_float('nash_rotation_weight'),
            autonomy_alpha_time_constant=self.get_float('autonomy_alpha_time_constant'),
            autonomy_alpha_rate_limit=self.get_float('autonomy_alpha_rate_limit'),
        )

    def normalize_fusion_policy(self, value):
        """Normalize a configured fusion policy name.

        Args:
            value: Configured fusion policy or alias.

        Returns:
            One of ``weighted_blending``, ``optimal_blending``, or
            ``nash_blending``. Unknown values fall back to weighted blending.
        """
        aliases = {
            'weighted': 'weighted_blending', 'weighted_blending': 'weighted_blending',
            'optimal': 'optimal_blending', 'optimal_blending': 'optimal_blending',
            'nash': 'nash_blending', 'nash_blending': 'nash_blending',
        }
        policy = aliases.get(str(value).strip().lower())
        if policy is None:
            self.get_logger().warn(
                f'Unknown fusion_policy={value!r}; falling back to weighted_blending'
            )
            return 'weighted_blending'
        return policy

    def get_float(self, name):
        """Read a ROS parameter as a float.

        Args:
            name: Parameter name.

        Returns:
            Parameter value converted to ``float``.
        """
        return float(self.get_parameter(name).value)

    def get_bool(self, name):
        """Read a ROS parameter as a bool.

        Args:
            name: Parameter name.

        Returns:
            Parameter value converted to ``bool``.
        """
        return bool(self.get_parameter(name).value)

    def get_string(self, name):
        """Read a ROS parameter as a stripped string.

        Args:
            name: Parameter name.

        Returns:
            Parameter value converted to ``str`` and stripped.
        """
        return str(self.get_parameter(name).value).strip()

    def make_predictor(self):
        """Create a predictor for reconstructing reference-model metrics.

        Returns:
            Predictor instance, or ``None`` when reference metrics are disabled
            or required paths are not configured.
        """
        if not self.get_bool('use_reference_metrics'):
            return None
        config_path = self.get_string('config_path')
        model_dir = self.get_string('model_dir')
        if not config_path or not model_dir:
            self.get_logger().warn(
                'Reference metrics requested but config_path/model_dir is empty; '
                'using fixed metrics'
            )
            return None
        return Predictor(self.get_logger(), config_path, model_dir)

    def saved_reference_confidence(self, predictor, saved_skill_name, saved_rmse, probe_eq):
        """Compute confidence with the saved skill family forced as best.

        Args:
            predictor: Predictor containing the current skill library.
            saved_skill_name: Skill name from ``similarity_transform_success_*.json``.
            saved_rmse: Matching RMSE of the saved reference transform.
            probe_eq: Preprocessed prompt trajectory used for matching.

        Returns:
            Skill confidence computed with the original family-level gap/ratio
            formula. The saved skill's family is forced to be the selected
            family, families with smaller MSE are ignored, and only families
            with MSE greater than or equal to the saved family are retained as
            second-family candidates.
        """

        def skill_family(name):
            parts = name.rsplit('_', 1)
            if len(parts) == 2 and parts[1].isdigit():
                return parts[0]
            return name

        saved_family = skill_family(saved_skill_name)
        saved_mse = saved_rmse * saved_rmse
        matches_by_family = {}
        eps = 1e-12

        for skill in predictor.skill_library.skills:
            family = skill_family(skill.name)
            if family == saved_family:
                continue
            _, _, _, _, rmse = estimate_rotation_scale_3d_search_by_count(
                skill.ref_eq,
                probe_eq,
                margin_pts=1000,
                step=15,
            )
            rmse = float(rmse)
            mse = rmse * rmse
            if family not in matches_by_family or mse < matches_by_family[family][2]:
                matches_by_family[family] = (skill.name, rmse, mse)

        discarded_smaller = []
        retained = []
        for family, match in matches_by_family.items():
            skill_name, rmse, mse = match
            if mse + eps < saved_mse:
                discarded_smaller.append((family, skill_name, rmse, mse))
            else:
                retained.append((family, skill_name, rmse, mse))

        if retained:
            second_family, second_name, second_rmse, second_mse = min(
                retained,
                key=lambda item: item[3],
            )
            mse_gap = max(0.0, second_mse - saved_mse)
            mse_ratio = second_mse / max(saved_mse, 1e-12)
            c_gap = 1.0 - math.exp(
                -mse_gap / max(predictor.skill_confidence_mse_temperature, 1e-12)
            )
            c_ratio = 1.0 - math.exp(
                -max(0.0, mse_ratio - 1.0)
                / max(predictor.skill_confidence_ratio_temperature, 1e-12)
            )
            confidence = float(math.sqrt(max(0.0, c_gap * c_ratio)))
        else:
            second_family = None
            second_name = None
            second_rmse = math.inf
            second_mse = math.inf
            mse_gap = math.inf
            mse_ratio = math.inf
            c_gap = 1.0
            c_ratio = 1.0
            confidence = 1.0

        self.get_logger().info(
            f'[ReferenceMetrics] forced family={saved_family} reference={saved_skill_name} | '
            f'saved_rmse={saved_rmse:.6f} saved_mse={saved_mse:.8f} | '
            f'discarded_smaller_families={len(discarded_smaller)} '
            f'retained_larger_or_equal_families={len(retained)} | '
            f'second_family={second_family} second_ref={second_name} '
            f'second_rmse={second_rmse:.6f} second_mse={second_mse:.8f} | '
            f'mse_gap={mse_gap:.8f} mse_ratio={mse_ratio:.3f} | '
            f'c_gap={c_gap:.3f} c_ratio={c_ratio:.3f} c_skill={confidence:.3f}'
        )
        return confidence

    def reconstruct_metrics(self, predictor, prompt_path, transform_path, config):
        """Reconstruct prediction metrics from saved reference artifacts.

        Args:
            predictor: Predictor loaded with the original model library.
            prompt_path: Saved prompt CSV for the trial.
            transform_path: Saved similarity transform JSON for the trial.
            config: Fixed fallback fusion configuration.

        Returns:
            PredictionMetrics reconstructed from the selected reference model, or
            fixed fallback metrics when reconstruction is unavailable.
        """
        if predictor is None or prompt_path is None or transform_path is None:
            return fixed_metrics(config)

        prompt_msg = read_prompt_trajectory(prompt_path)
        with transform_path.open('r', encoding='utf-8') as json_file:
            transform = json.load(json_file)

        original_skill_confidence_min = predictor.skill_confidence_min
        predictor.skill_confidence_min = -1.0
        try:
            ctx = predictor.prepare_prediction_context(
                prompt_msg,
                predict_force=self.get_bool('predict_force'),
            )
        finally:
            predictor.skill_confidence_min = original_skill_confidence_min

        if not ctx.get('ok'):
            self.get_logger().warn(
                f'Reference metric reconstruction failed for {prompt_path}: '
                f"{ctx.get('reason', 'context_not_ready')}"
            )
            return fixed_metrics(config)

        saved_skill_name = str(transform.get('skill_name', ''))
        if not saved_skill_name:
            self.get_logger().warn(
                f'{transform_path} has no skill_name; using matched predictor context'
            )
        else:
            skill_by_name = {skill.name: skill for skill in predictor.skill_library.skills}
            saved_skill = skill_by_name.get(saved_skill_name)
            if saved_skill is None:
                self.get_logger().warn(
                    f'Saved reference skill {saved_skill_name} is not in model_dir; '
                    'using matched predictor context'
                )
            else:
                ctx['skill'] = saved_skill
                ctx['ref_eq'] = saved_skill.ref_eq
                ctx['R'] = np.asarray(transform['R'], dtype=np.float64)
                ctx['s'] = float(transform['s'])
                ctx['t'] = np.asarray(transform['t'], dtype=np.float64)
                ctx['j_end'] = int(transform['j_end'])
                ctx['probe_in_ref'] = ((ctx['probe_eq'] - ctx['t']) / ctx['s']) @ ctx['R']
                ctx['probe_goal'] = ctx['s'] * (ctx['ref_eq'][-1] @ ctx['R'].T) + ctx['t']

                saved_rmse = saved_transform_rmse(saved_skill.ref_eq, ctx['probe_eq'], transform)
                ctx['skill_confidence'] = self.saved_reference_confidence(
                    predictor,
                    saved_skill_name,
                    saved_rmse,
                    ctx['probe_eq'],
                )
                ctx['skill_rmse'] = saved_rmse

        predicted = predictor.predict_from_context(ctx)
        if not predicted.success:
            self.get_logger().warn(
                f'Reference metric prediction was unsuccessful for {prompt_path}; '
                'using fixed metrics'
            )
            return fixed_metrics(config)
        return metrics_from_predicted(
            predicted,
            fallback_skill_name=saved_skill_name,
            source=f'reference_model:{saved_skill_name or predicted.skill_name}',
        )

    def run(self):
        """Run all requested offline fusion trials.

        Raises:
            FileNotFoundError: If ``input_path`` does not exist or no complete
            trial directories are found.
        """
        input_path = expand_path(str(self.get_parameter('input_path').value))
        output_text = str(self.get_parameter('output_dir').value).strip()
        output_dir = expand_path(output_text) if output_text else None
        recursive = bool(self.get_parameter('recursive').value)
        overwrite = bool(self.get_parameter('overwrite').value)
        leader_time_relative = bool(self.get_parameter('leader_time_relative').value)
        config = self.config()
        predictor = self.make_predictor()

        if not input_path.exists():
            raise FileNotFoundError(f'input_path does not exist: {input_path}')

        trial_dirs = find_trial_dirs(input_path, recursive)
        if not trial_dirs:
            raise FileNotFoundError(f'no trial directories found under {input_path}')

        wrote = 0
        for trial_dir in trial_dirs:
            prediction_path = newest_file(trial_dir, 'prediction_success_*.csv')
            leader_path = newest_file(trial_dir, 'leader_execution_*.csv')
            if prediction_path is None or leader_path is None:
                self.get_logger().warn(f'Skipping incomplete trial: {trial_dir}')
                continue

            prompt_path = companion_file(trial_dir, prediction_path, 'prompt_success_', '.csv')
            transform_path = companion_file(
                trial_dir,
                prediction_path,
                'similarity_transform_success_',
                '.json',
            )
            metrics = self.reconstruct_metrics(predictor, prompt_path, transform_path, config)

            output_path = fused_output_path(
                trial_dir,
                input_path if input_path.is_dir() else input_path.parent,
                output_dir,
                prediction_path,
            )
            if output_path.exists() and not overwrite:
                self.get_logger().info(f'Skipping existing output: {output_path}')
                continue

            count, min_alpha, max_alpha, min_autonomy, max_autonomy = self.fuse_trial(
                prediction_path,
                leader_path,
                output_path,
                config,
                metrics,
                leader_time_relative,
            )
            wrote += 1
            self.get_logger().info(
                f'Wrote {count} fused samples | alpha_G=[{min_alpha:.3f}, {max_alpha:.3f}] | '
                f'autonomy_level=[{min_autonomy:.3f}, {max_autonomy:.3f}] | '
                f'metrics={metrics.source} | skill={metrics.skill_name} | {output_path}'
            )

        self.get_logger().info(f'Offline fusion complete: {wrote} file(s) written')

    def compute_network_confidence(self, config):
        """Compute leader/network confidence using the online fuser formula.

        Args:
            config: Fusion configuration containing network confidence inputs.

        Returns:
            Network confidence in the range ``[0, 1]``.
        """
        d_max = max(config.network_delay_max, 1e-9)
        j_max = max(config.network_jitter_max, 1e-9)
        delay = (
            max(0.0, float(config.network_delay))
            if math.isfinite(float(config.network_delay)) else d_max
        )
        jitter = (
            max(0.0, float(config.network_jitter))
            if math.isfinite(float(config.network_jitter)) else j_max
        )
        c_d = math.exp(-config.network_k_delay * delay / d_max)
        c_j = math.exp(-config.network_k_jitter * jitter / j_max)
        w_d, w_j = normalized_weights(config.network_w_delay, config.network_w_jitter)
        return (c_d ** w_d * c_j ** w_j) ** max(config.network_gamma, 0.0)

    def compute_gp_confidence(self, config):
        """Compute GP confidence using the online fuser formula.

        Args:
            config: Fusion configuration containing prediction confidence inputs.

        Returns:
            GP confidence in the range ``[0, 1]``.
        """
        c_skill = config.skill_confidence
        if not math.isfinite(c_skill):
            c_skill = 0.0
        c_skill = clamp(c_skill, 0.0, 1.0)
        g_skill = 1.0 if c_skill >= config.gp_skill_min else 0.0
        c_var = math.exp(-config.gp_k_sigma * max(0.0, config.point_variance))
        c_chunk = math.exp(
            -config.gp_k_chunk * max(0.0, config.chunk_error)
            / max(config.gp_error_fail, 1e-9)
        )
        progress = config.progress if math.isfinite(config.progress) else 0.0
        c_progress = 1.0 / (
            1.0 + math.exp(-config.gp_k_progress * (progress - config.gp_progress_midpoint))
        )
        w_sigma, w_chunk, w_progress = normalized_weights(
            config.gp_w_sigma, config.gp_w_chunk, config.gp_w_progress
        )
        return g_skill * (
            c_var ** w_sigma * c_chunk ** w_chunk * c_progress ** w_progress
        ) ** max(config.gp_gamma, 0.0)

    def prediction_weight_online(self, config):
        """Compute the weighted-policy authority with online alpha filtering.

        Args:
            config: Fusion configuration for the current replay sample.

        Returns:
            Filtered prediction weight bounded by the configured min/max values.
        """
        c_net = self.compute_network_confidence(config)
        c_gp = self.compute_gp_confidence(config) * config.confidence_gain
        requested = c_gp / (c_net + c_gp + max(config.authority_eps, 1e-12))
        self._filtered_prediction_weight = clamp(
            self._filtered_prediction_weight + 0.1 * (
                requested - self._filtered_prediction_weight
            ),
            config.min_prediction_weight,
            config.max_prediction_weight,
        )
        return self._filtered_prediction_weight

    @staticmethod
    def latest_leader_pose(leader, elapsed, timeout):
        """Return the latest non-stale leader pose at a replay timestamp.

        Args:
            leader: Time-ordered leader ``TimedPose`` samples.
            elapsed: Replay time in seconds.
            timeout: Maximum permitted leader sample age in seconds.

        Returns:
            Latest leader pose, or ``None`` when it is unavailable or stale.
        """
        latest = None
        latest_time = None
        for sample in leader:
            if sample.time > elapsed:
                break
            latest = sample.pose
            latest_time = sample.time
        if latest is None or elapsed - latest_time > timeout:
            return None
        return latest

    def fuse_pose_online(self, predicted_pose, leader_pose, config):
        """Fuse poses with the selected online-fuser policy.

        Args:
            predicted_pose: GP pose sampled at the current replay time.
            leader_pose: Latest non-stale TDPA leader pose, if available.
            config: Fusion configuration for the current replay sample.

        Returns:
            Tuple of the online-fuser-equivalent pose and its alpha diagnostic.
        """
        if config.fusion_policy == 'weighted_blending':
            alpha = self.prediction_weight_online(config)
            return blend_pose(predicted_pose, leader_pose, alpha), alpha

        c_net = self.compute_network_confidence(config)
        c_gp = self.compute_gp_confidence(config)
        g_skill = 1.0 if c_gp > 0.0 else 0.0
        if config.fusion_policy == 'optimal_blending':
            conflict = pose_conflict(leader_pose, predicted_pose) if leader_pose else 0.0
            fused_pose, result = optimal_blend_pose(
                predicted_pose, leader_pose, c_net=c_net, c_gp=c_gp,
                alpha_prev=self._previous_optimal_weight, d=conflict,
                lambda_s=config.optimal_lambda_s, lambda_c=config.optimal_lambda_c,
                g_skill=g_skill, confidence_gain=config.confidence_gain,
                min_prediction_weight=config.min_prediction_weight,
                max_prediction_weight=config.max_prediction_weight,
                authority_eps=config.authority_eps,
            )
            self._previous_optimal_weight = result.alpha_g
            return fused_pose, result.alpha_g

        current_pose = self._last_fused_pose or leader_pose or predicted_pose
        fused_pose, _ = nash_blend_pose(
            current_pose, predicted_pose, leader_pose, c_net=c_net, c_gp=c_gp,
            g_skill=g_skill, confidence_gain=config.confidence_gain,
            human_effort=config.nash_human_effort, gp_effort=config.nash_gp_effort,
            human_agreement=config.nash_human_agreement,
            gp_agreement=config.nash_gp_agreement,
            agreement_ratio=config.nash_agreement_ratio,
            rotation_weight=config.nash_rotation_weight, eps=config.authority_eps,
        )
        # Nash has no scalar alpha result; record the same online confidence
        # authority for the passivity-controller diagnostic.
        return fused_pose, self.prediction_weight_online(config)

    @staticmethod
    def replay_times(duration, rate):
        """Generate online-fuser timer ticks through a prediction duration.

        Args:
            duration: Prediction duration in seconds.
            rate: Fuser timer frequency in hertz.

        Returns:
            Monotonic replay timestamps, including the exact final duration.
        """
        period = 1.0 / rate if rate > 0.0 else 0.005
        times = [index * period for index in range(int(math.floor(duration / period)) + 1)]
        if not times or duration - times[-1] > 1e-12:
            times.append(duration)
        return times

    def fuse_trial(
        self,
        prediction_path,
        leader_path,
        output_path,
        config,
        metrics,
        leader_time_relative,
    ):
        """Replay one trial using the online fuser timer and policy logic.

        The requested alpha is passed through the alpha-conditioning portion of
        ``AutonomyPassivityController`` and saved as ``autonomy_level``. The
        pose remains the unmodified online-fuser result: the C++ energy tank
        acts downstream on wrenches, which are not present in a pose CSV.

        Args:
            prediction_path: Path to the saved prediction trajectory CSV.
            leader_path: Path to the saved leader execution trajectory CSV.
            output_path: Destination path for the fused CSV.
            config: Base fusion configuration.
            metrics: Reconstructed prediction confidence metrics.
            leader_time_relative: Whether to align leader time to its first
                sample before replay.

        Returns:
            Number of fused samples, alpha minimum/maximum, and autonomy-level
            minimum/maximum, in that order.
        """
        prediction = read_trajectory(prediction_path)
        leader = read_trajectory(leader_path)
        prediction_poses = [sample.pose for sample in prediction]
        prediction_times = [sample.time for sample in prediction]
        leader_t0 = leader[0].time if leader_time_relative else 0.0
        leader = [TimedPose(max(0.0, sample.time - leader_t0), sample.pose) for sample in leader]

        self._previous_optimal_weight = 0.0
        self._last_fused_pose = None
        autonomy_controller = AutonomyPassivityController(
            config.autonomy_alpha_time_constant,
            config.autonomy_alpha_rate_limit,
        )
        fused = []
        alphas = []
        autonomy_levels = []
        previous_elapsed = 0.0
        for elapsed in self.replay_times(max(0.0, prediction_times[-1]), config.rate):
            predicted_pose = sample_timed_pose(prediction_poses, prediction_times, elapsed)
            if predicted_pose is None:
                continue
            leader_pose = self.latest_leader_pose(
                leader, elapsed, max(0.0, config.leader_timeout_sec)
            )
            point_variance = sample_timed_value(
                metrics.point_variances, prediction_times, elapsed, metrics.trajectory_variance
            )
            sample_config = replace(
                config,
                skill_confidence=metrics.skill_confidence,
                point_variance=point_variance,
                chunk_error=metrics.chunk_error,
                progress=metrics.progress,
            )
            fused_pose, alpha = self.fuse_pose_online(
                predicted_pose, leader_pose, sample_config
            )
            autonomy_level = autonomy_controller.update_alpha(alpha, elapsed - previous_elapsed)
            self._last_fused_pose = fused_pose
            fused.append(TimedPose(elapsed, fused_pose, alpha, autonomy_level))
            alphas.append(alpha)
            autonomy_levels.append(autonomy_level)
            previous_elapsed = elapsed

        write_trajectory(output_path, fused)
        return len(fused), min(alphas), max(alphas), min(autonomy_levels), max(autonomy_levels)


def main(args=None):
    """Run the offline fusion test node.

    Args:
        args: Optional ROS command-line arguments.
    """
    rclpy.init(args=args)
    node = OfflineFusionTestNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
