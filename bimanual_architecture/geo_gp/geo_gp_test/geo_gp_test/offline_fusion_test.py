"""Offline CSV-based test node for Geo-GP trajectory fusion."""

import csv
from dataclasses import dataclass, replace
import json
import math
from pathlib import Path

from geo_gp_fusion.policies.nash_blending import nash_blend_pose
from geo_gp_fusion.policies.optimal_blending import optimal_blend_pose, pose_conflict
from geo_gp_fusion.policies.weighted_blending import blend_pose, sample_timed_pose
from geo_gp_interfaces.msg import PromptTrajectory
from geo_gp_prediction.predictor import Predictor
from geometry.frame6d import estimate_rotation_scale_3d_search_by_count
from geometry.resample import resample_by_arclen_fraction
from geometry_msgs.msg import Pose, Vector3
import numpy as np
import rclpy
from rclpy.node import Node


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
    tank_energy: float = None
    passivity_lambda: float = None
    passivity_power: float = None
    autonomy_wrench_norm: float = None


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
    tdpa_enabled: bool
    tdpa_delay_sec: float
    optimal_lambda_s: float
    optimal_lambda_c: float
    nash_human_effort: float
    nash_gp_effort: float
    nash_human_agreement: float
    nash_gp_agreement: float
    nash_agreement_ratio: float
    nash_rotation_weight: float

    simulate_mujoco: bool
    mujoco_model_path: str
    mujoco_site_name: str
    mujoco_timestep: float
    mujoco_settle_time_sec: float
    mujoco_initialize_from_leader: bool
    mujoco_pos_stiffness: float
    mujoco_rot_stiffness: float
    mujoco_nullspace_stiffness: float
    mujoco_torque_rate_limit: float
    autonomy_tank_initial_energy: float
    autonomy_tank_max_energy: float
    autonomy_tank_recharge_efficiency: float
    autonomy_tank_power_epsilon: float
    autonomy_tank_velocity_epsilon: float
    autonomy_wrench_max_abs: float


class EnergyTank:
    """Numerically identical offline implementation of the C++ energy tank.

    Args:
        initial_energy: Initial stored energy in joules.
        max_energy: Upper bound on stored energy in joules.
        recharge_efficiency: Fraction of absorbed energy stored by the tank.
        power_epsilon: Numerical threshold for power classification.
        velocity_epsilon: Minimum Cartesian speed used to calculate power.
        wrench_max_abs: Per-axis absolute wrench limit.
    """

    def __init__(self, initial_energy, max_energy, recharge_efficiency,
                 power_epsilon, velocity_epsilon, wrench_max_abs):
        """Initialize the tank with bounded energy and safety parameters.

        Args:
            initial_energy: Initial stored energy in joules.
            max_energy: Upper bound on stored energy in joules.
            recharge_efficiency: Fraction of absorbed energy stored by the tank.
            power_epsilon: Numerical threshold for power classification.
            velocity_epsilon: Minimum Cartesian speed used to calculate power.
            wrench_max_abs: Per-axis absolute wrench limit.
        """
        self.initial_energy = float(initial_energy)
        self.max_energy = max(0.0, float(max_energy))
        self.recharge_efficiency = clamp(float(recharge_efficiency), 0.0, 1.0)
        self.power_epsilon = max(0.0, float(power_epsilon))
        self.velocity_epsilon = max(0.0, float(velocity_epsilon))
        self.wrench_max_abs = max(0.0, float(wrench_max_abs))
        self.energy = clamp(self.initial_energy, 0.0, self.max_energy)

    def update(self, wrench_autonomy, velocity, dt):
        """Scale an autonomy wrench according to available tank energy.

        Args:
            wrench_autonomy: Six-dimensional autonomy wrench.
            velocity: Actual six-dimensional end-effector velocity.
            dt: Physics time step in seconds.

        Returns:
            Tuple of safe wrench, wrench scale, and pre-scaling power.
        """
        wrench = np.clip(np.nan_to_num(wrench_autonomy),
                         -self.wrench_max_abs, self.wrench_max_abs)
        valid_velocity = bool(np.all(np.isfinite(velocity)))
        speed = float(np.linalg.norm(velocity)) if valid_velocity else 0.0
        power = float(np.dot(wrench, velocity)) if (
            dt > 0.0 and valid_velocity and speed > self.velocity_epsilon
        ) else 0.0
        scale = 1.0
        if power > self.power_epsilon and dt > 0.0:
            demand = power * dt
            scale = clamp(self.energy / (demand + self.power_epsilon), 0.0, 1.0)
            wrench *= scale
            self.energy -= scale * demand
        elif power < -self.power_epsilon and dt > 0.0:
            self.energy += self.recharge_efficiency * (-power) * dt
        self.energy = clamp(float(np.nan_to_num(self.energy)), 0.0, self.max_energy)
        return wrench, scale, power


@dataclass
class AutonomyPassivityOutput:
    """Offline counterpart of the C++ ``AutonomyPassivityOutput``.

    Attributes:
        wrench_autonomy_safe: Passivity-limited autonomy wrench.
        lambda_value: Applied autonomy-wrench scale.
        power: Autonomy-wrench power before scaling.
        tank_energy: Remaining energy after the update.
    """

    wrench_autonomy_safe: np.ndarray
    lambda_value: float
    power: float
    tank_energy: float


class AutonomyPassivityController:
    """Apply the same autonomy-wrench separation as the C++ controller.

    Args:
        tank: Energy tank used to limit the autonomy component.
    """

    def __init__(self, tank):
        """Initialize the controller with its autonomy energy tank.

        Args:
            tank: Energy tank used to limit the autonomy component.
        """
        self.tank = tank

    def update(self, wrench_leader, wrench_total, velocity, dt):
        """Limit only the wrench introduced by the fused autonomous target.

        Args:
            wrench_leader: Wrench generated by the leader-only target.
            wrench_total: Wrench generated by the fused target.
            velocity: Actual six-dimensional end-effector velocity.
            dt: Physics time step in seconds.

        Returns:
            AutonomyPassivityOutput containing the safe autonomy wrench and
            matching energy-tank diagnostics.
        """
        safe, lambda_value, power = self.tank.update(wrench_total - wrench_leader, velocity, dt)
        return AutonomyPassivityOutput(safe, lambda_value, power, self.tank.energy)


def pose_rotation(pose):
    """Convert a ROS pose orientation to a rotation matrix.

    Args:
        pose: ROS pose with an xyzw quaternion orientation.

    Returns:
        Three-by-three world-frame rotation matrix.
    """
    x, y, z, w = (pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(((1 - 2 * (y*y + z*z), 2 * (x*y-z*w), 2 * (x*z+y*w)),
                     (2 * (x*y+z*w), 1 - 2 * (x*x+z*z), 2 * (y*z-x*w)),
                     (2 * (x*z-y*w), 2 * (y*z+x*w), 1 - 2 * (x*x+y*y))))


def quaternion_from_rotation(rotation):
    """Convert a three-by-three rotation matrix to an xyzw quaternion.

    Args:
        rotation: Three-by-three world-frame rotation matrix.

    Returns:
        Tuple of four quaternion components in xyzw order.
    """
    matrix = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        quaternion = np.array((
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        ))
    else:
        index = int(np.argmax(np.diag(matrix)))
        next_index, last_index = (index + 1) % 3, (index + 2) % 3
        scale = 2.0 * math.sqrt(max(
            0.0,
            1.0 + matrix[index, index] - matrix[next_index, next_index]
            - matrix[last_index, last_index],
        ))
        if scale < 1e-12:
            return (0.0, 0.0, 0.0, 1.0)
        quaternion = np.zeros(4)
        quaternion[index] = 0.25 * scale
        quaternion[next_index] = (
            matrix[next_index, index] + matrix[index, next_index]
        ) / scale
        quaternion[last_index] = (
            matrix[last_index, index] + matrix[index, last_index]
        ) / scale
        quaternion[3] = (
            matrix[last_index, next_index] - matrix[next_index, last_index]
        ) / scale
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12 or not math.isfinite(norm):
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(quaternion / norm)


def rotation_error(current, desired):
    """Compute the world-frame rotation-vector error.

    Args:
        current: Current three-by-three rotation matrix.
        desired: Desired three-by-three rotation matrix.

    Returns:
        Three-dimensional world-frame rotation error.
    """
    relative = desired @ current.T
    angle = math.acos(clamp((float(np.trace(relative)) - 1.0) * 0.5, -1.0, 1.0))
    axis = np.array((relative[2, 1] - relative[1, 2],
                     relative[0, 2] - relative[2, 0],
                     relative[1, 0] - relative[0, 1]))
    axis_norm = float(np.linalg.norm(axis))
    return np.zeros(3) if angle < 1e-9 or axis_norm < 1e-9 else axis * angle / axis_norm


@dataclass
class TDPACartesianState:
    """Store one delayed Cartesian TDPA message.

    Attributes:
        velocity: Six-dimensional Cartesian velocity from the leader side.
        wrench: Six-dimensional Cartesian wrench from the follower side.
        energy_linear: Accumulated linear-channel TDPA energy.
        energy_rotational: Accumulated rotational-channel TDPA energy.
    """

    velocity: np.ndarray
    wrench: np.ndarray
    energy_linear: float
    energy_rotational: float


class CartesianTDPAPort:
    """Represent one three-axis Cartesian TDPA energy port.

    Attributes:
        energy_in: Energy received through the local port.
        energy_out: Energy delivered through the local port.
        energy_delayed: Most recently received remote energy value.
        energy_dissipated: Energy dissipated by the TDPA correction.
    """

    def __init__(self):
        """Initialize empty local, remote, and dissipated energy counters."""
        self.energy_in = 0.0
        self.energy_out = 0.0
        self.energy_delayed = 0.0
        self.energy_dissipated = 0.0

    def observe(self, velocity, wrench, dt):
        """Update local TDPA energy flows from a power sample.

        Args:
            velocity: Three-dimensional Cartesian velocity at this port.
            wrench: Three-dimensional Cartesian wrench at this port.
            dt: Integration time step in seconds.
        """
        if dt <= 0.0 or not np.all(np.isfinite(velocity)):
            return
        power = float(np.dot(velocity, wrench))
        if not math.isfinite(power):
            return
        self.energy_in += max(0.0, power) * dt
        self.energy_out += max(0.0, -power) * dt

    def limit_force(self, wrench, velocity, remote_energy, dt):
        """Apply the leader-side TDPA force correction.

        Args:
            wrench: Three-dimensional force before the TDPA correction.
            velocity: Three-dimensional local leader velocity.
            remote_energy: Delayed energy reported by the follower port.
            dt: Integration time step in seconds.

        Returns:
            Passivity-corrected three-dimensional leader force.
        """
        if dt <= 0.0:
            return wrench
        self.energy_delayed = float(remote_energy)
        shortage = self.energy_delayed - self.energy_out + self.energy_dissipated
        velocity_norm_sq = float(np.dot(velocity, velocity))
        if shortage >= 0.0 or velocity_norm_sq <= 1e-6:
            return wrench
        alpha = -shortage / (dt * velocity_norm_sq)
        correction = alpha * velocity
        self.energy_dissipated += dt * float(np.dot(velocity, correction))
        return wrench + correction

    def limit_velocity(self, velocity, wrench, remote_energy, dt):
        """Apply the follower-side TDPA velocity correction.

        Args:
            velocity: Three-dimensional velocity before the TDPA correction.
            wrench: Three-dimensional local follower wrench.
            remote_energy: Delayed energy reported by the leader port.
            dt: Integration time step in seconds.

        Returns:
            Passivity-corrected three-dimensional follower velocity.
        """
        if dt <= 0.0:
            return velocity
        self.energy_delayed = float(remote_energy)
        shortage = self.energy_delayed - self.energy_out + self.energy_dissipated
        wrench_norm_sq = float(np.dot(wrench, wrench))
        if shortage >= 0.0 or wrench_norm_sq <= 1e-6:
            return velocity
        beta = -shortage / (dt * wrench_norm_sq)
        correction = beta * wrench
        self.energy_dissipated += dt * float(np.dot(wrench, correction))
        return velocity + correction


def integrate_rotation_world(rotation, angular_velocity, dt):
    """Integrate world-frame angular velocity over one TDPA step.

    Args:
        rotation: Current three-by-three world-frame rotation matrix.
        angular_velocity: Three-dimensional world-frame angular velocity.
        dt: Integration time step in seconds.

    Returns:
        Updated three-by-three world-frame rotation matrix.
    """
    angle = float(np.linalg.norm(angular_velocity)) * dt
    if angle <= 1e-12:
        return rotation
    axis = angular_velocity / float(np.linalg.norm(angular_velocity))
    skew = np.array((
        (0.0, -axis[2], axis[1]),
        (axis[2], 0.0, -axis[0]),
        (-axis[1], axis[0], 0.0),
    ))
    delta = np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * skew @ skew
    return delta @ rotation


class OfflineCartesianTDPA:
    """Replay the leader/follower Cartesian TDPA state exchange.

    The implementation mirrors the two split three-axis TDPA ports used by the
    controller. Messages carry leader velocity, follower wrench feedback, and
    their accumulated energy; both directions share the configured one-way
    delay.
    """

    def __init__(self, initial_pose, delay_sec):
        """Initialize a delayed Cartesian TDPA replay.

        Args:
            initial_pose: Initial Cartesian reference pose for the follower.
            delay_sec: One-way communication delay in seconds.
        """
        self.reference_position = np.array((
            initial_pose.position.x,
            initial_pose.position.y,
            initial_pose.position.z,
        ))
        self.reference_rotation = pose_rotation(initial_pose)
        self.delay_sec = max(0.0, float(delay_sec))
        self.leader_linear = CartesianTDPAPort()
        self.leader_rotational = CartesianTDPAPort()
        self.follower_linear = CartesianTDPAPort()
        self.follower_rotational = CartesianTDPAPort()
        self.latest_leader = TDPACartesianState(
            np.zeros(6), np.zeros(6), 0.0, 0.0
        )
        self.latest_follower = TDPACartesianState(
            np.zeros(6), np.zeros(6), 0.0, 0.0
        )
        self.leader_queue = []
        self.follower_queue = []

    @staticmethod
    def _deliver(queue, elapsed, current):
        """Deliver all TDPA messages whose arrival time has elapsed.

        Args:
            queue: Time-ordered queue of arrival-time/state pairs.
            elapsed: Current replay time in seconds.
            current: Most recently delivered TDPA state.

        Returns:
            Latest TDPA state available at ``elapsed``.
        """
        while queue and queue[0][0] <= elapsed + 1e-12:
            _, current = queue.pop(0)
        return current

    def pose(self):
        """Return the TDPA-integrated follower Cartesian reference.

        Returns:
            Current follower target pose after velocity integration.
        """
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = self.reference_position
        (pose.orientation.x, pose.orientation.y, pose.orientation.z,
         pose.orientation.w) = quaternion_from_rotation(self.reference_rotation)
        return pose

    def step(self, elapsed, leader_velocity, follower_wrench, dt):
        """Exchange delayed TDPA states and advance the follower reference.

        Args:
            elapsed: Current replay time in seconds.
            leader_velocity: Current six-dimensional raw leader velocity.
            follower_wrench: Current six-dimensional follower control wrench.
            dt: Replay integration time step in seconds.

        Returns:
            TDPA-integrated follower target pose.
        """
        self.latest_leader = self._deliver(
            self.leader_queue, elapsed, self.latest_leader
        )
        self.latest_follower = self._deliver(
            self.follower_queue, elapsed, self.latest_follower
        )
        leader_velocity = np.nan_to_num(np.asarray(leader_velocity, dtype=np.float64))
        follower_wrench = np.nan_to_num(np.asarray(follower_wrench, dtype=np.float64))

        leader_wrench = self.latest_follower.wrench.copy()
        self.leader_linear.observe(
            leader_velocity[:3], leader_wrench[:3], dt
        )
        self.leader_rotational.observe(
            leader_velocity[3:], leader_wrench[3:], dt
        )
        leader_wrench[:3] = self.leader_linear.limit_force(
            leader_wrench[:3],
            leader_velocity[:3],
            self.latest_follower.energy_linear,
            dt,
        )
        leader_wrench[3:] = self.leader_rotational.limit_force(
            leader_wrench[3:],
            leader_velocity[3:],
            self.latest_follower.energy_rotational,
            dt,
        )
        leader_state = TDPACartesianState(
            leader_velocity,
            leader_wrench,
            self.leader_linear.energy_in,
            self.leader_rotational.energy_in,
        )
        self.leader_queue.append((elapsed + self.delay_sec, leader_state))

        desired_velocity = self.latest_leader.velocity.copy()
        self.follower_linear.observe(
            desired_velocity[:3], follower_wrench[:3], dt
        )
        self.follower_rotational.observe(
            desired_velocity[3:], follower_wrench[3:], dt
        )
        desired_velocity[:3] = self.follower_linear.limit_velocity(
            desired_velocity[:3],
            follower_wrench[:3],
            self.latest_leader.energy_linear,
            dt,
        )
        desired_velocity[3:] = self.follower_rotational.limit_velocity(
            desired_velocity[3:],
            follower_wrench[3:],
            self.latest_leader.energy_rotational,
            dt,
        )
        self.reference_position += dt * desired_velocity[:3]
        self.reference_rotation = integrate_rotation_world(
            self.reference_rotation,
            desired_velocity[3:],
            dt,
        )
        follower_state = TDPACartesianState(
            desired_velocity,
            follower_wrench,
            self.follower_linear.energy_in,
            self.follower_rotational.energy_in,
        )
        self.follower_queue.append((elapsed + self.delay_sec, follower_state))
        return self.pose()


class MujocoAutonomyPassivitySimulator:
    """Replay Panda dynamics and supply physical state to the passivity controller.

    Args:
        model_path: MuJoCo XML model path.
        site_name: End-effector site name.
        timestep: MuJoCo physics time step in seconds.
        initial_q: Initial positions of Panda joints one through seven.
        config: Offline fusion and passivity configuration.
        initial_pose: Optional leader pose used to initialize the end effector.
    """

    def __init__(
        self,
        model_path,
        site_name,
        timestep,
        initial_q,
        config,
        initial_pose=None,
    ):
        """Load the MuJoCo model and initialize the torque-controlled Panda.

        Args:
            model_path: MuJoCo XML model path.
            site_name: End-effector site name.
            timestep: MuJoCo physics time step in seconds.
            initial_q: Initial positions of Panda joints one through seven.
            config: Offline fusion and passivity configuration.
            initial_pose: Optional leader pose used for Cartesian IK initialization.

        Raises:
            FileNotFoundError: If the MuJoCo model path does not exist.
            RuntimeError: If the Python MuJoCo package is unavailable.
            ValueError: If the model does not expose the required Panda elements.
        """
        try:
            import mujoco
        except ImportError as exc:
            raise RuntimeError(
                'simulate_mujoco=true requires Python MuJoCo (pip install mujoco>=3.2).'
            ) from exc
        self.mujoco = mujoco
        path = expand_path(model_path)
        if not path.is_file():
            raise FileNotFoundError(f'MuJoCo model does not exist: {path}')
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        self.site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if self.site_id < 0:
            raise ValueError(f'MuJoCo site {site_name!r} does not exist')
        if len(initial_q) != 7:
            raise ValueError('mujoco_initial_joint_positions must have seven values')
        self.joints, self.qpos, self.dof, self.actuators = [], [], [], []
        for index in range(1, 8):
            joint = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, f'panda_joint{index}')
            actuator = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f'panda_act_trq{index}')
            if joint < 0 or actuator < 0:
                raise ValueError('Model must contain panda_joint1..7 and panda_act_trq1..7')
            self.joints.append(joint)
            self.qpos.append(int(self.model.jnt_qposadr[joint]))
            self.dof.append(int(self.model.jnt_dofadr[joint]))
            self.actuators.append(actuator)
        # scene.xml also defines position/velocity/gripper actuators. They
        # would apply forces at their default zero controls and do not exist in
        # the torque-only Cartesian controller, so disable them for this replay.
        torque_actuators = set(self.actuators)
        for actuator in range(self.model.nu):
            if actuator not in torque_actuators:
                self.model.actuator_gainprm[actuator, :] = 0.0
                self.model.actuator_biasprm[actuator, :] = 0.0
        self.data.qpos[self.qpos] = np.asarray(initial_q, dtype=np.float64)
        self.model.opt.timestep = max(float(timestep), 1e-5)
        self.stiffness = np.array([config.mujoco_pos_stiffness] * 3 +
                                  [config.mujoco_rot_stiffness] * 3)
        self.damping = np.array([2.0 * math.sqrt(config.mujoco_pos_stiffness)] * 3 +
                                [1.6 * math.sqrt(config.mujoco_rot_stiffness)] * 3)
        self.autonomy_pc = AutonomyPassivityController(EnergyTank(
            config.autonomy_tank_initial_energy, config.autonomy_tank_max_energy,
            config.autonomy_tank_recharge_efficiency, config.autonomy_tank_power_epsilon,
            config.autonomy_tank_velocity_epsilon, config.autonomy_wrench_max_abs,
        ))
        self.nullspace_stiffness = max(0.0, config.mujoco_nullspace_stiffness)
        self.torque_rate_limit = max(0.0, config.mujoco_torque_rate_limit)
        self.desired_nullspace_q = np.asarray(initial_q, dtype=np.float64).copy()
        self.previous_torque = None
        self.last_command_wrench = np.zeros(6)
        mujoco.mj_forward(self.model, self.data)
        self.initialization_error = None
        if initial_pose is not None:
            self.initialization_error = self.initialize_to_pose(initial_pose)
        self.desired_nullspace_q = self.data.qpos[self.qpos].copy()

    def current_pose(self):
        """Return the current simulated end-effector pose.

        Returns:
            Pose of the MuJoCo end-effector site.
        """
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = self.data.site_xpos[self.site_id]
        quaternion = quaternion_from_rotation(
            self.data.site_xmat[self.site_id].reshape(3, 3)
        )
        (pose.orientation.x, pose.orientation.y, pose.orientation.z,
         pose.orientation.w) = quaternion
        return pose

    def initialize_to_pose(self, target_pose):
        """Use damped least-squares IK to align the Panda with a target pose.

        The saved leader CSV contains Cartesian poses rather than joint
        positions. The configured joint positions are therefore only used as
        an IK seed; this method makes the simulated end effector start at the
        first leader pose.

        Args:
            target_pose: Leader pose used as the desired initial end-effector
                pose.

        Returns:
            Tuple of final position error, final rotation error, and convergence
            flag.
        """
        target_position = np.array((
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z,
        ))
        target_rotation = pose_rotation(target_pose)
        position_tolerance = 1e-4
        rotation_tolerance = 1e-3
        for _ in range(250):
            self.mujoco.mj_forward(self.model, self.data)
            position = self.data.site_xpos[self.site_id]
            rotation = self.data.site_xmat[self.site_id].reshape(3, 3)
            position_error = target_position - position
            orientation_error = rotation_error(rotation, target_rotation)
            if (
                float(np.linalg.norm(position_error)) <= position_tolerance
                and float(np.linalg.norm(orientation_error)) <= rotation_tolerance
            ):
                break
            jacobian_position = np.zeros((3, self.model.nv))
            jacobian_rotation = np.zeros((3, self.model.nv))
            self.mujoco.mj_jacSite(
                self.model,
                self.data,
                jacobian_position,
                jacobian_rotation,
                self.site_id,
            )
            jacobian = np.vstack((
                jacobian_position[:, self.dof],
                jacobian_rotation[:, self.dof],
            ))
            error = np.r_[position_error, orientation_error]
            damping = 1e-3
            joint_delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping * np.eye(6),
                error,
            )
            joint_delta = np.clip(joint_delta, -0.1, 0.1)
            for joint, qpos, delta in zip(self.joints, self.qpos, joint_delta):
                low, high = self.model.jnt_range[joint]
                self.data.qpos[qpos] = clamp(
                    float(self.data.qpos[qpos] + delta),
                    float(low),
                    float(high),
                )
        self.data.qvel[self.dof] = 0.0
        self.mujoco.mj_forward(self.model, self.data)
        position_error = float(np.linalg.norm(
            target_position - self.data.site_xpos[self.site_id]
        ))
        orientation_error = float(np.linalg.norm(rotation_error(
            self.data.site_xmat[self.site_id].reshape(3, 3),
            target_rotation,
        )))
        return (
            position_error,
            orientation_error,
            position_error <= position_tolerance
            and orientation_error <= rotation_tolerance,
        )

    def advance(self, leader_pose, fused_pose, duration):
        """Advance simulation while applying passivity-limited Cartesian wrench.

        Args:
            leader_pose: Leader-only Cartesian target pose.
            fused_pose: Fused Cartesian target pose.
            duration: Time to advance in seconds.

        Returns:
            Tuple of actual end-effector pose, tank energy, lambda, power, and
            safe autonomy-wrench norm from the final physics step.
        """
        leader_p = np.array((
            leader_pose.position.x, leader_pose.position.y, leader_pose.position.z,
        ))
        fused_p = np.array((
            fused_pose.position.x, fused_pose.position.y, fused_pose.position.z,
        ))
        leader_r, fused_r = pose_rotation(leader_pose), pose_rotation(fused_pose)
        remaining = max(0.0, float(duration))
        result = (self.autonomy_pc.tank.energy, 1.0, 0.0, 0.0)
        while remaining > 1e-12:
            dt = min(remaining, self.model.opt.timestep)
            jacp, jacr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
            self.mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self.site_id)
            velocity = np.r_[jacp @ self.data.qvel, jacr @ self.data.qvel]
            position = self.data.site_xpos[self.site_id]
            rotation = self.data.site_xmat[self.site_id].reshape(3, 3)
            leader_error = np.r_[leader_p - position, rotation_error(rotation, leader_r)]
            total_error = np.r_[fused_p - position, rotation_error(rotation, fused_r)]
            f_leader = self.stiffness * leader_error - self.damping * velocity
            f_total = self.stiffness * total_error - self.damping * velocity
            passivity = self.autonomy_pc.update(f_leader, f_total, velocity, dt)
            safe = passivity.wrench_autonomy_safe
            self.last_command_wrench = f_leader + safe
            jacobian = np.vstack((jacp[:, self.dof], jacr[:, self.dof]))
            joint_position = self.data.qpos[self.qpos]
            joint_velocity = self.data.qvel[self.dof]
            nullspace_command = (
                self.nullspace_stiffness * (self.desired_nullspace_q - joint_position)
                - 2.0 * math.sqrt(self.nullspace_stiffness) * joint_velocity
            )
            projector = np.eye(len(self.dof)) - jacobian.T @ np.linalg.pinv(jacobian.T)
            nullspace_torque = projector @ nullspace_command
            # The scene applies gravity through gravcomp=1. qfrc_bias includes
            # that same term, so remove qfrc_gravcomp to avoid double compensation.
            requested_torque = (
                jacobian.T @ (f_leader + safe)
                + self.data.qfrc_bias[self.dof]
                - self.data.qfrc_gravcomp[self.dof]
                + nullspace_torque
            )
            if self.previous_torque is None or self.torque_rate_limit <= 0.0:
                torque = requested_torque
            else:
                max_delta = self.torque_rate_limit * dt
                torque = np.clip(
                    requested_torque,
                    self.previous_torque - max_delta,
                    self.previous_torque + max_delta,
                )
            applied_torque = np.empty(len(self.actuators))
            for index, (actuator, value) in enumerate(zip(self.actuators, torque)):
                lo, hi = self.model.actuator_ctrlrange[actuator]
                applied_torque[index] = clamp(float(value), float(lo), float(hi))
                self.data.ctrl[actuator] = applied_torque[index]
            self.previous_torque = applied_torque
            self.mujoco.mj_step(self.model, self.data)
            remaining -= dt
            result = (passivity.tank_energy, passivity.lambda_value, passivity.power,
                      float(np.linalg.norm(safe)))
        return self.current_pose(), *result


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


def sample_timed_value(values, times, elapsed, default=0.0):
    """Sample the nearest value from a time-indexed sequence.

    Args:
        values: Values corresponding to ``times``.
        times: Sample timestamps in seconds.
        elapsed: Replay time in seconds.
        default: Value returned when the sequence cannot be sampled.

    Returns:
        The nearest time-aligned value, or ``default`` when unavailable.
    """
    if not values:
        return default
    if len(values) == 1 or not times:
        return values[0]
    count = min(len(values), len(times))
    if count <= 0:
        return default
    if elapsed <= times[0]:
        return values[0]
    if elapsed >= times[count - 1]:
        return values[count - 1]
    high = 1
    while high < count and times[high] < elapsed:
        high += 1
    low = max(0, high - 1)
    if high >= count:
        return values[count - 1]
    return values[low] if elapsed - times[low] <= times[high] - elapsed else values[high]


def write_trajectory(csv_path, samples):
    """Write a fused pose trajectory CSV.

    Args:
        csv_path: Destination CSV path.
        samples: Timed fused poses to write.

    The output contains the requested fusion authority and, when enabled,
    MuJoCo energy-tank diagnostics.
    """
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    has_alpha = any(sample.alpha is not None for sample in samples)
    fieldnames = POSE_FIELDS + (('alpha',) if has_alpha else ())
    passivity_fields = ('tank_energy', 'passivity_lambda', 'passivity_power',
                        'autonomy_wrench_norm')
    has_passivity = any(sample.tank_energy is not None for sample in samples)
    if has_passivity:
        fieldnames += passivity_fields
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
            if has_passivity:
                for field in passivity_fields:
                    value = getattr(sample, field)
                    row[field] = f'{value:.6f}' if value is not None else ''
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


def controlled_output_path(fused_path):
    """Build the simulated controller-output path from a fused-target path.

    Args:
        fused_path: Path to a ``fused_success_*.csv`` trajectory.

    Returns:
        Path to a ``controlled_success_*.csv`` trajectory.
    """
    return fused_path.with_name(
        fused_path.name.replace('fused_success_', 'controlled_success_', 1)
    )


def fused_output_path(
        trial_dir, input_root, output_dir, prediction_path, fusion_policy):
    """Build the destination path for one fused trajectory.

    Args:
        trial_dir: Directory containing the input trial CSV files.
        input_root: Root path used to preserve relative layout under
            ``output_dir``.
        output_dir: Optional output root. If ``None``, output is written beside
            the input CSV files.
        prediction_path: Prediction CSV whose timestamp is reused for the fused
            CSV filename.
        fusion_policy: Normalized policy name appended to the output filename.

    Returns:
        Destination path named ``fused_success_<timestamp>_<policy>.csv``.
    """
    suffix = prediction_path.name.replace('prediction_success_', 'fused_success_', 1)
    suffix_path = Path(suffix)
    suffix = f'{suffix_path.stem}_{fusion_policy}{suffix_path.suffix}'
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
        """Initialize the node and declare all offline replay parameters."""
        super().__init__('offline_fusion_test')
        self.declare_parameter('input_path', '~/geo-gp/data/06-02/preds')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('recursive', True)
        self.declare_parameter('overwrite', True)
        self.declare_parameter('leader_time_relative', False)
        self.declare_parameter('leader_delay_sec', 0.0)
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
        self.declare_parameter('tdpa_enabled', True)
        self.declare_parameter('tdpa_delay_sec', 0.0)
        # Optional physical replay. Values mirror cartesian_impedance_controller
        self.declare_parameter('simulate_mujoco', False)
        self.declare_parameter(
            'mujoco_model_path',
            '/home/user/humble_ws/src/bimanual_architecture/'
            'franka_description/mujoco/franka/scene.xml',
        )
        self.declare_parameter('mujoco_site_name', 'panda_ee_site')
        self.declare_parameter('mujoco_timestep', 0.001)
        self.declare_parameter('mujoco_settle_time_sec', 0.0)
        self.declare_parameter('mujoco_initialize_from_leader', True)
        self.declare_parameter(
            'mujoco_initial_joint_positions',
            [0.0, -math.pi / 4.0, 0.0, -3.0 * math.pi / 4.0,
             0.0, math.pi / 2.0, math.pi / 4.0],
        )
        self.declare_parameter('mujoco_pos_stiffness', 600.0)
        self.declare_parameter('mujoco_rot_stiffness', 30.0)
        self.declare_parameter('mujoco_nullspace_stiffness', 10.0)
        self.declare_parameter('mujoco_torque_rate_limit', 1000.0)
        self.declare_parameter('autonomy_tank_initial_energy', 2.0)
        self.declare_parameter('autonomy_tank_max_energy', 5.0)
        self.declare_parameter('autonomy_tank_recharge_efficiency', 0.8)
        self.declare_parameter('optimal_lambda_s', 0.10)
        self.declare_parameter('autonomy_tank_power_epsilon', 1e-9)
        self.declare_parameter('autonomy_tank_velocity_epsilon', 1e-6)
        self.declare_parameter('autonomy_wrench_max_abs', 100.0)
        self.declare_parameter('optimal_lambda_c', 0.05)
        self.declare_parameter('nash_human_effort', 0.2)
        self.declare_parameter('nash_gp_effort', 0.5)
        self.declare_parameter('nash_human_agreement', 0.05)
        self.declare_parameter('nash_gp_agreement', 0.10)
        self.declare_parameter('nash_agreement_ratio', 0.7)
        self.declare_parameter('nash_rotation_weight', 1.0)
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
            tdpa_enabled=self.get_bool('tdpa_enabled'),
            tdpa_delay_sec=max(0.0, self.get_float('tdpa_delay_sec')),
            optimal_lambda_s=self.get_float('optimal_lambda_s'),
            optimal_lambda_c=self.get_float('optimal_lambda_c'),
            nash_human_effort=self.get_float('nash_human_effort'),
            nash_gp_effort=self.get_float('nash_gp_effort'),
            nash_human_agreement=self.get_float('nash_human_agreement'),
            nash_gp_agreement=self.get_float('nash_gp_agreement'),
            nash_agreement_ratio=self.get_float('nash_agreement_ratio'),
            nash_rotation_weight=self.get_float('nash_rotation_weight'),
            simulate_mujoco=self.get_bool('simulate_mujoco'),
            mujoco_model_path=self.get_string('mujoco_model_path'),
            mujoco_site_name=self.get_string('mujoco_site_name'),
            mujoco_timestep=self.get_float('mujoco_timestep'),
            mujoco_settle_time_sec=max(0.0, self.get_float('mujoco_settle_time_sec')),
            mujoco_initialize_from_leader=self.get_bool(
                'mujoco_initialize_from_leader'
            ),
            mujoco_pos_stiffness=self.get_float('mujoco_pos_stiffness'),
            mujoco_rot_stiffness=self.get_float('mujoco_rot_stiffness'),
            mujoco_nullspace_stiffness=self.get_float(
                'mujoco_nullspace_stiffness'
            ),
            mujoco_torque_rate_limit=self.get_float('mujoco_torque_rate_limit'),
            autonomy_tank_initial_energy=self.get_float('autonomy_tank_initial_energy'),
            autonomy_tank_max_energy=self.get_float('autonomy_tank_max_energy'),
            autonomy_tank_recharge_efficiency=self.get_float('autonomy_tank_recharge_efficiency'),
            autonomy_tank_power_epsilon=self.get_float('autonomy_tank_power_epsilon'),
            autonomy_tank_velocity_epsilon=self.get_float('autonomy_tank_velocity_epsilon'),
            autonomy_wrench_max_abs=self.get_float('autonomy_wrench_max_abs'),
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
        leader_delay_sec = max(0.0, self.get_float('leader_delay_sec'))
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
                config.fusion_policy,
            )
            if output_path.exists() and not overwrite:
                self.get_logger().info(f'Skipping existing output: {output_path}')
                continue

            count, min_alpha, max_alpha = self.fuse_trial(
                prediction_path,
                leader_path,
                output_path,
                config,
                metrics,
                leader_time_relative,
                leader_delay_sec,
                config.tdpa_delay_sec,
            )
            wrote += 1
            self.get_logger().info(
                f'Wrote {count} fused samples | alpha_G=[{min_alpha:.3f}, {max_alpha:.3f}] | '
                f'TDPA={config.tdpa_enabled}, delay={config.tdpa_delay_sec:.3f}s | '
                f'leader_delay_legacy={leader_delay_sec:.3f}s | metrics={metrics.source} | '
                f'skill={metrics.skill_name} | {output_path}'
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
        leader_delay_sec,
        tdpa_delay_sec,
    ):
        """Replay one trial using the online fuser timer and policy logic.

        When ``simulate_mujoco`` is enabled, physical end-effector state drives
        the energy-tank update. The optional settle interval holds the final
        fused target after the prediction ends.

        Args:
            prediction_path: Path to the saved prediction trajectory CSV.
            leader_path: Path to the saved leader execution trajectory CSV.
            output_path: Destination path for the fused CSV.
            config: Base fusion configuration.
            metrics: Reconstructed prediction confidence metrics.
            leader_time_relative: Whether to align leader time to its first
                sample before replay.
            leader_delay_sec: Legacy direct delay applied when TDPA replay is
                disabled.
            tdpa_delay_sec: One-way delay for both TDPA communication directions.

        Returns:
            Number of fused samples and alpha minimum/maximum, in that order.
        """
        prediction = read_trajectory(prediction_path)
        leader = read_trajectory(leader_path)
        prediction_poses = [sample.pose for sample in prediction]
        prediction_times = [sample.time for sample in prediction]
        leader_t0 = leader[0].time if leader_time_relative else 0.0
        simulator = None
        if config.simulate_mujoco:
            initial_pose = leader[0].pose if config.mujoco_initialize_from_leader else None
            simulator = MujocoAutonomyPassivitySimulator(
                config.mujoco_model_path,
                config.mujoco_site_name,
                config.mujoco_timestep,
                list(self.get_parameter('mujoco_initial_joint_positions').value),
                config,
                initial_pose=initial_pose,
            )
            self.get_logger().info(
                f'Using MuJoCo EE state for autonomy energy tank: {config.mujoco_model_path}'
            )
            if simulator.initialization_error is not None:
                position_error, orientation_error, converged = simulator.initialization_error
                message = (
                    'Initialized MuJoCo EE from the first leader pose | '
                    f'position_error={position_error:.6f} m | '
                    f'orientation_error={orientation_error:.6f} rad'
                )
                if converged:
                    self.get_logger().info(message)
                else:
                    self.get_logger().warn(f'{message} | IK did not fully converge')
        leader = [TimedPose(max(0.0, sample.time - leader_t0), sample.pose) for sample in leader]
        leader_times = [sample.time for sample in leader]
        leader_velocities = []
        for index, sample in enumerate(leader):
            previous = leader[max(0, index - 1)]
            following = leader[min(len(leader) - 1, index + 1)]
            dt = max(following.time - previous.time, 1e-9)
            linear = np.array((
                (following.pose.position.x - previous.pose.position.x) / dt,
                (following.pose.position.y - previous.pose.position.y) / dt,
                (following.pose.position.z - previous.pose.position.z) / dt,
            ))
            angular = rotation_error(
                pose_rotation(previous.pose),
                pose_rotation(following.pose),
            ) / dt
            leader_velocities.append(np.r_[linear, angular])
        tdpa = (
            OfflineCartesianTDPA(leader[0].pose, tdpa_delay_sec)
            if config.tdpa_enabled
            else None
        )

        # OnlineFuserNode starts every accepted prediction with a fresh alpha
        # filter. Trials must not leak authority state into one another.
        self._filtered_prediction_weight = 0.0
        self._previous_optimal_weight = 0.0
        self._last_fused_pose = None
        fused = []
        controlled = []
        alphas = []
        previous_elapsed = 0.0
        replay_duration = max(0.0, prediction_times[-1]) + config.mujoco_settle_time_sec
        for elapsed in self.replay_times(replay_duration, config.rate):
            predicted_pose = sample_timed_pose(prediction_poses, prediction_times, elapsed)
            if predicted_pose is None:
                continue
            if tdpa is None:
                leader_pose = self.latest_leader_pose(
                    leader,
                    elapsed - leader_delay_sec,
                    max(0.0, config.leader_timeout_sec),
                )
            else:
                leader_velocity = sample_timed_value(
                    leader_velocities,
                    leader_times,
                    elapsed,
                    np.zeros(6),
                )
                follower_wrench = (
                    simulator.last_command_wrench
                    if simulator is not None
                    else np.zeros(6)
                )
                leader_pose = tdpa.step(
                    elapsed,
                    leader_velocity,
                    follower_wrench,
                    elapsed - previous_elapsed,
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
                network_delay=max(
                    config.network_delay,
                    tdpa_delay_sec if tdpa is not None else leader_delay_sec,
                ),
            )
            fused_pose, alpha = self.fuse_pose_online(
                predicted_pose, leader_pose, sample_config
            )
            self._last_fused_pose = fused_pose
            tank_energy = passivity_lambda = passivity_power = wrench_norm = None
            controlled_pose = fused_pose
            if simulator is not None:
                physical_leader_pose = leader_pose if leader_pose is not None else fused_pose
                (controlled_pose, tank_energy, passivity_lambda, passivity_power,
                 wrench_norm) = simulator.advance(
                    physical_leader_pose,
                    fused_pose,
                    elapsed - previous_elapsed,
                )
            fused.append(TimedPose(
                elapsed,
                fused_pose,
                alpha,
                tank_energy=tank_energy,
                passivity_lambda=passivity_lambda,
                passivity_power=passivity_power,
                autonomy_wrench_norm=wrench_norm,
            ))
            if simulator is not None:
                controlled.append(TimedPose(
                    elapsed,
                    controlled_pose,
                    alpha,
                    tank_energy=tank_energy,
                    passivity_lambda=passivity_lambda,
                    passivity_power=passivity_power,
                    autonomy_wrench_norm=wrench_norm,
                ))
            alphas.append(alpha)
            previous_elapsed = elapsed

        write_trajectory(output_path, fused)
        if controlled:
            write_trajectory(controlled_output_path(output_path), controlled)
        return len(fused), min(alphas), max(alphas)


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
