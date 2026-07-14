"""ROS 2 node for online fusion of Geo-GP prediction and TDPA leader pose."""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64MultiArray
from rclpy.time import Time

from geo_gp_interfaces.msg import PredictedTrajectory, TDPACartesianState
from geo_gp_fusion.policies.weighted_blending import blend_pose, clamp, sample_timed_pose
from geo_gp_fusion.policies.optimal_blending import optimal_blend_pose, pose_conflict
from geo_gp_fusion.policies.nash_blending import nash_blend_pose


class OnlineFuserNode(Node):
    """
    Fuse predicted trajectories with the live TDPA-integrated leader pose.

    The node samples the active prediction by elapsed time, blends that pose with
    the freshest TDPA leader target pose, and publishes the result in the existing
    controller input format.
    """

    def __init__(self):
        """
        Create subscriptions, publishers, parameters, and the fusion timer.
        """
        super().__init__('online_fuser')

        self.declare_parameter('prediction_topic', '/gp_predicted_trajectory_raw')
        self.declare_parameter('tdpa_pose_topic', '/tdpa/integrated_desired_pose')
        self.declare_parameter('network_state_topic', '/leader/tdpa_cartesian_state_delayed')
        self.declare_parameter('output_pose_topic', '/execution/desired_pose')
        self.declare_parameter('running_topic', '/execution/running')
        self.declare_parameter('rate', 200.0)
        self.declare_parameter('leader_timeout_sec', 0.1)
        self.declare_parameter('confidence_gain', 1.0)
        self.declare_parameter('fusion_policy', 'weighted_blending')
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
        self.declare_parameter('optimal_lambda_s', 0.10)
        self.declare_parameter('optimal_lambda_c', 0.05)
        self.declare_parameter('nash_human_effort', 0.2)
        self.declare_parameter('nash_gp_effort', 0.5)
        self.declare_parameter('nash_human_agreement', 0.05)
        self.declare_parameter('nash_gp_agreement', 0.10)
        self.declare_parameter('nash_agreement_ratio', 0.7)
        self.declare_parameter('nash_rotation_weight', 1.0)
        self.declare_parameter('progressive_update_enabled', True)
        self.declare_parameter('progressive_match_pos_eps', 1e-3)
        self.declare_parameter('progressive_match_time_eps', 1e-3)

        self.prediction_topic = self.get_parameter('prediction_topic').value
        self.tdpa_pose_topic = self.get_parameter('tdpa_pose_topic').value
        self.network_state_topic = self.get_parameter('network_state_topic').value
        self.output_pose_topic = self.get_parameter('output_pose_topic').value
        self.running_topic = self.get_parameter('running_topic').value
        self.rate = float(self.get_parameter('rate').value)
        self.leader_timeout_sec = float(self.get_parameter('leader_timeout_sec').value)
        self.confidence_gain = float(self.get_parameter('confidence_gain').value)
        self.fusion_policy = self._normalize_fusion_policy(self.get_parameter('fusion_policy').value)
        self.min_prediction_weight = float(self.get_parameter('min_prediction_weight').value)
        self.max_prediction_weight = float(self.get_parameter('max_prediction_weight').value)
        self.network_k_delay = float(self.get_parameter('network_k_delay').value)
        self.network_delay_max = float(self.get_parameter('network_delay_max').value)
        self.network_k_jitter = float(self.get_parameter('network_k_jitter').value)
        self.network_jitter_max = float(self.get_parameter('network_jitter_max').value)
        self.network_w_delay = float(self.get_parameter('network_w_delay').value)
        self.network_w_jitter = float(self.get_parameter('network_w_jitter').value)
        self.network_gamma = float(self.get_parameter('network_gamma').value)
        self.gp_skill_min = float(self.get_parameter('gp_skill_min').value)
        self.gp_k_sigma = float(self.get_parameter('gp_k_sigma').value)
        self.gp_k_chunk = float(self.get_parameter('gp_k_chunk').value)
        self.gp_error_fail = float(self.get_parameter('gp_error_fail').value)
        self.gp_k_progress = float(self.get_parameter('gp_k_progress').value)
        self.gp_progress_midpoint = float(self.get_parameter('gp_progress_midpoint').value)
        self.gp_w_sigma = float(self.get_parameter('gp_w_sigma').value)
        self.gp_w_chunk = float(self.get_parameter('gp_w_chunk').value)
        self.gp_w_progress = float(self.get_parameter('gp_w_progress').value)
        self.gp_gamma = float(self.get_parameter('gp_gamma').value)
        self.authority_eps = float(self.get_parameter('authority_eps').value)
        self.optimal_lambda_s = float(self.get_parameter('optimal_lambda_s').value)
        self.optimal_lambda_c = float(self.get_parameter('optimal_lambda_c').value)
        self.nash_human_effort = float(self.get_parameter('nash_human_effort').value)
        self.nash_gp_effort = float(self.get_parameter('nash_gp_effort').value)
        self.nash_human_agreement = float(self.get_parameter('nash_human_agreement').value)
        self.nash_gp_agreement = float(self.get_parameter('nash_gp_agreement').value)
        self.nash_agreement_ratio = float(self.get_parameter('nash_agreement_ratio').value)
        self.nash_rotation_weight = float(self.get_parameter('nash_rotation_weight').value)
        self.progressive_update_enabled = bool(
            self.get_parameter('progressive_update_enabled').value
        )
        self.progressive_match_pos_eps = float(
            self.get_parameter('progressive_match_pos_eps').value
        )
        self.progressive_match_time_eps = float(
            self.get_parameter('progressive_match_time_eps').value
        )

        self._lock = threading.Lock()
        self._latest_leader_pose = None
        self._latest_leader_stamp = None
        self._latest_network_delay = None
        self._latest_network_jitter = 0.0
        self._previous_network_delay = None
        self._network_state_received = False
        self._active_prediction = None
        self._prediction_start_time = None
        self._running = False
        self._last_fused_pose = None
        self._previous_optimal_weight = 0.0

        reliable_latest = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        best_effort_latest = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        running_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.prediction_sub = self.create_subscription(
            PredictedTrajectory,
            self.prediction_topic,
            self.prediction_callback,
            reliable_latest,
        )
        self.leader_pose_sub = self.create_subscription(
            PoseStamped,
            self.tdpa_pose_topic,
            self.leader_pose_callback,
            best_effort_latest,
        )
        self.network_state_sub = self.create_subscription(
            TDPACartesianState,
            self.network_state_topic,
            self.network_state_callback,
            best_effort_latest,
        )
        self.pose_pub = self.create_publisher(
            Float64MultiArray,
            self.output_pose_topic,
            10,
        )
        self.running_pub = self.create_publisher(Bool, self.running_topic, running_qos)

        period = 1.0 / self.rate if self.rate > 0.0 else 0.005
        self.timer = self.create_timer(period, self.timer_callback)

        self.publish_running(False)
        self.get_logger().info(
            'Online fuser started | '
            f'prediction={self.prediction_topic} | tdpa_pose={self.tdpa_pose_topic} | '
            f'network_state={self.network_state_topic} | output={self.output_pose_topic} | '
            f'running={self.running_topic} | fusion_policy={self.fusion_policy}'
        )

    def _normalize_fusion_policy(self, value):
        """Normalize the configured fusion policy name."""
        aliases = {
            'weighted': 'weighted_blending', 'weighted_blending': 'weighted_blending',
            'optimal': 'optimal_blending', 'optimal_blending': 'optimal_blending',
            'nash': 'nash_blending', 'nash_blending': 'nash_blending',
        }
        policy = aliases.get(str(value).strip().lower())
        if policy is None:
            self.get_logger().warn(f'Unknown fusion_policy={value!r}; falling back to weighted_blending')
            return 'weighted_blending'
        return policy

    def leader_pose_callback(self, msg):
        """
        Store the latest TDPA-integrated leader target pose.

        Args:
            msg: Pose stamped by the TDPA controller publisher.
        """
        now = self.get_clock().now()
        delay = self.estimate_network_delay_from_stamp(now, msg.header.stamp)
        with self._lock:
            self._latest_leader_pose = msg.pose
            self._latest_leader_stamp = now
            if delay is not None and not self._network_state_received:
                self._latest_network_delay = delay
                if self._previous_network_delay is not None:
                    self._latest_network_jitter = abs(delay - self._previous_network_delay)
                self._previous_network_delay = delay

    def network_state_callback(self, msg):
        """
        Update confidence inputs from measured TDPA round-trip delay.
        
        Args:
            msg: TDPACartesianState message with echo_tx_time_ns and tx_time_ns    
        """
        now = self.get_clock().now()
        delay = self.estimate_network_rtt_from_echo(now, msg.echo_tx_time_ns)
        if delay is None:
            delay = self.estimate_network_delay_from_tx_time(now, msg.tx_time_ns)
        if delay is None:
            delay = self.estimate_network_delay_from_stamp(now, msg.header.stamp)
        if delay is None:
            return
        with self._lock:
            self._network_state_received = True
            self._latest_network_delay = delay
            if self._previous_network_delay is not None:
                self._latest_network_jitter = abs(delay - self._previous_network_delay)
            self._previous_network_delay = delay

    def prediction_callback(self, msg):
        """
        Accept a successful prediction as the active online fusion segment.

        Args:
            msg: Predicted trajectory from the Geo-GP prediction node.
        """
        if not msg.success:
            self.get_logger().warn(
                f'Dropping unsuccessful prediction | skill={msg.skill_name}'
            )
            return
        if not msg.poses:
            self.get_logger().warn('Dropping empty prediction')
            return

        times = list(msg.time_from_start)
        if len(times) != len(msg.poses):
            dt = 1.0 / self.rate if self.rate > 0.0 else 0.005
            times = [i * dt for i in range(len(msg.poses))]
            self.get_logger().warn(
                f'Prediction time_from_start size mismatch, using fixed dt={dt:.6f}s'
            )

        next_prediction = {
            'skill_name': msg.skill_name,
            'poses': list(msg.poses),
            'times': times,
            'confidence': float(msg.confidence),
            'skill_confidence': float(msg.skill_confidence),
            'trajectory_variance': float(msg.variance_mean),
            'point_variances': list(msg.variance_means),
            'chunk_error': float(msg.chunk_error),
            'progress': float(msg.progress),
        }

        with self._lock:
            extend_in_place = (
                self.progressive_update_enabled
                and self.can_extend_prediction_in_place(next_prediction)
            )
            self._active_prediction = next_prediction
            if extend_in_place:
                self.get_logger().info(
                    f'Extended online fusion trajectory in-place | poses={len(msg.poses)}'
                )
                return

            self._prediction_start_time = self.get_clock().now()
            self._running = True
            self._previous_optimal_weight = 0.0

        self.publish_running(True)
        self.get_logger().info(
            f'Started online fusion | skill={msg.skill_name} | poses={len(msg.poses)} | '
            f'confidence={msg.confidence:.3f}'
        )

    def can_extend_prediction_in_place(self, next_prediction):
        """
        Check whether a new prediction extends the active one without restart.

        Args:
            next_prediction: Normalized prediction dict with poses and times.

        Returns:
            True when the new trajectory is a longer prefix-compatible update.
        """
        current = self._active_prediction
        if not self._running or current is None or self._prediction_start_time is None:
            return False

        current_poses = current['poses']
        current_times = current['times']
        next_poses = next_prediction['poses']
        next_times = next_prediction['times']

        if not current_poses or not current_times:
            return False
        if len(next_poses) <= len(current_poses):
            return False
        if len(current_times) != len(current_poses) or len(next_times) != len(next_poses):
            return False

        for i, current_pose in enumerate(current_poses):
            next_pose = next_poses[i]
            dx = current_pose.position.x - next_pose.position.x
            dy = current_pose.position.y - next_pose.position.y
            dz = current_pose.position.z - next_pose.position.z
            pos_err = math.sqrt(dx * dx + dy * dy + dz * dz)
            if pos_err > self.progressive_match_pos_eps:
                return False
            if abs(current_times[i] - next_times[i]) > self.progressive_match_time_eps:
                return False

        return True

    def timer_callback(self):
        """
        Publish one fused desired pose for the current timer tick.
        """
        with self._lock:
            pred = self._active_prediction
            start_time = self._prediction_start_time
            leader_pose = self._latest_leader_pose
            leader_stamp = self._latest_leader_stamp
            network_delay = self._latest_network_delay
            network_jitter = self._latest_network_jitter

        if pred is None or start_time is None:
            return

        now = self.get_clock().now()
        elapsed = max(0.0, (now - start_time).nanoseconds * 1e-9)
        times = pred['times']
        if times and elapsed > times[-1]:
            final_pose = sample_timed_pose(pred['poses'], times, times[-1])
            self.publish_pose(final_pose)
            self.finish_prediction(pred['skill_name'], elapsed)
            return

        predicted_pose = sample_timed_pose(pred['poses'], times, elapsed)
        if predicted_pose is None:
            self.finish_prediction(pred['skill_name'], elapsed)
            return

        leader_is_fresh = (
            leader_pose is not None
            and leader_stamp is not None
            and (now - leader_stamp).nanoseconds * 1e-9 <= self.leader_timeout_sec
        )
        if not leader_is_fresh:
            leader_pose = None
            self.get_logger().warn(
                'TDPA leader pose unavailable or stale, using prediction only',
                throttle_duration_sec=1.0,
            )

        point_variance = self.sample_timed_value(
            pred.get('point_variances', []),
            times,
            elapsed,
            pred.get('trajectory_variance', 0.0),
        )
        fused_pose = self.fuse_pose(
            predicted_pose, leader_pose, pred, network_delay, network_jitter,
            point_variance,
        )
        self._last_fused_pose = fused_pose
        self.publish_pose(fused_pose)


    def fuse_pose(
        self, predicted_pose, leader_pose, pred, network_delay, network_jitter,
        point_variance,
    ):
        """Fuse targets using the configured policy."""
        if self.fusion_policy == 'weighted_blending':
            return blend_pose(
                predicted_pose, leader_pose,
                self.prediction_weight(pred, network_delay, network_jitter, point_variance),
            )

        c_net = self.compute_network_confidence(network_delay, network_jitter)
        c_gp = self.compute_gp_confidence(pred, point_variance)
        g_skill = 1.0 if c_gp > 0.0 else 0.0
        if self.fusion_policy == 'optimal_blending':
            conflict = pose_conflict(leader_pose, predicted_pose) if leader_pose else 0.0
            fused_pose, result = optimal_blend_pose(
                predicted_pose, leader_pose, c_net=c_net, c_gp=c_gp,
                alpha_prev=self._previous_optimal_weight, d=conflict,
                lambda_s=self.optimal_lambda_s, lambda_c=self.optimal_lambda_c,
                g_skill=g_skill, confidence_gain=self.confidence_gain,
                min_prediction_weight=self.min_prediction_weight,
                max_prediction_weight=self.max_prediction_weight,
                authority_eps=self.authority_eps,
            )
            self._previous_optimal_weight = result.alpha_g
            return fused_pose

        current_pose = self._last_fused_pose or leader_pose or predicted_pose
        fused_pose, _ = nash_blend_pose(
            current_pose, predicted_pose, leader_pose, c_net=c_net, c_gp=c_gp,
            g_skill=g_skill, confidence_gain=self.confidence_gain,
            human_effort=self.nash_human_effort, gp_effort=self.nash_gp_effort,
            human_agreement=self.nash_human_agreement,
            gp_agreement=self.nash_gp_agreement,
            agreement_ratio=self.nash_agreement_ratio,
            rotation_weight=self.nash_rotation_weight, eps=self.authority_eps,
        )
        return fused_pose

    def sample_timed_value(self, values, times, elapsed, default=0.0):
        """
        Sample a value from a time-indexed list of values.

        Args:
            values: List of values corresponding to the times.
            times: List of time_from_start values for each value.
            elapsed: Elapsed time in seconds since the prediction started.
            default: Default value to return if sampling fails.

        Returns:
            The sampled value at the given elapsed time, or the default if unavailable.
        """
        if not values:
            return default
        if len(values) == 1 or not times:
            return values[0]
        n = min(len(values), len(times))
        if n <= 0:
            return default
        if elapsed <= times[0]:
            return values[0]
        if elapsed >= times[n - 1]:
            return values[n - 1]

        hi = 1
        while hi < n and times[hi] < elapsed:
            hi += 1
        lo = max(0, hi - 1)
        if hi >= n:
            return values[n - 1]
        return values[lo] if elapsed - times[lo] <= times[hi] - elapsed else values[hi]

    def estimate_network_delay_from_tx_time(self, now, tx_time_ns):
        """
        Estimate the one-way network delay from the leader's transmit timestamp.

        Args:
            now: Current ROS time.
            tx_time_ns: Transmit time in nanoseconds from the leader.
        
        Returns:
            Estimated one-way delay in seconds, or None if invalid.
        """
        if tx_time_ns <= 0:
            return None
        delay = (now.nanoseconds - int(tx_time_ns)) * 1e-9
        if not math.isfinite(delay):
            return None
        return max(0.0, delay)

    def estimate_network_rtt_from_echo(self, now, echo_tx_time_ns):
        """
        Estimate the round-trip time from the echo timestamp.
        
        Args:
            now: Current ROS time.
            echo_tx_time_ns: Echoed transmit time in nanoseconds from the leader.
            
        Returns:
            Estimated round-trip time in seconds, or None if invalid.
        """
        if echo_tx_time_ns <= 0:
            return None
        rtt = (now.nanoseconds - int(echo_tx_time_ns)) * 1e-9
        if not math.isfinite(rtt):
            return None
        return max(0.0, rtt)

    def estimate_network_delay_from_stamp(self, now, stamp_msg):
        """
        Estimate the one-way network delay from a ROS time stamp.

        Args:
            now: Current ROS time.
            stamp_msg: ROS time stamp message from the leader.

        Returns:
            Estimated one-way delay in seconds, or None if invalid.
        """
        stamp = Time.from_msg(stamp_msg)
        if stamp.nanoseconds <= 0:
            return None
        delay = (now - stamp).nanoseconds * 1e-9
        if not math.isfinite(delay):
            return None
        return max(0.0, delay)

    def normalized_weights(self, *weights):
        """
        Normalize a list of weights to sum to 1.0, with non-negative values.

        Args:
            *weights: Variable number of weight values.

        Returns:
            List of normalized weights that sum to 1.0.
        """
        clean = [max(0.0, float(w)) if math.isfinite(float(w)) else 0.0 for w in weights]
        total = sum(clean)
        if total <= 1e-12:
            return [1.0 / len(clean)] * len(clean)
        return [w / total for w in clean]

    def compute_network_confidence(self, delay, jitter):
        """
        Compute network confidence from delay and jitter metrics.
        
        Args:
            delay: Latest leader/follower RTT estimate in seconds.
            jitter: Latest RTT jitter estimate in seconds.
            
        Returns:
            Network confidence in [0, 1], where 1 is best.
        """
        d_max = max(self.network_delay_max, 1e-9)
        j_max = max(self.network_jitter_max, 1e-9)
        D = max(0.0, float(delay)) if delay is not None and math.isfinite(delay) else d_max
        J = max(0.0, float(jitter)) if jitter is not None and math.isfinite(jitter) else j_max
        c_d = math.exp(-self.network_k_delay * D / d_max)
        c_j = math.exp(-self.network_k_jitter * J / j_max)
        w_d, w_j = self.normalized_weights(self.network_w_delay, self.network_w_jitter)
        return (c_d ** w_d * c_j ** w_j) ** max(self.network_gamma, 0.0)

    def compute_gp_confidence(self, pred, point_variance=None):
        """
        Compute GP confidence from prediction metrics.

        Args:
            pred: Active prediction dict with confidence metrics.
            point_variance: Conservative variance for the currently sampled prediction point.

        Returns:
            GP confidence in [0, 1], where 1 is best.
        """
        c_skill = pred.get('skill_confidence', pred.get('confidence', 0.0))
        if not math.isfinite(c_skill):
            c_skill = 0.0
        c_skill = clamp(c_skill, 0.0, 1.0)
        g_skill = 1.0 if c_skill >= self.gp_skill_min else 0.0

        if point_variance is None:
            point_variance = pred.get('trajectory_variance', 0.0)
        point_variance = max(0.0, float(point_variance))
        chunk_error = max(0.0, float(pred.get('chunk_error', 0.0)))
        progress = float(pred.get('progress', 0.0))
        if not math.isfinite(progress):
            progress = 0.0

        c_var = math.exp(-self.gp_k_sigma * point_variance)
        c_chunk = math.exp(
            -self.gp_k_chunk * chunk_error / max(self.gp_error_fail, 1e-9)
        )
        c_prog = 1.0 / (
            1.0 + math.exp(-self.gp_k_progress * (progress - self.gp_progress_midpoint))
        )
        w_sigma, w_c, w_rho = self.normalized_weights(
            self.gp_w_sigma,
            self.gp_w_chunk,
            self.gp_w_progress,
        )
        return g_skill * (
            c_var ** w_sigma * c_chunk ** w_c * c_prog ** w_rho
        ) ** max(self.gp_gamma, 0.0)

    def prediction_weight(self, pred, network_delay, network_jitter, point_variance=None):
        """
        Compute GP authority alpha_G from network and GP confidence.

        Args:
            pred: Active prediction dict with confidence metrics.
            network_delay: Latest leader/follower RTT estimate in seconds.
            network_jitter: Latest RTT jitter estimate in seconds.
            point_variance: Conservative variance for the currently sampled prediction point.

        Returns:
            Prediction weight clipped by configured min/max bounds.
        """
        c_net = self.compute_network_confidence(network_delay, network_jitter)
        c_gp = self.compute_gp_confidence(pred, point_variance) * self.confidence_gain
        weight = c_gp / (c_net + c_gp + max(self.authority_eps, 1e-12))
        return clamp(weight, self.min_prediction_weight, self.max_prediction_weight)

    def publish_pose(self, pose):
        """
        Publish a pose in the existing controller Float64MultiArray format.

        Args:
            pose: Fused desired pose to publish.
        """
        if pose is None:
            return
        msg = Float64MultiArray()
        msg.data = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        self.pose_pub.publish(msg)

    def finish_prediction(self, skill_name, elapsed):
        """
        Clear the active prediction and publish execution stopped.

        Args:
            skill_name: Name carried by the prediction message.
            elapsed: Seconds spent executing the fusion segment.
        """
        with self._lock:
            self._active_prediction = None
            self._prediction_start_time = None
            self._running = False
        self.publish_running(False)
        self.get_logger().info(
            f'Online fusion finished | skill={skill_name} | elapsed={elapsed:.3f}s'
        )

    def publish_running(self, running):
        """
        Publish the execution running latch.

        Args:
            running: Whether the fuser is actively publishing an execution segment.
        """
        msg = Bool()
        msg.data = bool(running)
        self.running_pub.publish(msg)


def main(args=None):
    """
    Run the online fuser node.

    Args:
        args: Optional ROS command-line arguments.
    """
    rclpy.init(args=args)
    node = OnlineFuserNode()
    try:
        rclpy.spin(node)
    finally:
        node.publish_running(False)
        node.destroy_node()
        rclpy.shutdown()
