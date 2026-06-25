"""ROS 2 node for online fusion of Geo-GP prediction and TDPA leader pose."""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64MultiArray

from geo_gp_interfaces.msg import PredictedTrajectory
from geo_gp_fusion.policies.linear_blend import blend_pose, clamp, sample_timed_pose


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
        self.declare_parameter('output_pose_topic', '/execution/desired_pose')
        self.declare_parameter('running_topic', '/execution/running')
        self.declare_parameter('rate', 200.0)
        self.declare_parameter('leader_timeout_sec', 0.1)
        self.declare_parameter('confidence_gain', 1.0)
        self.declare_parameter('min_prediction_weight', 0.0)
        self.declare_parameter('max_prediction_weight', 1.0)

        self.prediction_topic = self.get_parameter('prediction_topic').value
        self.tdpa_pose_topic = self.get_parameter('tdpa_pose_topic').value
        self.output_pose_topic = self.get_parameter('output_pose_topic').value
        self.running_topic = self.get_parameter('running_topic').value
        self.rate = float(self.get_parameter('rate').value)
        self.leader_timeout_sec = float(self.get_parameter('leader_timeout_sec').value)
        self.confidence_gain = float(self.get_parameter('confidence_gain').value)
        self.min_prediction_weight = float(self.get_parameter('min_prediction_weight').value)
        self.max_prediction_weight = float(self.get_parameter('max_prediction_weight').value)

        self._lock = threading.Lock()
        self._latest_leader_pose = None
        self._latest_leader_stamp = None
        self._active_prediction = None
        self._prediction_start_time = None
        self._running = False

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
            f'output={self.output_pose_topic} | running={self.running_topic}'
        )

    def leader_pose_callback(self, msg):
        """
        Store the latest TDPA-integrated leader target pose.

        Args:
            msg: Pose stamped by the TDPA controller publisher.
        """
        with self._lock:
            self._latest_leader_pose = msg.pose
            self._latest_leader_stamp = self.get_clock().now()

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

        with self._lock:
            self._active_prediction = {
                'skill_name': msg.skill_name,
                'poses': list(msg.poses),
                'times': times,
                'confidence': float(msg.confidence),
            }
            self._prediction_start_time = self.get_clock().now()
            self._running = True

        self.publish_running(True)
        self.get_logger().info(
            f'Started online fusion | skill={msg.skill_name} | poses={len(msg.poses)} | '
            f'confidence={msg.confidence:.3f}'
        )

    def timer_callback(self):
        """
        Publish one fused desired pose for the current timer tick.
        """
        with self._lock:
            pred = self._active_prediction
            start_time = self._prediction_start_time
            leader_pose = self._latest_leader_pose
            leader_stamp = self._latest_leader_stamp

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

        prediction_weight = self.prediction_weight(pred['confidence'])
        fused_pose = blend_pose(predicted_pose, leader_pose, prediction_weight)
        self.publish_pose(fused_pose)

    def prediction_weight(self, confidence):
        """
        Convert Geo-GP confidence into the prediction-side blend weight.

        Args:
            confidence: Confidence value from the prediction message.

        Returns:
            Prediction weight clipped by configured min/max bounds.
        """
        if not math.isfinite(confidence):
            confidence = 0.0
        weight = confidence * self.confidence_gain
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
