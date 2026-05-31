import os
from datetime import datetime

import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geo_gp_interfaces.msg import PromptTrajectory, PredictedTrajectory
from .predictor import (
    Predictor,
    save_predicted_trajectory_to_csv,
    save_prompt_trajectory_to_csv,
    save_similarity_transform_to_json,
)


class PredictionNode(Node):
    def __init__(self):
        super().__init__('prediction_node')
        self.declare_parameter("config_path", "")
        self.declare_parameter("model_dir", "")
        self.declare_parameter("input_topic", "/gp_prompt_trajectory")
        self.declare_parameter("output_topic", "/gp_predicted_trajectory")
        self.declare_parameter("execution_running_topic", "/execution/running")
        self.declare_parameter("enabled_topic", "/geo_gp/enabled")
        self.declare_parameter("force_enabled_topic", "/geo_gp/force_prediction_enabled")
        self.declare_parameter("progressive_publish", True)
        self.declare_parameter("progressive_rollout_horizon", 15)
        self.declare_parameter("progressive_rollout_step", 15)
        self.declare_parameter("progressive_min_points", 10)
        self.declare_parameter("save_csv", True)
        self.declare_parameter("csv_output_dir", "")

        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        model_dir = self.get_parameter("model_dir").get_parameter_value().string_value
        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        execution_running_topic = self.get_parameter(
            "execution_running_topic"
        ).get_parameter_value().string_value
        enabled_topic = self.get_parameter(
            "enabled_topic"
        ).get_parameter_value().string_value
        force_enabled_topic = self.get_parameter(
            "force_enabled_topic"
        ).get_parameter_value().string_value
        self._progressive_publish = self.get_parameter(
            "progressive_publish"
        ).get_parameter_value().bool_value
        self._progressive_rollout_horizon = max(
            1,
            int(
                self.get_parameter("progressive_rollout_horizon")
                .get_parameter_value()
                .integer_value
            ),
        )
        self._progressive_rollout_step = max(
            1,
            int(
                self.get_parameter("progressive_rollout_step")
                .get_parameter_value()
                .integer_value
            ),
        )
        self._progressive_min_points = max(
            2,
            int(
                self.get_parameter("progressive_min_points")
                .get_parameter_value()
                .integer_value
            ),
        )
        self._save_csv = self.get_parameter("save_csv").get_parameter_value().bool_value
        self._csv_output_dir = self.get_parameter("csv_output_dir").get_parameter_value().string_value
        self.predictor = Predictor(self.get_logger(), config_path, model_dir)
        self.qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._lock = threading.Lock()
        self._wake_event = threading.Event()
        self._shutdown = False
        self._latest_prompt = None
        self._latest_seq = 0
        self._execution_running = False
        self._enabled = False
        self._force_enabled = False

        # Subscriber
        self.prompt_sub = self.create_subscription(
            PromptTrajectory,
            input_topic,
            self.prompt_callback,
            self.qos_profile
        )
        self.execution_running_sub = self.create_subscription(
            Bool,
            execution_running_topic,
            self.execution_running_callback,
            self.state_qos,
        )
        self.enabled_sub = self.create_subscription(
            Bool,
            enabled_topic,
            self.enabled_callback,
            self.state_qos,
        )
        self.force_enabled_sub = self.create_subscription(
            Bool,
            force_enabled_topic,
            self.force_enabled_callback,
            self.state_qos,
        )

        # Publisher
        self.pred_pub = self.create_publisher(
            PredictedTrajectory,
            output_topic,
            self.qos_profile
        )
        self._worker = threading.Thread(target=self._prediction_worker, daemon=True)
        self._worker.start()

        self.get_logger().info(
            "Geo GP Prediction Node Started | "
            f"input={input_topic} | output={output_topic} | enabled_topic={enabled_topic} | "
            f"force_enabled_topic={force_enabled_topic}"
        )

    def prompt_callback(self, msg: PromptTrajectory):
        n = len(msg.poses)
        with self._lock:
            if not self._enabled:
                self.get_logger().info(
                    f"Dropping prompt trajectory with {n} poses because Geo-GP is disabled"
                )
                return
            if self._execution_running:
                self.get_logger().info(
                    f"Dropping prompt trajectory with {n} poses because execution is running"
                )
                return
            self._latest_seq += 1
            seq = self._latest_seq
            self._latest_prompt = (seq, msg)

        self.get_logger().info(f"Queued prompt trajectory seq={seq} with {n} poses")
        self._wake_event.set()

    def execution_running_callback(self, msg: Bool):
        with self._lock:
            self._execution_running = msg.data
            if msg.data:
                self._latest_prompt = None

    def enabled_callback(self, msg: Bool):
        with self._lock:
            self._enabled = msg.data
            if not self._enabled:
                self._latest_prompt = None

        self.get_logger().info(f"Geo-GP prediction {'ENABLED' if msg.data else 'DISABLED'}")

    def force_enabled_callback(self, msg: Bool):
        with self._lock:
            self._force_enabled = msg.data

        self.get_logger().info(
            f"Geo-GP force prediction {'ENABLED' if msg.data else 'DISABLED'}"
        )

    def _make_output_paths(self, success: bool) -> dict:
        output_dir = self._csv_output_dir or os.getcwd()
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        status = "success" if success else "failed"
        return {
            "prediction": os.path.join(
                output_dir, f"prediction_{status}_{timestamp}.csv"
            ),
            "prompt": os.path.join(output_dir, f"prompt_{status}_{timestamp}.csv"),
            "similarity_transform": os.path.join(
                output_dir, f"similarity_transform_{status}_{timestamp}.json"
            ),
        }

    def _save_prediction_artifacts(
        self,
        paths: dict,
        pred: PredictedTrajectory,
        prompt_msg: PromptTrajectory,
        seq: int,
    ) -> None:
        if not self._save_csv:
            return

        ctx = self.predictor.last_prediction_context
        try:
            if len(pred.poses) > 0:
                save_predicted_trajectory_to_csv(paths["prediction"], pred)
            save_prompt_trajectory_to_csv(paths["prompt"], prompt_msg)
            if ctx is not None and ctx.get("ok"):
                save_similarity_transform_to_json(paths["similarity_transform"], ctx)
            self.get_logger().info(
                "Saved prediction artifacts | "
                f"seq={seq} | success={pred.success} | poses={len(pred.poses)} | "
                f"prediction={paths['prediction']} | prompt={paths['prompt']}"
                + (
                    f" | transform={paths['similarity_transform']}"
                    if ctx is not None and ctx.get("ok")
                    else ""
                )
            )
        except OSError as exc:
            self.get_logger().error(f"Failed to save prediction artifacts: {exc}")

    def _prediction_worker(self):
        while True:
            self._wake_event.wait()
            self._wake_event.clear()

            while True:
                with self._lock:
                    if self._shutdown:
                        return
                    if self._latest_prompt is None:
                        break
                    if not self._enabled:
                        self._latest_prompt = None
                        break
                    if self._execution_running:
                        self._latest_prompt = None
                        break

                    seq, msg = self._latest_prompt
                    force_enabled = self._force_enabled
                    self._latest_prompt = None

                self.get_logger().info(
                    f"Starting prediction for latest prompt seq={seq} with {len(msg.poses)} poses "
                    f"and force_enabled={force_enabled}"
                )
                if self._progressive_publish:
                    published_any = False
                    last_pred = None
                    for idx, total_chunks, pred_chunk in self.predictor.iter_progressive_predictions(
                        msg,
                        predict_force=force_enabled,
                        first_chunk_horizon=self._progressive_rollout_horizon,
                        rollout_step=self._progressive_rollout_step,
                    ):
                        last_pred = pred_chunk
                        with self._lock:
                            enabled = self._enabled
                            execution_running = self._execution_running
                        if not enabled:
                            self.get_logger().info(
                                f"Skipping progressive chunk publish for seq={seq} because Geo-GP is disabled"
                            )
                            break
                        if execution_running and not published_any:
                            self.get_logger().info(
                                f"Skipping progressive chunk publish for seq={seq} because execution started early"
                            )
                            break
                        if not pred_chunk.success:
                            self.get_logger().info(
                                f"Skipped progressive chunk {idx}/{total_chunks} | seq={seq} | unsuccessful prediction"
                            )
                            continue
                        if len(pred_chunk.poses) < self._progressive_min_points and not published_any:
                            self.get_logger().info(
                                f"Skipped progressive chunk {idx}/{total_chunks} | seq={seq} | "
                                f"poses={len(pred_chunk.poses)} below min_points={self._progressive_min_points}"
                            )
                            continue
                        self.pred_pub.publish(pred_chunk)
                        published_any = True
                        self.get_logger().info(
                            f"Published progressive chunk {idx}/{total_chunks} | seq={seq} "
                            f"| poses={len(pred_chunk.poses)}"
                        )
                    if self._save_csv:
                        pred_to_save = last_pred
                        if pred_to_save is None:
                            pred_to_save = self.predictor.predict(
                                msg, predict_force=force_enabled
                            )
                        self._save_prediction_artifacts(
                            self._make_output_paths(pred_to_save.success),
                            pred_to_save,
                            msg,
                            seq,
                        )
                    continue

                pred = self.predictor.predict(msg, predict_force=force_enabled)
                with self._lock:
                    enabled = self._enabled
                    execution_running = self._execution_running

                if not enabled:
                    self.get_logger().info(
                        f"Skipping predicted trajectory publish for seq={seq} because Geo-GP is disabled"
                    )
                    continue
                if execution_running:
                    self.get_logger().info(
                        f"Skipping predicted trajectory publish for seq={seq} because execution is running"
                    )
                    continue

                self.pred_pub.publish(pred)
                if self._save_csv:
                    self._save_prediction_artifacts(
                        self._make_output_paths(pred.success), pred, msg, seq
                    )
                self.get_logger().info(
                    f"Published predicted trajectory | seq={seq} | success={pred.success} | "
                    f"skill={pred.skill_name} | confidence={pred.confidence}"
                )

    def destroy_node(self):
        with self._lock:
            self._shutdown = True
        self._wake_event.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = PredictionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
