import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geo_gp_interfaces.msg import PromptTrajectory, PredictedTrajectory
from .predictor import Predictor


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
        self._worker_busy = False
        self._last_published_seq = 0
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

    def _prediction_worker(self):
        while True:
            self._wake_event.wait()
            self._wake_event.clear()

            while True:
                with self._lock:
                    if self._shutdown:
                        return
                    if self._latest_prompt is None:
                        self._worker_busy = False
                        break
                    if not self._enabled:
                        self._latest_prompt = None
                        self._worker_busy = False
                        break
                    if self._execution_running:
                        self._latest_prompt = None
                        self._worker_busy = False
                        break

                    self._worker_busy = True
                    seq, msg = self._latest_prompt
                    force_enabled = self._force_enabled
                    self._latest_prompt = None

                self.get_logger().info(
                    f"Starting prediction for latest prompt seq={seq} with {len(msg.poses)} poses "
                    f"and force_enabled={force_enabled}"
                )
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
                self._last_published_seq = seq
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
