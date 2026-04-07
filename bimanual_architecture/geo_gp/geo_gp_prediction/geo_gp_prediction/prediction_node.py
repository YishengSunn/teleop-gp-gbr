import rclpy
from rclpy.node import Node

from geo_gp_interfaces.msg import PromptTrajectory, PredictedTrajectory
from .predictor import Predictor


class PredictionNode(Node):
    def __init__(self):
        super().__init__('prediction_node')
        self.declare_parameter("config_path", "")
        self.declare_parameter("model_dir", "")

        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        model_dir = self.get_parameter("model_dir").get_parameter_value().string_value
        self.predictor = Predictor(self.get_logger(), config_path, model_dir)

        # Subscriber
        self.prompt_sub = self.create_subscription(
            PromptTrajectory,
            '/gp_prompt_trajectory',
            self.prompt_callback,
            10
        )

        # Publisher
        self.pred_pub = self.create_publisher(
            PredictedTrajectory,
            '/gp_predicted_trajectory',
            10
        )

        self.get_logger().info("Geo GP Prediction Node Started")

    def prompt_callback(self, msg: PromptTrajectory):
        n = len(msg.poses)
        self.get_logger().info(f"Received prompt trajectory with {n} poses")

        pred = self.predictor.predict(msg)

        self.pred_pub.publish(pred)

        self.get_logger().info(
            f"Published predicted trajectory | success={pred.success} | skill={pred.skill_name} | "
            f"confidence={pred.confidence}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = PredictionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
