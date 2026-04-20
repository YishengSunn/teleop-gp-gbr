import sys
import threading
import rclpy

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class GeoGPToggle(Node):
    def __init__(self):
        super().__init__('geo_gp_toggle')
        self.declare_parameter('enabled_topic', '/geo_gp/enabled')
        self.declare_parameter('start_enabled', False)

        enabled_topic = self.get_parameter('enabled_topic').get_parameter_value().string_value
        self.enabled = self.get_parameter('start_enabled').get_parameter_value().bool_value
        self._shutdown = False
        self._qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(Bool, enabled_topic, self._qos)
        self._pending_shutdown = False
        self._check_timer = self.create_timer(0.1, self.check_shutdown)

        self.get_logger().info(
            "Geo-GP keyboard toggle ready | "
            f"topic={enabled_topic} | initial_enabled={self.enabled}"
        )
        self.publish_state()

        self._input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self._input_thread.start()

    def publish_state(self):
        msg = Bool()
        msg.data = self.enabled
        self.pub.publish(msg)
        self.get_logger().info(
            f"Geo-GP {'ENABLED' if self.enabled else 'DISABLED'} | "
            "press 'g' then Enter to toggle, 'q' then Enter to quit"
        )

    def input_loop(self):
        while not self._shutdown:
            try:
                user_input = input().strip().lower()
            except EOFError:
                self._shutdown = True
                self._pending_shutdown = True
                return

            if user_input == 'g':
                self.enabled = not self.enabled
                self.publish_state()
            elif user_input == 'q':
                self._shutdown = True
                self._pending_shutdown = True
                return
            elif user_input:
                self.get_logger().info("Unknown command. Use 'g' to toggle or 'q' to quit.")

    def check_shutdown(self):
        if not self._pending_shutdown:
            return

        self._pending_shutdown = False
        self.get_logger().info('Shutting down Geo-GP keyboard toggle')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GeoGPToggle()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._shutdown = True
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
