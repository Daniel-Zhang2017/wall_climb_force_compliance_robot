import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

class TwistListener(Node):
    def __init__(self):
        super().__init__("twist_listener")
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.cb, qos_profile=qos)
        self.get_logger().info("Listener ready, BEST_EFFORT")

    def cb(self, msg):
        self.get_logger().info(f"✅ Received twist: linear.z = {msg.linear.z:.4f}")

def main():
    rclpy.init()
    node = TwistListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
