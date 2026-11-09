#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class DummyWrench(Node):
    def __init__(self):
        super().__init__("dummy_wrench_pub")
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Tunable Fz parameter
        self.declare_parameter("fz", -14.0)
        self.fz = self.get_parameter("fz").value

        # Publish to /franka_robot_state/wrench (you remap this in launch)
        self.pub = self.create_publisher(WrenchStamped, "/franka_robot_state/wrench", qos_reliable)
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.get_logger().info(f"✅ Dummy wrench ready | topic=/franka_robot_state/wrench, Fz={self.fz:.2f}, frame_id=ft_sensor_link")

    def timer_cb(self):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ft_sensor_link"

        msg.wrench.force.x = 0.0
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = self.fz

        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyWrench()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
