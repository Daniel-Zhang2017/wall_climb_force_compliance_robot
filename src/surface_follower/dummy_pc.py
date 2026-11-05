import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

class DummyDepthPoints(Node):
    def __init__(self):
        super().__init__("dummy_depth_points")
        # Use RELIABLE for RViz compatibility
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.pc_pub = self.create_publisher(PointCloud2, "/depth/points", qos)
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.get_logger().info("✅ Dummy /depth/points publisher ready (RELIABLE)")

    def timer_cb(self):
        xs = np.linspace(-0.2, 0.2, 10)
        ys = np.linspace(-0.2, 0.2, 10)
        xx, yy = np.meshgrid(xs, ys)
        zz = np.zeros_like(xx)

        points = np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=-1)
        points = points.astype(np.float32)

        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = "ft_sensor_link"

        msg = pc2.create_cloud_xyz32(header=hdr, points=points)
        self.pc_pub.publish(msg)

def main():
    rclpy.init()
    node = DummyDepthPoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
