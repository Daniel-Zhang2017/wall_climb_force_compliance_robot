#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class DummyDepthPoints(Node):
    def __init__(self):
        super().__init__("dummy_depth_points")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pc_pub = self.create_publisher(PointCloud2, "/depth/points", qos)
        self.timer = self.create_timer(0.1, self.timer_cb)

        # ROS parameters, match your plane marker
        self.declare_parameter("plane_center_x", 0.3)
        self.declare_parameter("plane_center_y", 0.0)
        self.declare_parameter("plane_z", 0.47)
        self.declare_parameter("plane_size", 0.6)
        self.declare_parameter("num_points", 120)
        self.declare_parameter("noise", 0.002)

        self.cx = self.get_parameter("plane_center_x").value
        self.cy = self.get_parameter("plane_center_y").value
        self.plane_z = self.get_parameter("plane_z").value
        self.plane_size = self.get_parameter("plane_size").value
        self.num_points = self.get_parameter("num_points").value
        self.noise = self.get_parameter("noise").value

        self.get_logger().info(
            f"✅ Dummy point cloud: center ({self.cx}, {self.cy}, {self.plane_z}), size={self.plane_size}"
        )

    def timer_cb(self):
        half = self.plane_size / 2.0
        # Sample points around center (cx, cy)
        xs = np.random.uniform(-half, half, self.num_points) + self.cx
        ys = np.random.uniform(-half, half, self.num_points) + self.cy
        zs = np.full_like(xs, self.plane_z) + np.random.normal(0, self.noise, self.num_points)

        pts = np.stack([xs, ys, zs], axis=1)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "panda_link0"

        cloud_msg = pc2.create_cloud_xyz32(header, pts)
        self.pc_pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyDepthPoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
