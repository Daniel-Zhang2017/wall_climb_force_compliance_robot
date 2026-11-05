import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, Twist
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class SurfaceFollowerNode(Node):
    def __init__(self):
        super().__init__("surface_follower_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("target_force_z", -15.0),
                ("kp_force", 0.02),
                ("max_vel", 0.15),
                ("force_deadzone", 0.5),
            ]
        )
        self.target_force_z = self.get_parameter("target_force_z").value
        self.kp_force = self.get_parameter("kp_force").value
        self.max_vel = self.get_parameter("max_vel").value
        self.force_deadzone = self.get_parameter("force_deadzone").value

        # cache
        self.latest_fz = None
        self.surface_normal = np.array([0.0, 0.0, 1.0])  # fallback normal +z
        self.got_valid_normal = False

        # subscribers
        self.ft_sub = self.create_subscription(
            WrenchStamped, "/ft_sensor/wrench", self.wrench_callback, qos_profile_sensor_data
        )
        self.pc_sub = self.create_subscription(
            PointCloud2, "/depth/points", self.pointcloud_callback, qos_profile_sensor_data
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", qos_profile_sensor_data)

        # control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("✅ Node initialized, timer created, pointcloud enabled")

    def wrench_callback(self, msg: WrenchStamped):
        self.latest_fz = msg.wrench.force.z
        self.get_logger().info(f"👉 Received wrench, fz = {self.latest_fz:.2f}")

    def pointcloud_callback(self, msg: PointCloud2):
        pts = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if len(pts) < 3:
            self.get_logger().warn("Not enough points for PCA normal estimation")
            self.got_valid_normal = False
            return

        pts_np = np.array(pts)
        pts_np = np.column_stack([pts_np['x'], pts_np['y'], pts_np['z']]).astype(np.float32)

        centroid = np.mean(pts_np, axis=0)
        cov = np.cov((pts_np - centroid).T)
        eigvals, eigvecs = np.linalg.eig(cov)
        idx = np.argmin(eigvals)
        normal = eigvecs[:, idx].real
        if normal[2] < 0:
            normal = -normal
        normal = normal / np.linalg.norm(normal)

        self.surface_normal = normal
        self.got_valid_normal = True
        self.get_logger().info(f"📌 Updated surface normal: {normal.round(3)}")

    def control_loop(self):
        self.get_logger().info("=== control loop tick ===")
        twist = Twist()
        if self.latest_fz is None:
            self.cmd_vel_pub.publish(twist)
            return

        error = self.latest_fz - self.target_force_z
        self.get_logger().info(f"fz={self.latest_fz:.2f}, target={self.target_force_z:.2f}, error={error:.2f}")

        if abs(error) < self.force_deadzone:
            vel_mag = 0.0
        else:
            vel_mag = self.kp_force * error
            vel_mag = np.clip(vel_mag, -self.max_vel, self.max_vel)

        n = self.surface_normal
        twist.linear.x = n[0] * vel_mag
        twist.linear.y = n[1] * vel_mag
        twist.linear.z = n[2] * vel_mag

        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"📤 Publish twist | mag:{vel_mag:.4f}, normal:{n.round(3)}")

def main():
    rclpy.init()
    node = SurfaceFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
