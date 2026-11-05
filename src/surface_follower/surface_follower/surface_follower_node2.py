import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, Twist
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2


class SurfaceFollowerNode2(Node):
    def __init__(self):
        super().__init__("surface_follower_node2")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("target_force_z", -15.0),
                ("kp_force", 0.02),
                ("max_vel", 0.15),
                ("force_deadzone", 0.5),
                ("marker_length", 0.15),
                ("enable_gravity_compensation", False),
                ("gravity_force", [0.0, 0.0, -9.81]),
                ("sweep_speed", 0.03),
                ("enable_sweep", True),
                ("sweep_bounds", 0.4),
            ]
        )
        self.use_sim_time = self.get_parameter("use_sim_time").value
        self.target_force_z = self.get_parameter("target_force_z").value
        self.kp_force = self.get_parameter("kp_force").value
        self.max_vel = self.get_parameter("max_vel").value
        self.force_deadzone = self.get_parameter("force_deadzone").value
        self.marker_len = self.get_parameter("marker_length").value
        self.enable_grav_comp = self.get_parameter("enable_gravity_compensation").value
        self.grav_force = np.array(self.get_parameter("gravity_force").value)
        self.sweep_speed = self.get_parameter("sweep_speed").value
        self.enable_sweep = self.get_parameter("enable_sweep").value
        self.sweep_bounds = self.get_parameter("sweep_bounds").value

        self.latest_fz = None
        self.surface_normal = np.array([0.0, 0.0, 1.0])
        self.plane_centroid = np.array([0.0, 0.0, 0.0])
        self.got_valid_normal = False

        # Bounce sweep state
        self.sweep_sign = 1.0
        self.sweep_travel = 0.0
        self.dt = 0.1

        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        self.ft_sub = self.create_subscription(
            WrenchStamped, "/ft_sensor/wrench", self.wrench_callback, qos_reliable
        )
        self.pc_sub = self.create_subscription(
            PointCloud2, "/depth/points", self.pointcloud_callback, qos_reliable
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", qos_reliable)
        self.marker_pub = self.create_publisher(Marker, "/surface_normal_marker", qos_reliable)

        self.control_timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("✅ SVD surface follower + bounce sweep ready")

    def wrench_callback(self, msg: WrenchStamped):
        raw_fz = msg.wrench.force.z
        if self.enable_grav_comp:
            raw_fz -= self.grav_force[2]
        self.latest_fz = raw_fz

    def pointcloud_callback(self, msg: PointCloud2):
        pts = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if len(pts) < 3:
            self.get_logger().warn("Not enough points for plane fitting")
            self.got_valid_normal = False
            return

        pts_np = np.array(pts)
        pts_np = np.column_stack([pts_np["x"], pts_np["y"], pts_np["z"]]).astype(np.float32)
        centroid = np.mean(pts_np, axis=0)
        self.plane_centroid = centroid
        X = pts_np - centroid

        # SVD plane fit
        _, _, vh = np.linalg.svd(X, full_matrices=False)
        normal = vh[-1, :]
        if normal[2] < 0:
            normal = -normal
        normal = normal / np.linalg.norm(normal)
        self.surface_normal = normal
        self.got_valid_normal = True
        self.publish_marker(msg.header.frame_id)

    def publish_marker(self, frame_id: str):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "surface_normal"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD

        m.pose.position.x = float(self.plane_centroid[0])
        m.pose.position.y = float(self.plane_centroid[1])
        m.pose.position.z = float(self.plane_centroid[2])

        target_dir = self.surface_normal
        unit_x = np.array([1.0, 0.0, 0.0])

        if np.allclose(target_dir, unit_x):
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        elif np.allclose(target_dir, -unit_x):
            qx, qy, qz, qw = 0.0, 0.0, 1.0, 0.0
        else:
            cross = np.cross(unit_x, target_dir)
            dot = np.dot(unit_x, target_dir)
            s = np.sqrt((1 + dot) / 2.0)
            inv = 1.0 / (2.0 * s)
            qx = float(cross[0] * inv)
            qy = float(cross[1] * inv)
            qz = float(cross[2] * inv)
            qw = float(s)

        m.pose.orientation.x = qx
        m.pose.orientation.y = qy
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw

        m.scale.x = self.marker_len
        m.scale.y = 0.015
        m.scale.z = 0.015
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0
        self.marker_pub.publish(m)

    def publish_sweep_marker(self, frame_id: str, sweep_dir: np.ndarray):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "sweep_direction"
        m.id = 1
        m.type = Marker.ARROW
        m.action = Marker.ADD

        m.pose.position.x = float(self.plane_centroid[0])
        m.pose.position.y = float(self.plane_centroid[1])
        m.pose.position.z = float(self.plane_centroid[2])

        target_dir = sweep_dir
        unit_x = np.array([1.0, 0.0, 0.0])

        if np.linalg.norm(target_dir) < 1e-6:
            m.action = Marker.DELETE
            self.marker_pub.publish(m)
            return

        target_dir = target_dir / np.linalg.norm(target_dir)

        if np.allclose(target_dir, unit_x):
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        elif np.allclose(target_dir, -unit_x):
            qx, qy, qz, qw = 0.0, 0.0, 1.0, 0.0
        else:
            cross = np.cross(unit_x, target_dir)
            dot = np.dot(unit_x, target_dir)
            s = np.sqrt((1 + dot) / 2.0)
            inv = 1.0 / (2.0 * s)
            qx = float(cross[0] * inv)
            qy = float(cross[1] * inv)
            qz = float(cross[2] * inv)
            qw = float(s)

        m.pose.orientation.x = qx
        m.pose.orientation.y = qy
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw

        m.scale.x = self.marker_len * 0.8
        m.scale.y = 0.012
        m.scale.z = 0.012
        # Yellow
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0

        self.marker_pub.publish(m)

    def control_loop(self):
        twist = Twist()
        if self.latest_fz is None:
            self.cmd_vel_pub.publish(twist)
            return

        n = self.surface_normal
        vel_mag_normal = 0.0

        # -------- normal force close loop --------
        error = self.latest_fz - self.target_force_z
        if abs(error) < self.force_deadzone:
            vel_mag_normal = 0.0
        else:
            vel_mag_normal = self.kp_force * error
            vel_mag_normal = np.clip(vel_mag_normal, -self.max_vel, self.max_vel)
        v_normal = n * vel_mag_normal

        # -------- tangential bounce sweep --------
        v_tangent = np.zeros(3)
        if self.enable_sweep:
            ref_y = np.array([0.0, 1.0, 0.0])
            t = ref_y - n * np.dot(ref_y, n)
            t_norm = np.linalg.norm(t)
            if t_norm > 1e-6:
                t = t / t_norm
                # apply sign
                v_tangent = t * self.sweep_speed * self.sweep_sign
                # integrate travel
                self.sweep_travel += self.sweep_speed * self.sweep_sign * self.dt

                # reverse direction when hit bound
                if abs(self.sweep_travel) > self.sweep_bounds:
                    self.sweep_sign *= -1.0
                    self.sweep_travel = np.clip(self.sweep_travel, -self.sweep_bounds, self.sweep_bounds)
                    self.get_logger().info(f"🔄 Reverse sweep direction! sign={self.sweep_sign}")

        v_total = v_normal + v_tangent

        twist.linear.x = float(v_total[0])
        twist.linear.y = float(v_total[1])
        twist.linear.z = float(v_total[2])

        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(
            f"Fz:{self.latest_fz:.2f} | normal_v:{vel_mag_normal:.4f} | sweep_dir:{np.round(v_tangent,3)} | travel:{self.sweep_travel:.3f}"
        )

        if self.got_valid_normal:
            self.publish_sweep_marker("ft_sensor_link", v_tangent)


def main():
    rclpy.init()
    node = SurfaceFollowerNode2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
