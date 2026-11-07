#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import TwistStamped, TransformStamped, Vector3, WrenchStamped, Point
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from std_srvs.srv import Trigger
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from tf_transformations import quaternion_matrix


def transform_twist(twist_in: TwistStamped, transform: TransformStamped) -> TwistStamped:
    q = transform.transform.rotation
    R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    p = np.array([transform.transform.translation.x,
                  transform.transform.translation.y,
                  transform.transform.translation.z])

    v_in = np.array([twist_in.twist.linear.x, twist_in.twist.linear.y, twist_in.twist.linear.z])
    w_in = np.array([twist_in.twist.angular.x, twist_in.twist.angular.y, twist_in.twist.angular.z])

    v_rot = R @ v_in
    w_rot = R @ w_in

    twist_out = TwistStamped()
    twist_out.header.stamp = twist_in.header.stamp
    twist_out.header.frame_id = transform.header.frame_id
    twist_out.twist.linear.x = float(v_rot[0])
    twist_out.twist.linear.y = float(v_rot[1])
    twist_out.twist.linear.z = float(v_rot[2])
    twist_out.twist.angular.x = float(w_rot[0])
    twist_out.twist.angular.y = float(w_rot[1])
    twist_out.twist.angular.z = float(w_rot[2])
    return twist_out


def gram_schmidt_tangent(normal: np.ndarray) -> np.ndarray:
    ref = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(normal, ref)) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    tangent = ref - np.dot(ref, normal) * normal
    tangent = tangent / np.linalg.norm(tangent)
    return tangent


class SurfaceFollowerNode3(Node):
    def __init__(self):
        super().__init__("surface_follower_node3")
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameters(
            namespace="",
            parameters=[
                ("target_force_z", -15.0),
                ("kp_force", 0.02),
                ("max_vel", 0.15),
                ("force_deadzone", 0.5),
                ("marker_length", 0.15),
                ("enable_gravity_comp", False),
                ("gravity_force", [0.0, 0.0, 0.0]),
                ("sweep_speed", 0.03),
                ("enable_sweep", True),
                ("sweep_bounds", 0.4),
                ("servo_twist_topic", "/servo_node/delta_twist_cmds"),
                ("sensor_frame", "ft_sensor_link"),
                ("planning_frame", "panda_link0"),
                ("force_estop_threshold", -25.0),
                ("log_enable", False),
            ],
        )

        self.target_force_z = self.get_parameter("target_force_z").value
        self.kp_force = self.get_parameter("kp_force").value
        self.max_vel = self.get_parameter("max_vel").value
        self.force_deadzone = self.get_parameter("force_deadzone").value
        self.marker_length = self.get_parameter("marker_length").value
        self.enable_gravity_comp = self.get_parameter("enable_gravity_comp").value
        self.gravity_force = np.array(self.get_parameter("gravity_force").value)
        self.sweep_speed = self.get_parameter("sweep_speed").value
        self.enable_sweep = self.get_parameter("enable_sweep").value
        self.sweep_bounds = self.get_parameter("sweep_bounds").value
        self.servo_twist_topic = self.get_parameter("servo_twist_topic").value
        self.sensor_frame = self.get_parameter("sensor_frame").value
        self.planning_frame = self.get_parameter("planning_frame").value
        self.force_estop_threshold = self.get_parameter("force_estop_threshold").value
        self.log_enable = self.get_parameter("log_enable").value

        self.latest_force = None
        self.surface_normal = np.array([0.0, 0.0, 1.0])
        self.plane_centroid = np.array([0.0, 0.0, 0.0])
        self.has_normal = False
        self.sweep_sign = 1.0
        self.sweep_travel = 0.0
        self.estop_active = False
        self.last_time = self.get_clock().now()
        self.debug_counter = 0

        # ========== DEBUG SWITCH ==========
        self.DEBUG_FORCE_MOVE = True
        self.test_vel_x = 0.2
        # ==================================

        # Increase tf buffer duration
        self.tf_buffer = Buffer(rclpy.duration.Duration(seconds=1.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        self.force_sub = self.create_subscription(
            WrenchStamped, "/ft_sensor/wrench", self.force_callback, qos_reliable
        )
        self.pc_sub = self.create_subscription(
            PointCloud2, "/depth/points", self.pc_callback, qos_reliable
        )

        self.servo_twist_pub = self.create_publisher(TwistStamped, self.servo_twist_topic, 10)
        self.get_logger().info(f"✅ servo_twist_pub created, topic = {self.servo_twist_topic}")
        self.marker_pub = self.create_publisher(Marker, "/surface_marker", qos_reliable)

        self.reset_sweep_srv = self.create_service(
            Trigger, "~/reset_sweep", self.reset_sweep_cb, callback_group=self.cb_group
        )

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.get_logger().info("✅ SurfaceFollowerNode3 ready (with debug move switch)")
        if self.DEBUG_FORCE_MOVE:
            self.get_logger().warn("⚠️ DEBUG_FORCE_MOVE ENABLED: publishing fixed Z velocity, ignoring sensor data")

    def reset_sweep_cb(self, req, resp):
        self.sweep_sign = 1.0
        self.sweep_travel = 0.0
        self.estop_active = False
        resp.success = True
        resp.message = "Sweep reset, estop cleared"
        return resp

    def force_callback(self, msg: WrenchStamped):
        raw_f = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        if self.enable_gravity_comp:
            raw_f = raw_f - self.gravity_force
        self.latest_force = raw_f
        fz = raw_f[2]

        if fz < self.force_estop_threshold:
            self.estop_active = True
            self.get_logger().error(f"⚠️ FORCE ESTOP! Fz={fz:.2f}")
        elif self.estop_active and fz > (self.force_estop_threshold + 2.0):
            self.estop_active = False
            self.get_logger().info("✅ Force estop cleared")

    def pc_callback(self, msg: PointCloud2):
        points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pts_raw = np.array(list(points_gen))
        if pts_raw.shape[0] < 10:
            self.has_normal = False
            return
        pts_list = np.column_stack([pts_raw['x'], pts_raw['y'], pts_raw['z']]).astype(np.float32)

        centroid = np.mean(pts_list, axis=0)
        self.plane_centroid = centroid
        cov = np.cov((pts_list - centroid).T)
        _, _, vh = np.linalg.svd(cov)
        normal = vh[2, :]
        normal = normal / np.linalg.norm(normal)

        if np.dot(normal, self.surface_normal) < 0:
            normal = -normal
        self.surface_normal = normal
        self.has_normal = True
        self.publish_marker(msg.header.frame_id)

    def publish_marker(self, frame_id: str):
        cx = float(self.plane_centroid[0])
        cy = float(self.plane_centroid[1])
        cz = float(self.plane_centroid[2])
        nx = float(self.surface_normal[0])
        ny = float(self.surface_normal[1])
        nz = float(self.surface_normal[2])

        marker_n = Marker()
        marker_n.header.frame_id = frame_id
        marker_n.header.stamp = self.get_clock().now().to_msg()
        marker_n.ns = "normal"
        marker_n.id = 0
        marker_n.type = Marker.ARROW
        marker_n.action = Marker.ADD
        marker_n.pose.position.x = cx
        marker_n.pose.position.y = cy
        marker_n.pose.position.z = cz
        marker_n.scale.x = self.marker_length
        marker_n.scale.y = 0.005
        marker_n.scale.z = 0.005
        marker_n.color.r = 0.0
        marker_n.color.g = 1.0
        marker_n.color.b = 1.0
        marker_n.color.a = 1.0

        start = Point(x=cx, y=cy, z=cz)
        end = Point(
            x=cx + nx * self.marker_length,
            y=cy + ny * self.marker_length,
            z=cz + nz * self.marker_length,
        )
        marker_n.points = [start, end]
        self.marker_pub.publish(marker_n)

        tangent = gram_schmidt_tangent(self.surface_normal)
        tx = float(tangent[0])
        ty = float(tangent[1])
        tz = float(tangent[2])

        marker_t = Marker()
        marker_t.header.frame_id = frame_id
        marker_t.header.stamp = self.get_clock().now().to_msg()
        marker_t.ns = "sweep_dir"
        marker_t.id = 1
        marker_t.type = Marker.ARROW
        marker_t.action = Marker.ADD
        marker_t.pose.position.x = cx
        marker_t.pose.position.y = cy
        marker_t.pose.position.z = cz
        marker_t.scale.x = self.marker_length
        marker_t.scale.y = 0.005
        marker_t.scale.z = 0.005
        marker_t.color.r = 1.0
        marker_t.color.g = 1.0
        marker_t.color.b = 0.0
        marker_t.color.a = 1.0

        end_t = Point(
            x=cx + tx * self.marker_length,
            y=cy + ty * self.marker_length,
            z=cz + tz * self.marker_length,
        )
        marker_t.points = [start, end_t]
        self.marker_pub.publish(marker_t)

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Debug fixed velocity branch
        if self.DEBUG_FORCE_MOVE:
            twist_sensor = TwistStamped()
            twist_sensor.header.stamp = now.to_msg()
            twist_sensor.header.frame_id = self.sensor_frame
            twist_sensor.twist.linear.z = 0.0
            twist_sensor.twist.linear.x = self.test_vel_x
            twist_sensor.twist.linear.y = 0.0
            twist_sensor.twist.angular.x = 0.0
            twist_sensor.twist.angular.y = 0.0
            twist_sensor.twist.angular.z = 0.0
            try:
                # Use latest available transform for debug, no time extrapolation
                t_stamp = self.tf_buffer.lookup_transform(
                    self.planning_frame,
                    self.sensor_frame,
                    rclpy.time.Time()
                )
                twist_planning = transform_twist(twist_sensor, t_stamp)
                self.get_logger().info(f"🟢 DEBUG publish: linear.x={twist_planning.twist.linear.x:.4f}, frame={twist_planning.header.frame_id}")
            except Exception as e:
                self.get_logger().warn(f"TF lookup failed in debug move: {e}")
                # TF fail: publish zero velocity in planning frame
                twist_planning = TwistStamped()
                twist_planning.header.stamp = now.to_msg()
                twist_planning.header.frame_id = self.planning_frame
            self.servo_twist_pub.publish(twist_planning)
            return

        # Normal surface follower logic
        twist_sensor = TwistStamped()
        twist_sensor.header.stamp = now.to_msg()
        twist_sensor.header.frame_id = self.sensor_frame

        if self.estop_active or self.latest_force is None or (not self.has_normal):
            twist_planning = TwistStamped()
            twist_planning.header.stamp = now.to_msg()
            twist_planning.header.frame_id = self.planning_frame
            self.servo_twist_pub.publish(twist_planning)
            return

        fz = self.latest_force[2]
        err = fz - self.target_force_z
        vel_normal = 0.0
        if abs(err) > self.force_deadzone:
            vel_normal = self.kp_force * err
            vel_normal = np.clip(vel_normal, -self.max_vel, self.max_vel)

        v_normal = self.surface_normal * vel_normal
        v_tangent = np.zeros(3)

        if self.enable_sweep:
            tangent = gram_schmidt_tangent(self.surface_normal)
            v_tangent = tangent * self.sweep_speed * self.sweep_sign
            self.sweep_travel += self.sweep_speed * self.sweep_sign * dt
            if abs(self.sweep_travel) > self.sweep_bounds:
                self.sweep_sign *= -1.0
                self.sweep_travel = np.clip(self.sweep_travel, -self.sweep_bounds, self.sweep_bounds)
                self.get_logger().info(f"🔄 Reverse sweep direction, sign={self.sweep_sign}")

        v_total = v_normal + v_tangent
        twist_sensor.twist.linear.x = float(v_total[0])
        twist_sensor.twist.linear.y = float(v_total[1])
        twist_sensor.twist.linear.z = float(v_total[2])
        twist_sensor.twist.angular.x = 0.0
        twist_sensor.twist.angular.y = 0.0
        twist_sensor.twist.angular.z = 0.0

        tf_ok = False
        t_stamp = None
        try:
            # Lag compensation: query transform 0.1s earlier to avoid future extrapolation
            lookup_time = now - rclpy.duration.Duration(seconds=0.1)
            t_stamp = self.tf_buffer.lookup_transform(
                self.planning_frame,
                self.sensor_frame,
                lookup_time,
                timeout=rclpy.duration.Duration(seconds=0.3)
            )
            twist_planning = transform_twist(twist_sensor, t_stamp)
            tf_ok = True
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            twist_planning = TwistStamped()
            twist_planning.header.stamp = now.to_msg()
            twist_planning.header.frame_id = self.planning_frame

        self.debug_counter += 1
        if self.log_enable and self.debug_counter % 20 == 0:
            self.get_logger().info("===== SURFACE FOLLOWER DEBUG =====")
            self.get_logger().info(f"Sensor twist (ft_sensor_link): linear=[{twist_sensor.twist.linear.x:.3f}, {twist_sensor.twist.linear.y:.3f}, {twist_sensor.twist.linear.z:.3f}]")
            self.get_logger().info(f"Published twist frame_id: {twist_planning.header.frame_id}")
            self.get_logger().info(f"Target topic: {self.servo_twist_topic}")
            if tf_ok and t_stamp is not None:
                self.get_logger().info(f"TF OK | trans: x={t_stamp.transform.translation.x:.3f}, y={t_stamp.transform.translation.y:.3f}, z={t_stamp.transform.translation.z:.3f}")
            self.get_logger().info(f"Fz={fz:.2f}, err={err:.2f}, vn={vel_normal:.3f}, sweep_travel={self.sweep_travel:.3f}, sweep_sign={self.sweep_sign}")

        self.servo_twist_pub.publish(twist_planning)
        if self.log_enable:
            self.get_logger().info(
                f"Fz={fz:.2f}, err={err:.2f}, vn={vel_normal:.3f}, sweep_travel={self.sweep_travel:.3f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SurfaceFollowerNode3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
