#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import TwistStamped, TransformStamped, Vector3, WrenchStamped, Point
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from std_srvs.srv import Trigger
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from tf_transformations import quaternion_matrix

import cvxpy as cp

def transform_twist(twist_in: TwistStamped, transform: TransformStamped, target_frame: str) -> TwistStamped:
    """
    Transform a twist (linear + angular velocity) from source frame to target frame.
    Only rotate the velocity vector (twist does NOT get translated).
    Used here: convert velocity command from ft_sensor_link frame to panda_link0 (planning frame) for MoveIt Servo.
    """
    q = transform.transform.rotation
    R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

    v_in = np.array([twist_in.twist.linear.x, twist_in.twist.linear.y, twist_in.twist.linear.z])
    w_in = np.array([twist_in.twist.angular.x, twist_in.twist.angular.y, twist_in.twist.angular.z])

    v_out = R @ v_in
    w_out = R @ w_in

    twist_out = TwistStamped()
    twist_out.header.stamp = twist_in.header.stamp
    twist_out.header.frame_id = target_frame
    twist_out.twist.linear.x = float(v_out[0])
    twist_out.twist.linear.y = float(v_out[1])
    twist_out.twist.linear.z = float(v_out[2])
    twist_out.twist.angular.x = float(w_out[0])
    twist_out.twist.angular.y = float(w_out[1])
    twist_out.twist.angular.z = float(w_out[2])
    return twist_out


def gram_schmidt_tangent(normal: np.ndarray) -> np.ndarray:
    """
    Given surface normal vector, compute one tangent vector lying ON the surface.
    This tangent direction is the sweep direction for the arm to scan along the surface.
    Gram-Schmidt orthogonalization: subtract normal component from reference axis, then normalize.
    """
    ref = np.array([1.0, 0.0, 0.0])
    if np.allclose(normal, ref):
        ref = np.array([0.0, 1.0, 0.0])
    tangent = ref - np.dot(ref, normal) * normal
    tangent = tangent / np.linalg.norm(tangent)
    return tangent

def solve_mpc(self, f_meas, f_target, dt):
    """
    MPC QP solver for force regulation
    Model: f_{k+1} = f_k + A * vn * dt
    Input:
        f_meas: current measured Fz
        f_target: desired Fz
        dt: time step
    Return:
        vn_opt: optimal normal velocity (only first action)
    """
    N = self.N
    A = self.A * dt

    # Decision variable: predicted normal velocity sequence
    vn = cp.Variable(N)
    f_pred = cp.Variable(N+1)

    constraints = []
    constraints += [f_pred[0] == f_meas]

    # State prediction constraints
    for k in range(N):
        constraints += [f_pred[k+1] == f_pred[k] + A * vn[k]]
        constraints += [vn[k] >= self.min_vel]
        constraints += [vn[k] <= self.max_vel]

    # Cost function
    cost = 0.0
    for k in range(N):
        cost += self.Q * cp.square(f_pred[k+1] - f_target)
        cost += self.R * cp.square(vn[k])
    # Delta input penalty (smoothness)
    cost += self.Rd * cp.square(vn[0] - self.prev_vn)
    for k in range(1, N):
        cost += self.Rd * cp.square(vn[k] - vn[k-1])

    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.CVXOPT, verbose=False)

    if prob.status != cp.OPTIMAL:
        self.get_logger().warn(f"MPC infeasible, status: {prob.status}")
        return 0.0

    vn_opt = vn.value[0]
    self.prev_vn = vn_opt
    return vn_opt

class SurfaceFollowerNode5(Node):
    def __init__(self):
        super().__init__("surface_follower_node5")
        self.cb_group = ReentrantCallbackGroup()
        # Reentrant callback group: allow force / pointcloud / control timer callbacks run concurrently

        # ========== ROS Parameters ==========
        self.declare_parameters(
            namespace="",
            parameters=[
                ("target_force_z", -15.0),        # Desired contact force along Z (sensor frame)
                ("A", 20.0),
                ("horizon_N", 8),
                ("Q_force", 1.0),
                ("R_vel", 0.05),
                ("R_delta", 0.1),
                ("min_vel", -0.15),
                ("max_vel", 0.15),

                ("max_vel", 0.15),                # Maximum Cartesian velocity (m/s)
                ("force_deadzone", 0.5),          # Deadzone: ignore small force error to avoid jitter
                ("marker_length", 0.15),          # Length of normal / sweep arrow markers in RViz
                ("enable_gravity_comp", False),    # Gravity compensation for FT sensor
                ("gravity_force", [0.0, 0.0, 0.0]),
                ("sweep_speed", 0.2),            # Speed to scan along surface tangent (m/s)
                ("enable_sweep", True),
                ("sweep_limit", 4.0),             # Max sweep travel distance before reversing direction
                ("servo_twist_topic", "/servo_node/delta_twist_cmds"),
                ("sensor_frame", "ft_sensor_link"),
                ("planning_frame", "panda_link0"),
                ("force_estop_threshold", -25.0), # Emergency stop: if force exceeds this negative value
                ("log_enable", True),
                ("DEBUG_FORCE_MOVE", False),       # Debug mode: fixed test velocity, skip force control
                ("test_vel_x", 0.02),
                ("test_vel_z", 0.0),
                # --- NEW: Horizontal plane marker parameters ---
                ("plane_z", 0.47),                # Z height of reference plane (panda_link0)
                ("plane_size", 0.6),              # Square side length of plane
                ("plane_alpha", 0.75),            # Transparency: 0=fully transparent, 1=opaque
            ],
        )

        self.target_force_z = self.get_parameter("target_force_z").value
        # MPC parameters
        self.A = self.get_parameter("A").value
        self.N = self.get_parameter("horizon_N").value
        self.Q = self.get_parameter("Q_force").value
        self.R = self.get_parameter("R_vel").value
        self.Rd = self.get_parameter("R_delta").value
        self.min_vel = self.get_parameter("min_vel").value
        self.max_vel = self.get_parameter("max_vel").value

        # Save previous control input for delta cost
        self.prev_vn = 0.0


        self.max_vel = self.get_parameter("max_vel").value
        self.force_deadzone = self.get_parameter("force_deadzone").value
        self.marker_length = self.get_parameter("marker_length").value
        self.enable_gravity_comp = self.get_parameter("enable_gravity_comp").value
        self.gravity_force = np.array(self.get_parameter("gravity_force").value)
        self.sweep_speed = self.get_parameter("sweep_speed").value
        self.enable_sweep = self.get_parameter("enable_sweep").value
        self.sweep_limit = self.get_parameter("sweep_limit").value
        self.servo_twist_topic = self.get_parameter("servo_twist_topic").value
        self.sensor_frame = self.get_parameter("sensor_frame").value
        self.planning_frame = self.get_parameter("planning_frame").value
        self.force_estop_threshold = self.get_parameter("force_estop_threshold").value
        self.log_enable = self.get_parameter("log_enable").value
        self.debug_force_move = self.get_parameter("DEBUG_FORCE_MOVE").value
        self.test_vel_x = self.get_parameter("test_vel_x").value
        self.test_vel_z = self.get_parameter("test_vel_z").value

        self.plane_z = self.get_parameter("plane_z").value
        self.plane_size = self.get_parameter("plane_size").value
        self.plane_alpha = self.get_parameter("plane_alpha").value

        # ========== Runtime State Variables ==========
        self.latest_force = None                 # Latest wrench reading from FT sensor
        self.surface_normal = np.array([0.0, 0.0, 1.0]) # Estimated surface normal
        self.has_normal = False                  # Flag: valid normal from point cloud
        self.sweep_sign = 1.0                    # +1 / -1 for sweep direction toggle
        self.sweep_distance = 0.0                # Accumulated sweep travel distance
        self.estop_active = False
        self.last_time = self.get_clock().now()
        self.debug_counter = 0

        # PID states for force control
        self.force_integral = 0.0
        self.last_force_error = 0.0


        # TF buffer & listener for frame transformation
        self.tf_buffer = Buffer(rclpy.duration.Duration(seconds=2.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ========== Subscribers ==========
        self.force_sub = self.create_subscription(
            WrenchStamped, "/ft_sensor/wrench", self.force_callback, qos_reliable, callback_group=self.cb_group
        )
        self.pc_sub = self.create_subscription(
            PointCloud2, "/depth/points", self.pc_callback, qos_reliable, callback_group=self.cb_group
        )

        # ========== Publishers ==========
        self.servo_twist_pub = self.create_publisher(TwistStamped, self.servo_twist_topic, qos_reliable)
        self.marker_pub = self.create_publisher(Marker, "/surface_marker", qos_reliable)

        # ========== Service: reset sweep ==========
        self.reset_sweep_srv = self.create_service(Trigger, "~/reset_sweep", self.reset_sweep_cb, callback_group=self.cb_group)

        # ========== Timers ==========
        self.timer_period = 0.3
        self.control_timer = self.create_timer(self.timer_period, self.control_loop, callback_group=self.cb_group)
        # New slow timer: publish horizontal plane marker every 0.5s (marker needs periodic refresh for RViz)
        self.plane_timer = self.create_timer(0.5, self.publish_horizontal_plane)

        self.get_logger().info(f"✅ servo_twist_pub created, topic = {self.servo_twist_topic}")
        self.get_logger().info("✅ SurfaceFollowerNode3 ready")

    def publish_horizontal_plane(self):
        """
        Publish a thin, semi-transparent square horizontal plane marker for RViz visualization.
        Frame: planning_frame (panda_link0). This is the reference surface we want the arm to follow.
        Marker type: CUBE with extremely small Z thickness (0.001m) to simulate flat sheet.
        Color: light green, configurable transparency.
        """
        plane_marker = Marker()
        plane_marker.header.frame_id = self.planning_frame
        plane_marker.header.stamp = self.get_clock().now().to_msg()
        plane_marker.ns = "horizontal_plane"
        plane_marker.id = 10
        plane_marker.type = Marker.CUBE
        plane_marker.action = Marker.ADD

        plane_marker.pose.position.x = 0.3
        plane_marker.pose.position.y = 0.0
        plane_marker.pose.position.z = self.plane_z
        plane_marker.pose.orientation.w = 1.0

        plane_marker.scale.x = self.plane_size
        plane_marker.scale.y = self.plane_size
        plane_marker.scale.z = 0.01
        plane_marker.color.r = 0.0
        plane_marker.color.g = 0.8
        plane_marker.color.b = 0.2
        plane_marker.color.a = self.plane_alpha

        self.marker_pub.publish(plane_marker)

    def force_callback(self, msg: WrenchStamped):
        """Callback for FT sensor wrench message. Read contact force, check force estop."""
        self.get_logger().info(f"🔥 force_callback triggered! Fz = {msg.wrench.force.z:.2f}")
        raw_f = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        if self.enable_gravity_comp:
            raw_f = raw_f - self.gravity_force
        self.latest_force = raw_f

        fz = raw_f[2]
        if fz < self.force_estop_threshold:
            self.estop_active = True
            self.get_logger().error(f"🛑 FORCE ESTOP, fz={fz:.2f}")
        else:
            self.estop_active = False

    def pc_callback(self, msg: PointCloud2):
        """
        Callback for point cloud (/depth/points).
        Estimate surface normal using SVD on point cloud covariance:
        SVD: eigenvector with smallest eigenvalue = normal direction of fitted plane.
        Flip normal direction to keep consistent orientation, then publish normal & tangent arrows.
        """
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pts_raw = list(points)
        if len(pts_raw) < 10:
            self.has_normal = False
            return
        pts = np.array([[p[0], p[1], p[2]] for p in pts_raw])
        centroid = np.mean(pts, axis=0)
        cov = np.cov((pts - centroid).T)
        _, _, vh = np.linalg.svd(cov)
        normal = vh[2, :]
        if np.dot(normal, self.surface_normal) < 0:
            normal = -normal
        self.surface_normal = normal / np.linalg.norm(normal)
        self.has_normal = True
        self.publish_marker(centroid, self.surface_normal)

    def publish_marker(self, origin: np.ndarray, normal: np.ndarray):
        """
        Publish two arrow markers:
        - Cyan arrow: surface normal (contact direction)
        - Yellow arrow: sweep tangent (scan direction along surface)
        Frame: ft_sensor_link
        """
        marker_normal = Marker()
        marker_normal.header.frame_id = self.sensor_frame
        marker_normal.header.stamp = self.get_clock().now().to_msg()
        marker_normal.ns = "normal"
        marker_normal.id = 0
        marker_normal.type = Marker.ARROW
        marker_normal.action = Marker.ADD
        marker_normal.pose.orientation.w = 1.0
        marker_normal.scale.x = self.marker_length
        marker_normal.scale.y = 0.005
        marker_normal.scale.z = 0.005
        marker_normal.color.r = 0.0
        marker_normal.color.g = 1.0
        marker_normal.color.b = 1.0
        marker_normal.color.a = 1.0
        p0 = Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2]))
        p1 = Point(
            x=float(origin[0] + normal[0]*self.marker_length),
            y=float(origin[1] + normal[1]*self.marker_length),
            z=float(origin[2] + normal[2]*self.marker_length)
        )
        marker_normal.points.append(p0)
        marker_normal.points.append(p1)
        self.marker_pub.publish(marker_normal)

        tangent = gram_schmidt_tangent(normal)
        marker_tangent = Marker()
        marker_tangent.header.frame_id = self.sensor_frame
        marker_tangent.header.stamp = self.get_clock().now().to_msg()
        marker_tangent.ns = "sweep_dir"
        marker_tangent.id = 1
        marker_tangent.type = Marker.ARROW
        marker_tangent.action = Marker.ADD
        marker_tangent.pose.orientation.w = 1.0
        marker_tangent.scale.x = self.marker_length
        marker_tangent.scale.y = 0.005
        marker_tangent.scale.z = 0.005
        marker_tangent.color.r = 1.0
        marker_tangent.color.g = 1.0
        marker_tangent.color.b = 0.0
        marker_tangent.color.a = 1.0
        p0t = Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2]))
        p1t = Point(
            x=float(origin[0] + tangent[0]*self.marker_length),
            y=float(origin[1] + tangent[1]*self.marker_length),
            z=float(origin[2] + tangent[2]*self.marker_length)
        )
        marker_tangent.points.append(p0t)
        marker_tangent.points.append(p1t)
        self.marker_pub.publish(marker_tangent)

    def reset_sweep_cb(self, req, resp):
        """ROS service: reset sweep direction and accumulated distance."""
        self.sweep_sign = 1.0
        self.sweep_distance = 0.0
        resp.success = True
        resp.message = "sweep reset"
        return resp

    def control_loop(self):
        """
        Main control loop (10Hz):
        1. Debug branch: fixed velocity if DEBUG_FORCE_MOVE=True
        2. Guard condition: stop if estop / no force / no valid normal
        3. Force feedback: compute velocity along surface normal to maintain target contact force
        4. Tangent sweep: move along surface tangent, reverse after sweep_limit
        5. Velocity saturation: cap total velocity under max_vel
        6. Transform twist from sensor frame to base frame and publish to MoveIt Servo
        """
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        twist_cmd = TwistStamped()
        twist_cmd.header.stamp = now.to_msg()
        twist_cmd.header.frame_id = self.planning_frame

        if self.debug_force_move:
            twist_sensor = TwistStamped()
            twist_sensor.header.stamp = now.to_msg()
            twist_sensor.header.frame_id = self.sensor_frame
            twist_sensor.twist.linear.x = self.test_vel_x
            twist_sensor.twist.linear.z = self.test_vel_z

            try:
                t = self.tf_buffer.lookup_transform(
                    self.planning_frame, self.sensor_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1)
                )
                twist_cmd = transform_twist(twist_sensor, t, self.planning_frame)
            except Exception as e:
                self.get_logger().warn(f"Debug TF lookup failed: {e}")
                twist_cmd.header.frame_id = self.planning_frame
                twist_cmd.twist.linear.x = 0.0
                twist_cmd.twist.linear.y = 0.0
                twist_cmd.twist.linear.z = 0.0
            self.servo_twist_pub.publish(twist_cmd)
            self.get_logger().info(f"🟢 DEBUG move, x={self.test_vel_x:.3f}, z={self.test_vel_z:.3f}")
            return

        if self.estop_active or self.latest_force is None or not self.has_normal:
            twist_cmd.twist.linear.x = 0.0
            twist_cmd.twist.linear.y = 0.0
            twist_cmd.twist.linear.z = 0.0
            self.servo_twist_pub.publish(twist_cmd)
            
            self.prev_vn = 0.0

            self.debug_counter += 1
            if self.log_enable and self.debug_counter % 20 == 0:
                self.get_logger().info(f"🚫 Guard: estop={self.estop_active}, force_ok={self.latest_force is not None}, has_normal={self.has_normal}")
            return

        fz = self.latest_force[2]
        
        err = fz - self.target_force_z
        vel_normal = 0.0

        if abs(err) > self.force_deadzone:
            vel_normal = self.solve_mpc(fz, self.target_force_z, dt)
        else:
            vel_normal = 0.0
            # freeze previous input
            self.prev_vn = self.prev_vn

        tangent = gram_schmidt_tangent(self.surface_normal)
        vel_tangent = self.sweep_sign * self.sweep_speed * tangent

        # Combine normal (force control) + tangent (scanning) velocity
        vel_total = vel_normal * self.surface_normal + vel_tangent
        v_norm = np.linalg.norm(vel_total)
        if v_norm > self.max_vel:
            vel_total = vel_total / v_norm * self.max_vel

        self.sweep_distance += self.sweep_sign * self.sweep_speed * dt
        if abs(self.sweep_distance) > self.sweep_limit:
            self.sweep_sign *= -1
            self.sweep_distance = 0.0
            self.get_logger().info(f"🔄 Reverse sweep direction, sign={self.sweep_sign}")

        twist_sensor = TwistStamped()
        twist_sensor.header.stamp = now.to_msg()
        twist_sensor.header.frame_id = self.sensor_frame
        twist_sensor.twist.linear.x = float(vel_total[0])
        twist_sensor.twist.linear.y = float(vel_total[1])
        twist_sensor.twist.linear.z = float(vel_total[2])

        try:
            t = self.tf_buffer.lookup_transform(
                self.planning_frame, self.sensor_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1)
            )
            twist_cmd = transform_twist(twist_sensor, t, self.planning_frame)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            twist_cmd.header.frame_id = self.planning_frame
            twist_cmd.twist.linear.x = 0.0
            twist_cmd.twist.linear.y = 0.0
            twist_cmd.twist.linear.z = 0.0

        self.servo_twist_pub.publish(twist_cmd)

        if self.log_enable:
            self.get_logger().info(
                f"Fz={fz:.2f}, err={err:.2f}, vn={vel_normal:.3f}, "
                f"vel_total={vel_total.round(3)}, sweep_dist={self.sweep_distance:.3f}, sign={self.sweep_sign}, "
                f"out_base_twist=({twist_cmd.twist.linear.x:.3f}, {twist_cmd.twist.linear.y:.3f}, {twist_cmd.twist.linear.z:.3f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SurfaceFollowerNode5()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
