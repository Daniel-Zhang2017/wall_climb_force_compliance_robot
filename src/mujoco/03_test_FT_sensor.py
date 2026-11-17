import os
import mujoco
import mujoco.viewer
import numpy as np

xml_dir = "../franka_emika_panda"
xml_filename = "scene_pos2.xml"
xml_path = os.path.join(xml_dir, xml_filename)

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_center_site")
ft_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "ee_force")

# Home joint
q = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
data.qpos[:7] = q
mujoco.mj_forward(model, data)

# IK gains
k = 6
damping = 0.06
dt = model.opt.timestep

# ========== Admittance / Force Compliance Parameters (Z axis) ==========
F_desired_z = -4.0       # Target contact force (N, negative = push down)
B_adm = 6.0              # Virtual damping
force_deadband = 0.3     # Deadband to filter small force noise
# =======================================================================

# Motion sequence
mujoco.mj_forward(model, data)
p_start = data.site_xpos[site_id].copy()
plane_z = 0.21
p_down_target = np.array([p_start[0], p_start[1], plane_z])

waypoints_xy = np.array([
    [0.5,  0.0],
    [0.5,  0.2],
    [0.3,  0.2],
    [0.3,  0.0],
    [0.5,  0.0],
])
waypoint_idx = 0
stage = 1

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_forward(model, data)
        p_curr = data.site_xpos[site_id].copy()
        x_curr, y_curr, z_curr = p_curr

        Fx, Fy, Fz_meas = data.sensordata[ft_id : ft_id + 3]

        if stage == 1:
            p_des = p_down_target
            dp = p_des - p_curr
            if np.linalg.norm(p_curr - p_down_target) < 0.015:
                stage = 2
                print("Reached horizontal plane, start compliant XY tracing")
        else:
            xy_des = waypoints_xy[waypoint_idx]
            p_des = np.array([xy_des[0], xy_des[1], plane_z])

            dp_xy = p_des[:2] - p_curr[:2]
            vel_xy = k * dp_xy

            # Z admittance with deadband, SIGN FIXED
            F_err_z = Fz_meas - F_desired_z
            if abs(F_err_z) < force_deadband:
                F_err_z = 0.0
            vel_z = - F_err_z / B_adm

            dp = np.array([vel_xy[0], vel_xy[1], vel_z])

            if np.linalg.norm(p_curr[:2] - xy_des) < 0.015:
                print(f"✅ Arrived at waypoint: x={xy_des[0]:.2f}, y={xy_des[1]:.2f}")
                waypoint_idx += 1
                if waypoint_idx >= len(waypoints_xy):
                    waypoint_idx = 0
                    print("🔁 Rectangle loop finished, restart trajectory")

        # DLS IK
        J = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, J, None, site_id)
        J = J[:, :7]

        dq = J.T @ np.linalg.inv(J @ J.T + damping**2 * np.eye(3)) @ dp
        q = q + dq * dt
        q = np.clip(q, model.jnt_range[:7, 0], model.jnt_range[:7, 1])

        data.ctrl[:7] = q
        data.ctrl[7:9] = 0.04  # gripper open

        mujoco.mj_step(model, data)
        viewer.sync()

        # Print real-time position + force
        print(f"x: {x_curr:.4f} m | y: {y_curr:.4f} m | z: {z_curr:.4f} m | Fx: {Fx:.2f} Fy: {Fy:.2f} Fz_meas: {Fz_meas:.2f} N | Fz_target: {F_desired_z:.2f} N")

print("Simulation ended")
