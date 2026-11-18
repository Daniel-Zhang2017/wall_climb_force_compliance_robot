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

# Home joint
q = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
data.qpos[:7] = q
mujoco.mj_forward(model, data)
print(f"Initial EE Z = {data.site_xpos[site_id][2]:.4f}")

# IK gains
k_pos = 6.0
damping = 0.06
dt = model.opt.timestep

# Table & hover setting
table_z = 0.21
clearance = 0.05
hover_z = table_z + clearance

# Cleaning scan parameters
x_start = 0.3
x_end   = 0.6
y_min   = 0.0
y_max   = 0.3
x_step  = 0.025   # increase X by this value after each left-right sweep
scan_direction = 1  # 1: y+, -1: y-
current_x = x_start

STALL_TOL = 2e-5
STALL_COUNT_LIMIT = 80
stall_counter = 0
prev_z = data.site_xpos[site_id][2]

# First target: go to start scan point
first_target = np.array([current_x, y_min, hover_z])
stage = 1   # 1: descend to start point, 2: scan cleaning

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_forward(model, data)
        p_curr = data.site_xpos[site_id].copy()
        x_curr, y_curr, z_curr = p_curr

        dp = np.zeros(3)
        if stage == 1:
            err = first_target - p_curr
            dp = k_pos * err

            dz = z_curr - prev_z
            if abs(dz) < STALL_TOL:
                stall_counter += 1
            else:
                stall_counter = 0
            prev_z = z_curr

            print(f"descending to scan start | z={z_curr:.4f}, stall_cnt={stall_counter}")
            if stall_counter >= STALL_COUNT_LIMIT:
                stage = 2
                print(f">>> Start surface cleaning scan, hover z={hover_z:.3f}")
        else:
            # Stage2: back-and-forth scan
            if scan_direction == 1:
                target_y = y_max
            else:
                target_y = y_min

            target_p = np.array([current_x, target_y, hover_z])
            err = target_p - p_curr
            dp = k_pos * err

            # Arrived at y end, step X and flip direction
            if np.isclose(y_curr, target_y, atol=0.015):
                print(f"✅ Reached y end: x={current_x:.2f}, y={target_y:.2f}")
                current_x += x_step
                scan_direction *= -1

                # Reset scan when hitting x boundary
                if current_x > x_end:
                    current_x = x_start
                    scan_direction = 1
                    print("🔁 Scan finished, restart cleaning path")

        # DLS IK
        J = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, J, None, site_id)
        J = J[:, :7]
        dq = J.T @ np.linalg.inv(J @ J.T + damping**2 * np.eye(3)) @ dp
        q = q + dq * dt
        q = np.clip(q, model.jnt_range[:7, 0], model.jnt_range[:7, 1])

        data.ctrl[:7] = q
        data.ctrl[7:9] = 0.04

        mujoco.mj_step(model, data)
        viewer.sync()
        print(f"x: {x_curr:.4f} | y: {y_curr:.4f} | z: {z_curr:.4f}")

print("Simulation ended")
