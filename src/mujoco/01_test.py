import os
import mujoco
import mujoco.viewer
import numpy as np

xml_dir = "../franka_emika_panda"
xml_filename = "scene_pos.xml"
xml_path = os.path.join(xml_dir, xml_filename)

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

site_id = model.site("ee_center_site").id

# Home joint
q = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
data.qpos[:7] = q
mujoco.mj_forward(model, data)

# Gains for IK
k = 2.5
damping = 0.06
dt = model.opt.timestep

# --------------------------
# Motion sequence
# Stage 1: move DOWN to target Z (horizontal plane height)
# Stage 2: move X-Y in that fixed Z plane
# --------------------------
# First get current EE pos
mujoco.mj_forward(model, data)
p_start = data.site_xpos[site_id].copy()
plane_z = 0.25       # <-- This is your horizontal plane height (tune this!)
p_down_target = np.array([p_start[0], p_start[1], plane_z])

# Waypoints on the horizontal plane (Z fixed = plane_z)
waypoints_xy = np.array([
    [0.5,  0.0],
    [0.5,  0.2],
    [0.3,  0.2],
    [0.3,  0.0],
    [0.5,  0.0],
])
waypoint_idx = 0
stage = 1  # 1 = go down, 2 = horizontal XY movement

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_forward(model, data)
        p_curr = data.site_xpos[site_id].copy()

        # Select target
        if stage == 1:
            p_des = p_down_target
            # check if we arrived at plane
            if np.linalg.norm(p_curr - p_down_target) < 0.015:
                stage = 2
                print("Reached horizontal plane, start XY movement")
        else:
            xy_des = waypoints_xy[waypoint_idx]
            p_des = np.array([xy_des[0], xy_des[1], plane_z])
            if np.linalg.norm(p_curr[:2] - xy_des) < 0.015:
                waypoint_idx += 1
                if waypoint_idx >= len(waypoints_xy):
                    waypoint_idx = 0  # loop the rectangle

        # Jacobian IK (3D position)
        J = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, J, None, site_id)
        J = J[:, :7]

        dp = p_des - p_curr
        dq = J.T @ np.linalg.inv(J @ J.T + damping**2 * np.eye(3)) @ (k * dp)

        q = q + dq * dt
        # clamp joint limits
        q = np.clip(q, model.jnt_range[:7, 0], model.jnt_range[:7, 1])

        data.ctrl[:7] = q
        data.ctrl[7:9] = 0.04  # gripper open

        mujoco.mj_step(model, data)
        viewer.sync()
