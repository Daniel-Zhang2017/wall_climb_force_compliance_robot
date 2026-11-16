import os
import mujoco
import mujoco.viewer
import numpy as np
import cv2

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

# IK parameters
damping = 0.06
dt = model.opt.timestep
tcp_speed = 0.55   # m/s, XY tracing speed (0.8 is extremely fast, may cause instability)
k_z = 2.0          # gain for holding constant Z height
plane_z = 0.25     # fixed height / distance above table

# Motion sequence
mujoco.mj_forward(model, data)
p_start = data.site_xpos[site_id].copy()
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

# --------------------------
# Save frames as PNG instead of cv2 mp4
# --------------------------
width, height = 1280, 720
renderer = mujoco.Renderer(model, width=width, height=height)
frame_dir = "frames"
os.makedirs(frame_dir, exist_ok=True)
frame_idx = 0

def record_frame():
    global frame_idx
    renderer.update_scene(data)
    img = renderer.render()
    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    cv2.imwrite(os.path.join(frame_dir, f"frame_{frame_idx:06d}.png"), img_bgr)
    frame_idx += 1

# Main simulation loop
try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_forward(model, data)
            p_curr = data.site_xpos[site_id].copy()

            if stage == 1:
                p_des = p_down_target
                if np.linalg.norm(p_curr - p_down_target) < 0.015:
                    stage = 2
                    print("Reached horizontal plane, start XY movement (constant height)")
            else:
                xy_des = waypoints_xy[waypoint_idx]
                p_des = np.array([xy_des[0], xy_des[1], plane_z])
                if np.linalg.norm(p_curr[:2] - xy_des) < 0.015:
                    waypoint_idx += 1
                    if waypoint_idx >= len(waypoints_xy):
                        print("Finished rectangle loop, restart waypoints")
                        waypoint_idx = 0

            # --------------------------
            # Decoupled XY constant speed + Z height regulation
            # Ensures fixed distance from table
            # --------------------------
            xy_err = p_des[:2] - p_curr[:2]
            xy_err_norm = np.linalg.norm(xy_err)
            if xy_err_norm > 1e-6:
                xy_vel = xy_err / xy_err_norm * tcp_speed
            else:
                xy_vel = np.zeros(2)

            z_err = plane_z - p_curr[2]
            z_vel = k_z * z_err

            v_des = np.array([xy_vel[0], xy_vel[1], z_vel])

            # Damped least squares IK
            J = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, J, None, site_id)
            J = J[:, :7]

            dq = J.T @ np.linalg.inv(J @ J.T + damping**2 * np.eye(3)) @ v_des
            q = q + dq * dt
            q = np.clip(q, model.jnt_range[:7, 0], model.jnt_range[:7, 1])

            data.ctrl[:7] = q
            data.ctrl[7:9] = 0.04  # gripper open

            mujoco.mj_step(model, data)
            viewer.sync()
            record_frame()

except KeyboardInterrupt:
    print("\nInterrupted")
finally:
    renderer.close()
    print(f"Frames saved to {frame_dir}/")
    print("Now run ffmpeg command below:")
    print('ffmpeg -framerate 30 -i frames/frame_%06d.png -c:v libx264 -crf 24 -preset medium -pix_fmt yuv420p -movflags +faststart panda_motion_github.mp4')
