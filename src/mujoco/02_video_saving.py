import os
import mujoco
import mujoco.viewer
import numpy as np
import subprocess

xml_dir = "../franka_emika_panda"
xml_filename = "scene_pos.xml"
xml_path = os.path.join(xml_dir, xml_filename)

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

WIDTH, HEIGHT = 1280, 720
RENDER_FPS = 30
renderer = mujoco.Renderer(model, WIDTH, HEIGHT)

# ffmpeg pipe: raw RGB → mp4
ffmpeg_cmd = [
    "ffmpeg",
    "-y",
    "-f", "rawvideo",
    "-pixel_format", "rgb24",
    "-video_size", f"{WIDTH}x{HEIGHT}",
    "-framerate", str(RENDER_FPS),
    "-i", "-",
    "-c:v", "libx264",
    "-preset", "fast",
    "-pix_fmt", "yuv420p",
    "panda_plane.mp4"
]
ffmpeg_proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)

site_id = model.site("attachment_site").id
q = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
data.qpos[:7] = q
mujoco.mj_forward(model, data)

k = 2.5
damping = 0.06
dt = model.opt.timestep
plane_z = 0.25

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
frame_interval = 1.0 / RENDER_FPS
next_render_time = 0.0

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_forward(model, data)
        p_curr = data.site_xpos[site_id].copy()

        if stage == 1:
            p_des = p_down_target
            if np.linalg.norm(p_curr - p_down_target) < 0.015:
                stage = 2
                print("Reached horizontal plane, start XY movement")
        else:
            xy_des = waypoints_xy[waypoint_idx]
            p_des = np.array([xy_des[0], xy_des[1], plane_z])
            if np.linalg.norm(p_curr[:2] - xy_des) < 0.015:
                waypoint_idx += 1
                if waypoint_idx >= len(waypoints_xy):
                    print("Finished rectangle trajectory")
                    break  # exit while loop

        J = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, J, None, site_id)
        J = J[:, :7]
        dp = p_des - p_curr
        dq = J.T @ np.linalg.inv(J @ J.T + damping**2 * np.eye(3)) @ (k * dp)
        q = q + dq * dt
        q = np.clip(q, model.jnt_range[:7, 0], model.jnt_range[:7, 1])

        data.ctrl[:7] = q
        data.ctrl[7:9] = 0.04

        mujoco.mj_step(model, data)
        viewer.sync()

        # Render only at target FPS (not every physics step)
        if data.time >= next_render_time:
            renderer.update_scene(data)
            img = renderer.render()
            try:
                ffmpeg_proc.stdin.write(img.tobytes())
            except BrokenPipeError:
                print("ffmpeg pipe broken")
                break
            next_render_time += frame_interval

# Cleanup ffmpeg
ffmpeg_proc.stdin.close()
ffmpeg_proc.wait()
print("Saved panda_plane.mp4")
