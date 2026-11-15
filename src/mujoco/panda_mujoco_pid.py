import mujoco
import numpy as np
import os
from mujoco import viewer

# ========== Tuned for YOUR scene_pos.xml ==========
pre_pos_target = np.array([0.60, 0.0, 0.30, 0.0, 0.0, 0.0])
CART_TOL = 0.03
K_CART = 0.4
MAX_CART_VEL = 0.05
DT = 0.002

TARGET_FZ = -12.0
KP_F = 0.05
KI_F = 0.01
KD_F = 0.002
INTEGRAL_LIMIT = 2.0
SCAN_VEL_X = 0.02
APPROACH_VEL_Z = -0.005
APPROACH_THRESHOLD = -0.3
DAMP = 1.5e-2
MAX_POS_STEP = 0.015

# Franka joint limits q1~q7
JOINT_LIMITS = np.array([
    [-2.8973, 2.8973],
    [-1.7628, 1.7628],
    [-2.8973, 2.8973],
    [-3.0718, -0.0698],
    [-2.8973, 2.8973],
    [-0.0175, 3.7525],
    [-2.8973, 2.8973]
])

force_integral = 0.0
last_f_error = 0.0
phase = "pre_pos"
SIM_TIME = 80.0

# ========== Load scene ==========
xml_dir = "../franka_emika_panda"
xml_filename = "scene_pos.xml"
xml_path = os.path.join(xml_dir, xml_filename)

with open(xml_path, "r") as f:
    xml_str = f.read()

old_cwd = os.getcwd()
os.chdir(xml_dir)
try:
    model = mujoco.MjModel.from_xml_string(xml_str)
finally:
    os.chdir(old_cwd)

data = mujoco.MjData(model)

ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_center_site")
key_home = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
print("ee_site_id =", ee_site_id)

mujoco.mj_resetDataKeyframe(model, data, key_home)
mujoco.mj_forward(model, data)
data.ctrl[:7] = data.qpos[:7].copy()

log_fz = []
log_t = []
t = 0.0
err_stuck_timer = 0.0

# ========== Helper Functions ==========
def get_ee_fz(model, data):
    fz = 0.0
    for i, con in enumerate(data.contact):
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, force)
        fz += force[2]
    return fz

def damped_pinv(J, lam):
    return J.T @ np.linalg.inv(J @ J.T + lam * np.eye(J.shape[0]))

# ========== Main Control Loop ==========
with viewer.launch_passive(model, data) as v:
    v.cam.distance = 3.2
    v.cam.elevation = -20
    v.cam.azimuth = 120
    v.cam.lookat[:] = [0.4, 0.0, 0.3]

    while t < SIM_TIME and v.is_running():
        fz = get_ee_fz(model, data)
        ee_pos = data.site_xpos[ee_site_id].copy()
        q = data.qpos[:7]

        jac = np.zeros((6, model.nv))
        mujoco.mj_jacSite(model, data, jac[:3], jac[3:], ee_site_id)
        J = jac[:, :7]

        if phase == "pre_pos":
            # if fz > 4.0:
            #     print(f"[SAFETY] pre_pos unexpected contact Fz={fz:.2f}, hold")
            #     dqdot = np.zeros(7)
            # else:
            err = pre_pos_target[:3] - ee_pos
            err_norm = np.linalg.norm(err)
            v_cart = np.zeros(6)
            v_cart[:3] = K_CART * err
            v_norm = np.linalg.norm(v_cart[:3])
            if v_norm > MAX_CART_VEL:
                v_cart[:3] *= MAX_CART_VEL / v_norm
                dqdot = damped_pinv(J, DAMP) @ v_cart

            q_des = q + dqdot * DT
            q_des = np.clip(q_des, q - MAX_POS_STEP, q + MAX_POS_STEP)
            for j in range(7):
                q_des[j] = np.clip(q_des[j], JOINT_LIMITS[j,0], JOINT_LIMITS[j,1])
            data.ctrl[:7] = q_des

            err = pre_pos_target[:3] - ee_pos
            err_norm = np.linalg.norm(err)
            print(f"t={t:.2f}, pre_pos, ee={ee_pos.round(3)}, err={err_norm:.3f}, Fz={fz:.2f}")
            # Stuck detection
            if err_norm > 0.12:
                err_stuck_timer += DT
                if err_stuck_timer > 4.0:
                    print(f"⚠️ WARNING: IK stuck, current q={q.round(3)}")
                    err_stuck_timer = 0.0
            else:
                err_stuck_timer = 0.0

            if err_norm < CART_TOL:
                phase = "approach"
                print("==== Arrived above red table, start vertical approach ====")

        elif phase == "approach":
            v_cart = np.array([0.0, 0.0, APPROACH_VEL_Z, 0.0, 0.0, 0.0])
            dqdot = damped_pinv(J, DAMP) @ v_cart
            q_des = q + dqdot * DT
            q_des = np.clip(q_des, q - MAX_POS_STEP, q + MAX_POS_STEP)
            for j in range(7):
                q_des[j] = np.clip(q_des[j], JOINT_LIMITS[j, 0], JOINT_LIMITS[j, 1])
            data.ctrl[:7] = q_des

            print(f"t={t:.2f}, approach, ee={ee_pos.round(3)}, Fz={fz:.2f}")
            if fz < APPROACH_THRESHOLD:
                phase = "force_scan"
                print("==== Detected contact with red table, start force-controlled scan ====")
                force_integral = 0.0
                last_f_error = 0.0

        else: # force scan
            f_err = TARGET_FZ - fz
            force_integral += f_err * DT
            force_integral = np.clip(force_integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
            f_deriv = (f_err - last_f_error) / DT
            vz = KP_F * f_err + KI_F * force_integral + KD_F * f_deriv
            vz = np.clip(vz, -MAX_CART_VEL, MAX_CART_VEL)
            v_cart = np.array([SCAN_VEL_X, 0.0, vz, 0.0, 0.0, 0.0])

            dqdot = damped_pinv(J, DAMP) @ v_cart
            q_des = q + dqdot * DT
            q_des = np.clip(q_des, q - MAX_POS_STEP, q + MAX_POS_STEP)
            for j in range(7):
                q_des[j] = np.clip(q_des[j], JOINT_LIMITS[j, 0], JOINT_LIMITS[j, 1])
            data.ctrl[:7] = q_des
            last_f_error = f_err

            print(f"t={t:.2f}, scan, ee={ee_pos.round(3)}, Fz={fz:.2f}")

        mujoco.mj_step(model, data)
        t += DT
        log_fz.append(fz)
        log_t.append(t)

# Plot
import matplotlib.pyplot as plt
plt.figure(figsize=(9, 4))
plt.plot(log_t, log_fz, label="Measured Fz")
plt.axhline(TARGET_FZ, color="red", linestyle="--", label="Target Fz")
plt.axhline(APPROACH_THRESHOLD, color="gray", linestyle=":", label="Contact trigger")
plt.xlabel("Time (s)")
plt.ylabel("Fz (N)")
plt.title("Panda: Direct Cartesian IK, No Joint Waypoint")
plt.legend()
plt.grid(True)
plt.show()
