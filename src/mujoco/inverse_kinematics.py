"""Inverse Kinematics using MuJoCo's Jacobian — modern example."""

import numpy as np
import mujoco
import mujoco.viewer

XML = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="2 2 0.1" rgba=".9 .9 .9 1"/>

    <!-- Target site (visual only) -->
    <site name="target" pos="0.3 0.2 0.5" size="0.02" rgba="1 0 0 0.5"/>

    <!-- 3-link planar arm -->
    <body name="base" pos="0 0 0.5">
      <joint name="j1" type="hinge" axis="0 0 1"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" rgba="0.2 0.6 0.8 1"/>
      <body name="link2" pos="0.3 0 0">
        <joint name="j2" type="hinge" axis="0 0 1"/>
        <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" rgba="0.3 0.7 0.3 1"/>
        <body name="link3" pos="0.3 0 0">
          <joint name="j3" type="hinge" axis="0 0 1"/>
          <geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.015" rgba="0.8 0.4 0.1 1"/>
          <site name="end_effector" pos="0.2 0 0" size="0.015" rgba="0 0 1 1"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="j1" ctrlrange="-5 5"/>
    <motor joint="j2" ctrlrange="-5 5"/>
    <motor joint="j3" ctrlrange="-5 5"/>
  </actuator>
</mujoco>
"""


def ik_step(model, data, target_pos, site_name="end_effector", step_size=0.5, damping=1e-4):
    """One step of damped least-squares IK."""
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    current_pos = data.site_xpos[site_id]

    # Compute error
    error = target_pos - current_pos

    # Compute Jacobian
    jacp = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, None, site_id)

    # Damped least-squares: dq = J^T (J J^T + λI)^-1 * error
    jjt = jacp @ jacp.T + damping * np.eye(3)
    dq = jacp.T @ np.linalg.solve(jjt, error)

    return step_size * dq


def main():
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)

    # Get target position
    target_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "target")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and data.time < 1000.0:
            mujoco.mj_forward(model, data)
            target_pos = data.site_xpos[target_id]

            # IK control
            dq = ik_step(model, data, target_pos)
            data.ctrl[:] = 50.0 * dq - 10.0 * data.qvel[:]

            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()