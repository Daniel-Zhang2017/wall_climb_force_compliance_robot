import mujoco
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
scene_path = os.path.join(script_dir, "../franka_emika_panda/scene.xml")
print("Trying to load:", scene_path)
model = mujoco.MjModel.from_xml_path(scene_path)
print("✅ XML loaded successfully")
