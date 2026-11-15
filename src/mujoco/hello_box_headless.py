import mujoco

# 从 MJCF/XML 文件编译出静态模型。
# model 里保存的是结构和参数，比如 body、joint、geom、质量、时间步等。
model = mujoco.MjModel.from_xml_path("hello.xml")

# 创建和 model 对应的运行时状态。
# data 里保存的是会随仿真变化的量，比如时间、位置、速度、接触和中间计算结果。
data = mujoco.MjData(model)

# 每次 mj_step 会按 model.opt.timestep 推进一个仿真步。
# hello.xml 里 timestep=0.002，所以这里会运行大约 1000 步，直到仿真时间到 2 秒。
while data.time < 2.0:
    mujoco.mj_step(model, data)

print("sim time:", data.time)

# 按 body 名称读取 box 的世界坐标位置。
# xpos 是 MuJoCo 在前向计算后写入 data 的结果；copy() 用来拷贝一份当前数值，避免后续仿真覆盖。
print("box position:", data.body("box").xpos.copy())