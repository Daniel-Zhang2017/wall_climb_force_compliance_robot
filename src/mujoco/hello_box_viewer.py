import time

import mujoco
import mujoco.viewer

# 和无渲染版本一样，先编译模型，再创建运行时状态。
model = mujoco.MjModel.from_xml_path("hello.xml")
data = mujoco.MjData(model)

# launch_passive 会打开一个交互 viewer，但物理仿真仍然由当前脚本控制。
# 进入 with 代码块后，viewer 负责显示；离开 with 代码块后，窗口资源会自动释放。
with mujoco.viewer.launch_passive(model, data) as viewer:
    # viewer.is_running() 会在窗口关闭后变成 False。
    # data.time < 10.0 让脚本最多运行 10 秒，避免无限循环。
    while viewer.is_running() and data.time < 10.0:
        # 记录这一轮循环开始的真实时间，用来把仿真速度压到接近实时。
        step_start = time.time()

        # 推进一个物理步；hello.xml 里 timestep=0.002，所以一次前进 2 ms 仿真时间。
        mujoco.mj_step(model, data)

        # 把最新的 data 同步到 viewer。
        # 如果不调用 sync()，窗口可能不会及时显示当前仿真状态。
        viewer.sync()

        # 计算这一轮物理步和渲染同步已经花了多久。
        # 如果还没用完一个 timestep，就 sleep 一下，避免仿真跑得比真实时间快太多。
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)