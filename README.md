# wall_climb_force_compliance_robot

## Overview

ROS2 Humble demo package: Manipulator constant normal **force compliance (force control)** on planar wall, with bump adaptation & force-based E-Stop. Built on ROS2, Nav2, MoveIt2, Gazebo. **Control Algorithm Comparison (Wall-Following Force Control)**: P, PID, Python-based CVXPY MPC, Acados MPC. Suitable for **facade inspection / cleaning / surface contact task prototype**.

### Application scenario: Facade / wall surface cleaning
This surface follower is designed for end-effector cleaning tasks on building facades, glass walls and panel surfaces.
The cleaning pad needs consistent contact pressure against the wall while sweeping.
Surface normal from point cloud aligns the cleaning disk perpendicular to the local wall, and decouples motion into normal and tangential directions.
Force compliance on the normal direction maintains steady pressing force for cleaning; the end-effector yields automatically over seams and small bumps to avoid scratching the surface or jamming.
The auto bounce sweep implements back-and-forth wiping motion.
Note: This demo focuses on end-effector contact compliance only; it does not include vacuum adhesion for wall climbing or full area coverage planner.

> Force-controlled surface scanning / surface follower, using FT wrench feedback + point-cloud surface normal estimation.
> Visualizable target plane in RViz2, with dummy test nodes for rapid validation.

## 📌 Project Overview

This ROS2 package implements a **cartesian force-controlled surface follower** for Franka Panda arm:

- Use FT sensor wrench to maintain constant contact force against a surface
- Use point cloud (`/depth/points`) to estimate surface normal (SVD plane fitting)
- Two velocity components:
  - Normal direction: proportional force feedback (keep target pressing force)
  - Tangent direction: back-and-forth sweep scan along surface
- Command velocity via **MoveIt2 Servo** (`delta_twist_cmds`)
- RViz visualization: transparent reference plane marker + normal/tangent arrow markers
- Dummy test nodes: synthetic wrench and synthetic point cloud for simulation testing without real hardware

Tested on: **ROS 2 Humble, Ubuntu 22.04, Franka Panda, MoveIt2 Servo, ros2_control**

## demo videos

**Video 1: surface-follower-servo-horizontal-moving**

https://github.com/user-attachments/assets/609c732a-f2f8-4e91-b309-5a31a628de95

# Control Algorithm Comparison (Wall-Following Force Control)

This table compares four control strategies implemented in this surface-following manipulator project, targeting **facade cleaning and compliant surface contact tasks**.


| Controller | Core Principle | Pros | Cons | Use Case |
| --- | --- | --- | --- | --- |
| **Proportional (P) Control** | Adjust normal velocity directly by force error | Extremely simple, zero integral drift, low computation cost, easy tuning | Static force error exists; no oscillation damping; no explicit constraint handling | Simulation verification, rapid prototyping, basic surface-following validation |
| **PID Control** | Proportional + integral + derivative feedback | Eliminates steady-state error; widely adopted in industry; lightweight and robust | Prone to integral windup; hard-clipped constraints only; no predictive ability; sensitive to varying surface stiffness | Fast real-world deployment, low-frequency force control, general robotic compliance tasks |
| **CVXPY MPC (Python)** | Finite-horizon convex optimization | Native constraint support; predictive behaviour; easy model modification in Python | High solver latency; unstable for high-frequency loops; not real-time capable | Algorithm comparison, simulation study, MPC logic validation |
| **Acados MPC** | Code-generated real-time SQP-RTI optimization | Microsecond-level solving speed; hard/soft constraint support; stable high-frequency closed loop; compatible with real manipulators | Requires compilation environment; model changes require code regeneration | Final physical robot deployment, high-precision compliant cleaning, safety-critical surface contact |

## Summary

For wall-cleaning robotic tasks:

- PID provides reliable and fast deployment.
- Python-based CVXPY MPC is suitable for algorithm research and comparison.
- **Acados MPC is the final deployed solution**, offering predictive control and strict velocity constraints to avoid collision, improve contact stability, and adapt to uneven building facades.
  
## Summary for surface_follower_node2.py and surface_follower2.launch.py
This package implements a **geometry + force-based surface follower** for manipulator end-effector, designed for wall-climbing / surface contact tasks (e.g. facade inspection, polishing, cleaning).

✅ Implemented features
- Plane fitting & surface normal estimation from point cloud (SVD)
- Normal-direction force compliance control: maintain constant contact force against the surface
- Tangential surface sweep along the surface plane
- Auto bounce scanning: move back and forth within a travel bound
- RViz visualization markers: cyan arrow = surface normal, yellow arrow = sweep tangent direction
- Output twist velocity command for low-level motion control
```
ros2 topic pub --rate 10 /ft_sensor/wrench geometry_msgs/msg/WrenchStamped '{header: {frame_id: ft_sensor_link}, wrench: {force: {x: 0.0, y: 0.0, z: -14.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}}'
```
```
#new terminator
cd src/surface_follower

python3 dummy_pc.py #publish point cloud data
```
```
#new terminator
ros2 launch surface_follower surface_follower2.launch.py
```

**demo image**

<img width="1173" height="603" alt="569178cf52f71d98a850b3d1f4d68ea0" src="https://github.com/user-attachments/assets/80479ccf-4f81-4f7b-a914-d7f04c752e71" />

📥 Input topics
- `/depth/points` (PointCloud2): point cloud for plane fitting
- `/ft_sensor/wrench` (WrenchStamped): force-torque sensor measurement

📤 Output topics
- `/cmd_vel` (Twist): end-effector velocity command (for demo)

⚠️ Current limitations
- Tangential travel is open-loop integrated, will drift over time
- Only supports single-line raster scan (1D bounce), not full area coverage
- No frame transform / safety E-stop for real manipulator yet
- Plane fitting assumes a locally flat surface

本功能包(surface_follower_node2.py and surface_follower2.launch.py
)实现**基于点云几何 + 力反馈的机械臂表面跟随器**，面向幕墙作业、表面打磨、巡检等贴壁作业场景。

✅ 已实现功能
- 点云SVD平面拟合，估计局部曲面法向
- 法向力柔顺控制：维持末端与墙面恒定接触力
- 沿表面切向横向扫掠
- 自动往返反弹扫描：在设定行程内来回移动
- RViz可视化标记：青色箭头 = 曲面法向，黄色箭头 = 扫掠切线方向
- 输出Twist速度指令，用于运动控制

📥 输入话题
- `/depth/points` (PointCloud2)：用于平面拟合的深度点云
- `/ft_sensor/wrench` (WrenchStamped)：力传感器数据

📤 输出话题
- `/cmd_vel` (Twist)：末端速度指令（当前Demo版本）

⚠️ 当前局限
- 扫掠位移为开环积分，长时间运行存在漂移
- 仅支持单条直线往返扫描，尚不支持整片区域全覆盖
- 还未接入MoveIt Servo、坐标系变换、机械臂力超限急停
- 平面拟合假设局部区域近似平面

## surface-follower-servo-control

main code: surface_follower_node3.py and surface_follower_servo.launch.py

```
colcon build
source install/setup.bash

ros2 launch panda_sim_bringup panda_fake_control.launch.py
```

```
#new terminator
ros2 launch surface_follower surface_follower_servo.launch.py
```
```
ros2 run surface_follower dummy_pc

ros2 run surface_follower dummy_wrench --ros-args -p fz:=-15.0
```

```
#new terminator, start the servo
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"
```

### Open RViz2

Add these displays:

- `TF`
- `Marker` → topic: `/surface_markers`
- `PointCloud2` → topic: `/depth/points`
- `RobotModel` (Panda URDF)

You will see:

- Green semi-transparent plane at `(0.3, 0, 0.55)`
- Point cloud lying exactly on the plane
- Cyan arrow: estimated surface normal
- Yellow arrow: sweep tangent direction
- Arm sweeps back and forth while maintaining constant contact force

---

## 🧪 Debug Mode

Set `DEBUG_FORCE_MOVE: true` in `surface_follower.yaml`

- Force control is disabled
- Node sends fixed `test_vel_x / test_vel_z` velocity
- Useful to verify Servo movement independently of force feedback

---

## 🛠️ How it works

1. **Force feedback**: Compute error between measured Fz and target force → generate velocity along surface normal
2. **Surface normal estimation**: SVD on point cloud covariance to fit a plane and extract normal vector
3. **Sweep generation**: Gram-Schmidt orthogonalization to get tangent direction on surface
4. **Twist transform**: Convert velocity command from FT sensor frame → robot base frame (`panda_link0`)
5. **MoveIt Servo**: Execute Cartesian velocity commands in real-time

> 
> The control law is a simple proportional force controller, suitable for slow surface scanning. For production use, you may add low-pass filtering on wrench and point cloud.

### MoveIt Servo Loop Control
Launching `servo_node_main` only loads the node, parameters, collision checker, planning scene monitor and subscribers. **It does NOT start the real-time servo update loop**. Until you call this Trigger service, Servo ignores all `TwistStamped` / `JointJog` / `PoseStamped` jog commands and sends zero commands to the robot

**Why it is designed this way?**

>1. **Safety (the main reason)**
Servo is for high-frequency velocity jogging (teleop, visual servoing, surface following like your Panda force-control scan). If it started automatically on launch, any stale / misconfigured / accidentally published Twist message would immediately move the arm.
The service gives you a manual arming step: you verify TF, planning scene, collision model, robot state are all good before enabling motion.
>2. **Separate initialization from activation**
   >- After `ros2 launch ... servo.launch.py`: node is alive, parameters parsed, PSM running, status topic publishing.
   >- After `start_servo`: the real-time servo thread begins reading jog topics, solving IK, checking singularities + self-collision, and outputting joint velocity commands to `joint_trajectory_controller` / `forward_velocity_controller`MoveIt 2.
>3. **Paired lifecycle controls**
   
> 
> Target: ROS 2 Humble, `moveit_servo == 2.5.10` (Panda Cartesian jogging / surface-follower use case)

This section documents how to safely control the MoveIt Servo real-time servo loop.
The servo node is **not active immediately after launch**; you must explicitly arm it via service calls before it will process `TwistStamped` jog commands.

> 
> ⚠️ Important version quirk:
> `/servo_node/pause_servo` is overloaded with both `std_srvs/srv/SetBool` and `std_srvs/srv/Trigger`.
> **Do NOT use the `SetBool` interface**, it can cause service waiting/hanging. Always use the `Trigger` services below.

#### Service Table

| Operation | Command | Behavior | Recovery |
| --- | --- | --- | --- |
| Start / Arm servo loop | `ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"` | Enable IK, collision checking, velocity output; listen to `/servo_node/delta_twist_cmds` | N/A |
| Pause servo | `ros2 service call /servo_node/pause_servo std_srvs/srv/Trigger "{}"` | Freeze motion, keep servo node, TF and collision monitor alive | Call `unpause_servo` |
| Resume / Unpause | `ros2 service call /servo_node/unpause_servo std_srvs/srv/Trigger "{}"` | Resume processing twist commands from running/paused state | N/A |
| Full stop servo loop | `ros2 service call /servo_node/stop_servo std_srvs/srv/Trigger "{}"` | Disable servo computation, return to idle state | Must call `start_servo` again to re-arm |

## Monitor Servo Status

Inspect runtime state: paused flag, collision, singularity, stale state warnings:

```
ros2 topic echo /servo_node/status
```

#### Basic notes:

>1. A **FT (Force/Torque) sensor** in robotics and automation is a multi-axis measurement device—often mounted at a robot's wrist or end-of-arm tooling—that **detects forces and rotational moments (torques)** across six degrees of freedom (F_x, F_y, F_z, M_x, M_y, M_z).
>2. **Point cloud source**: default is RGB-D depth camera, viewing the wall approximately along surface normal.The RGB-D camera is eye-in-hand, mounted on the manipulator **end-effector together with the F/T sensor**.
It faces approximately normal to the wall to capture local surface point cloud for plane fitting.
>3. This implementation is NOT designed for a 2D LiDAR out of the box.
A 2D LiDAR (scan plane parallel to wall) requires accumulating sequential scans to reconstruct local surface for normal estimation.
>4. Surface normal estimation is implemented via **SVD plane fitting**.
Mathematically this is **equivalent to PCA**: the smallest singular/ eigen vector gives the normal of the locally fitted plane.
**SVD is chosen here for better numerical stability over covariance-based PCA**.
>5. [rviz2]: **Stereo is NOT SUPPORTED is just an INFO log**, not an error. You can safely ignore it! What it means: Your graphics card or OpenGL setup does not support RViz2's stereoscopic 3D split-screen rendering (VR / dual-display mode). It will not affect normal functionality at all: displaying point clouds, images, TF, lidar, cameras, stereo cameras, and disparity maps will all work perfectly fine.
>6. - MoveIt2 → 输出轨迹 → 交给 **`joint_trajectory_controller`**（ros2_control）闭环执行
>    - **MoveIt Servo** → 持续末端速度指令 → **`forward_command_controller`**（ros2_control）下发
>    - Servo 报错、关节不动、TF 不更新，**90% 是 ros2_control 链路问题**：
>    - controller 没激活、interface 不匹配
>    - command_interface 名字不一致（position /velocity 搞混）
>    - use_sim_time 不同步、QoS、TF 时间戳冲突
>7. **Control Algorithm Comparison (Wall-Following Force Control)**: Summary For wall-cleaning robotic tasks:
>    - PID provides reliable and fast deployment.
>    - Python-based CVXPY MPC is suitable for algorithm research and comparison.
>    - **Acados MPC is the final deployed solution**, offering predictive control and strict velocity constraints to avoid collision, improve contact stability, and adapt to uneven building facades.


