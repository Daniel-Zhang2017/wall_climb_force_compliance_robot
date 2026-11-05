# wall_climb_force_compliance_robot

## Overview

ROS2 Humble demo package: Manipulator constant normal force compliance on planar wall, with bump adaptation & force-based E-Stop. Built on ROS2, Nav2, MoveIt2, Gazebo. Suitable for **facade inspection / cleaning / surface contact task prototype**.

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

<img width="1680" height="1050" alt="Screenshot for surface_follower_node2" src="https://github.com/user-attachments/assets/fdf32f41-e7f4-48b8-8ec2-916b9ccc1cf3" />

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


#### Basic notes:

>1. A **FT (Force/Torque) sensor** in robotics and automation is a multi-axis measurement device—often mounted at a robot's wrist or end-of-arm tooling—that **detects forces and rotational moments (torques)** across six degrees of freedom (F_x, F_y, F_z, M_x, M_y, M_z).

