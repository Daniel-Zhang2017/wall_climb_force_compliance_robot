## Issue: ROS 2 Python node receives sensor messages but control timer never executes, no output on command topic

### Symptom

- Node starts normally, log prints `✅ Node initialized, timer created`
- Subscriber callback (e.g. FT wrench sensor) triggers continuously, sensor data is received correctly
- Timer callback (control loop) **never runs**, no control calculation log, no `/cmd_vel` message published
- `ros2 topic info` shows publisher endpoint exists, but no messages can be received by listeners

### Root Cause

`use_sim_time: true` was set in the launch file, but **no simulation clock source (`/clock` topic) was running**.

> 
> In ROS 2: when `use_sim_time = True`, all `rclpy` timers are driven by the `/clock` topic instead of wall-clock time. If nothing publishes `/clock`, timers are suspended indefinitely.
> Subscriber callbacks are still triggered by incoming messages, so sensor reception still works — this creates the confusing "partial working" behaviour.

### Solution

1. For real hardware / wall-clock testing: set `use_sim_time: False` in launch parameters.
```
parameters=[{"use_sim_time": False}]
```
2. If you need simulation time (Gazebo):

```
ros2 topic pub --rate 100 /clock rosgraph_msgs/msg/Clock '{clock: {sec: 0, nanosec: 0}}'
```
   - Keep `use_sim_time: True`
   - Ensure Gazebo is running (Gazebo publishes `/clock` automatically)
   - For standalone debug without Gazebo, start a fake clock publisher:

### Additional related pitfalls encountered during debug (append as notes)

1. **QoS mismatch**: `ros2 topic echo` defaults to `RELIABLE`, sensor nodes often use `BEST_EFFORT` / `qos_profile_sensor_data`. Mismatched QoS profiles lead to invisible failure: endpoints exist but no data transfer.
2. **ROS_DOMAIN_ID & DDS backend**: All terminals must share the same `ROS_DOMAIN_ID` and same RMW implementation (e.g. Cyclone DDS / FastDDS), otherwise discovery fails.
3. **rclpy timer object garbage collection**: Timers must be stored as `self.timer_name`; local variables will be collected and stop firing.