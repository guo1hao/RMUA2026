# RMUA2026 ROS1 Workspace

这是一个基于 ROS Noetic 的 RMUA2026 本地开发工作空间，当前包含四个基础包：

- `airsim_ros`：与模拟器精确匹配的最小消息定义，目前包含 `RotorPWM`。
- `rmua_msgs`：比赛接口相关的自定义消息。
- `rmua_sensor_hub`：把模拟器传出的全部比赛传感器数据统一转发到项目命名空间下，供其他节点直接订阅。
- `rmua_flight_control`：基于 PWM 的基础飞控节点，直接向模拟器发送电机指令。

## 构建

```bash
source /opt/ros/noetic/setup.bash
cd /home/hao/workspace-RosNoetic/rmua2026
catkin_make
```

## 一键启动

默认模拟器目录：`/home/hao/robomaster/RMUA/2026/simulator_12.0.0.5`

```bash
cd /home/hao/workspace-RosNoetic/rmua2026
./scripts/start_stack.sh 123
```

如果要使用后台渲染模式：

```bash
cd /home/hao/workspace-RosNoetic/rmua2026
./scripts/start_stack_offscreen.sh 123
```

如果要直接启动基础 PWM 悬停飞控：

```bash
cd /home/hao/workspace-RosNoetic/rmua2026
./scripts/start_pwm_stack.sh 123 1.0
```

后台模式：

```bash
cd /home/hao/workspace-RosNoetic/rmua2026
./scripts/start_pwm_stack_offscreen.sh 123 1.0
```

如果要启动官方案例的状态估计与 PD 低层对照链路，并在同一路径任务上对比：

```bash
cd /home/hao/workspace-RosNoetic/rmua2026
./scripts/start_official_path_mission.sh 123 69
```

后台模式：

```bash
cd /home/hao/workspace-RosNoetic/rmua2026
./scripts/start_official_path_mission_offscreen.sh 123 69
```

PWM 栈默认以待机模式启动。等模拟器稳定、传感器开始正常发布后，再执行：

```bash
cd /home/hao/workspace-RosNoetic/rmua2026
./scripts/enable_pwm_controller.sh
```

如需退出控制：

```bash
cd /home/hao/workspace-RosNoetic/rmua2026
./scripts/disable_pwm_controller.sh
```

如果模拟器路径变化，可以通过环境变量覆盖：

```bash
export RMUA_SIM_ROOT=/path/to/simulator_12.0.0.5
./scripts/start_stack.sh 123
```

## 传感器转发主题

传感器会被转发到 `/rmua/sensors/drone_1/*` 命名空间下，例如：

- `/rmua/sensors/drone_1/front_left/scene`
- `/rmua/sensors/drone_1/front_right/scene`
- `/rmua/sensors/drone_1/back_left/scene`
- `/rmua/sensors/drone_1/back_right/scene`
- `/rmua/sensors/drone_1/imu`
- `/rmua/sensors/drone_1/lidar`
- `/rmua/sensors/drone_1/pose_gt`
- `/rmua/sensors/drone_1/gps`
- `/rmua/sensors/drone_1/debug/wind`
- `/rmua/sensors/drone_1/debug/rotor_pwm`
- `/rmua/sensors/drone_1/initial_pose`
- `/rmua/sensors/drone_1/end_goal`
- `/rmua/sensors/drone_1/status`

其中 `/rmua/sensors/drone_1/status` 会给出各传感器是否已经收到数据，以及最近一次收到数据的时间。

## PWM 飞控接口

基础 PWM 飞控节点默认读取：

- `/rmua/sensors/drone_1/pose_gt`
- `/rmua/sensors/drone_1/imu`
- `/rmua/sensors/drone_1/initial_pose`

默认发布：

- `/airsim_node/drone_1/rotor_pwm_cmd`
- `/rmua/control/basic/state`
- `/rmua/control/basic/target_pose`
- `/rmua/control/basic/pwm_cmd`

默认命令接口：

- `/rmua/control/basic/enable`：`std_msgs/Bool`，`true` 开启控制。
- `/rmua/control/basic/pose_setpoint`：`geometry_msgs/PoseStamped`，世界系 NED 目标点与目标偏航。

如果不发外部目标点，控制器会在启用后抓取当前稳定的机体位姿，并把默认悬停目标设置为该位置上方 `hover_altitude_m` 米。

## 官方对照链路

为了和官方案例做同场景对照，当前工作区额外提供一套官方风格链路：

- `official_eskf_node`：移植自官方 `imu_gps_odometry`，输入 `/rmua/sensors/drone_1/gps`、`/rmua/sensors/drone_1/imu`、`/rmua/sensors/drone_1/initial_pose`，输出 `/rmua/control/official/eskf_odom` 与 `/rmua/control/official/eskf_pose`。
- `official_pwm_controller_node`：保留官方 PD 控制律与混控方式，但去掉示例工程里写死目标点和演示限幅逻辑，改为订阅项目统一的 `/rmua/control/basic/pose_setpoint`。
- `official_pwm_path_mission.launch`：使用官方估计位姿驱动现有路径任务，实现同一任务层下的低层 A/B 对照。
