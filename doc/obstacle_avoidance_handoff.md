# Claude 避障开发交接说明

这份文档用于把当前 RMUA2026 本地工程里和雷达、路径任务、低层控制相关的已知事实交给 Claude，避免它在不了解当前工程约束的前提下直接重写整套控制链。

## 1. 项目背景

- 工作区根目录：/home/hao/workspace-RosNoetic/rmua2026
- ROS 版本：ROS Noetic
- 系统环境：Ubuntu 20.04
- 当前主控链路：任务层 path_mission_node -> 低层 basic_pwm_controller_node -> AirSim PWM 接口
- 当前常用启动命令：

```bash
cd /home/hao/workspace-RosNoetic/rmua2026
./scripts/start_competition_mode.sh
```

- 停止命令：

```bash
cd /home/hao/workspace-RosNoetic/rmua2026
./scripts/start_competition_mode.sh stop
```

## 2. 当前工程里“雷达”实际指什么

当前工程里接入的是 lidar 话题，不是单线 LaserScan。

官方文档里给出的原始话题名是：

- /airsim_node/drone_1/lidar

当前工程通过 sensor_hub 做了一层统一转发，任务和算法层应该优先读 relay 后的话题：

- /rmua/sensors/drone_1/lidar

对应代码位置：

- sensor_hub relay 配置在 src/rmua_sensor_hub/src/sensor_hub_node.cpp
- 启动入口在 src/rmua_sensor_hub/launch/sensor_hub.launch
- 比赛模式传感器栈入口在 src/rmua_sensor_hub/launch/competition_stack.launch

## 3. 当前工程里已经明确的 lidar 参数

下面这些是当前仓库中能直接确认的事实。

### 3.1 话题路径

- 原始 source topic：/airsim_node/drone_1/lidar
- relay 后 topic：/rmua/sensors/drone_1/lidar
- 传感器状态 topic：/rmua/sensors/drone_1/status

### 3.2 relay 启动参数

来自 src/rmua_sensor_hub/launch/sensor_hub.launch 和 src/rmua_sensor_hub/launch/competition_stack.launch：

- drone_name：drone_1
- source_root：/airsim_node
- relay_root：/rmua/sensors
- queue_size：10
- status_publish_rate：2.0 Hz

### 3.3 readiness 监控

SensorHubStatus.msg 中已经包含：

- lidar_ready
- lidar_last_received

所以 Claude 开发避障时，可以直接订阅：

- /rmua/sensors/drone_1/status

在 lidar 尚未 ready 时禁止进入依赖雷达的避障逻辑，避免空数据误判。

### 3.4 官方文档已确认的 MID360 物理参数

你后续给 Claude 的避障开发，应当默认以下物理参数已经有官方依据，不再属于“完全未知”：

- 传感器型号：MID360 激光雷达
- 安装位置：位于机体中心向上 50 mm 处
- 水平视场角：360 度
- 垂直视场角：上方 40 度、下方 19 度
- 有效探测距离：30 m
- 频率：10 Hz
- 单帧点数：20000 点

按图示理解，垂直视场是非对称的，等价于相对水平面大约 +40 度 / -19 度，总垂直覆盖约 59 度。

这部分信息可直接作为避障感知建模的已知输入，例如：

- 前向危险扇区的角度裁剪
- 上下方障碍过滤范围
- 近距刹停和侧绕时的距离阈值初值
- 点云稀疏度与刷新率估计

### 3.5 当前仓库里没有明确给出的 lidar 参数

当前仓库没有把以下信息硬编码进版本库：

- lidar 的消息类型
- 点云 frame_id
- 水平视场角 / 垂直视场角
- 最小量程 / 最大量程
- 点数、扫描频率、线数
- 机体坐标系下的精确安装外参

原因是 sensor_hub 使用 topic_tools/ShapeShifter 做泛型转发，仓库本身不依赖具体 lidar 消息类型。

Claude 不要凭空假设它一定是 LaserScan。当前更合理的做法是：

1. 运行模拟器后，用 rostopic type /rmua/sensors/drone_1/lidar 确认消息类型。
2. 用 rostopic echo -n 1 /rmua/sensors/drone_1/lidar 或 rviz 确认 frame_id 和数据形态。
3. 如需更底层的传感器配置，再去模拟器目录里的 settings.json 或官方 AirSim 配置中核对。

## 4. 当前控制链路

### 4.1 传感器侧

sensor_hub 目前统一 relay 的主要话题有：

- /rmua/sensors/drone_1/pose_gt
- /rmua/sensors/drone_1/imu
- /rmua/sensors/drone_1/lidar
- /rmua/sensors/drone_1/gps
- /rmua/sensors/drone_1/debug/wind
- /rmua/sensors/drone_1/debug/rotor_pwm
- /rmua/sensors/drone_1/initial_pose
- /rmua/sensors/drone_1/end_goal

### 4.2 任务层

核心节点：src/rmua_flight_control/src/path_mission_node.cpp

当前任务层职责：

- 读取 drone_path.csv
- 做路径前视采样
- 管理 pending1 / pending2 切换
- 管理 stable point 69 的驻停与出稳定点切换
- 发布位置姿态 setpoint 到 /rmua/control/basic/pose_setpoint
- 通过 /rmua/control/basic/enable 启动低层控制器

当前比赛模式 launch：src/rmua_flight_control/launch/pwm_path_mission.launch

### 4.3 低层控制器

核心节点：src/rmua_flight_control/src/basic_pwm_controller_node.cpp

当前低层输入：

- pose_topic：/rmua/sensors/drone_1/pose_gt
- imu_topic：/rmua/sensors/drone_1/imu
- pose_setpoint_topic：/rmua/control/basic/pose_setpoint
- enable_topic：/rmua/control/basic/enable

当前低层输出：

- /airsim_node/drone_1/rotor_pwm_cmd

低层当前已经能稳定处理“任务层给出的世界系目标位姿”，Claude 不要把避障直接塞进 PWM 层，优先在任务层之上处理。

## 5. 避障开发建议接入点

### 5.1 推荐方案

推荐 Claude 在“任务层 setpoint”和“低层控制器”之间加一个新的避障节点，而不是把避障硬塞进 basic_pwm_controller_node。

推荐结构：

1. path_mission_node 继续产出原始路径 setpoint。
2. 新增 obstacle_avoidance_node 订阅原始路径 setpoint、pose_gt、imu、lidar。
3. obstacle_avoidance_node 输出修正后的安全 setpoint 给 basic_pwm_controller_node。

这样做的优点：

- 不破坏当前已调好的低层控制器。
- 不把路径任务状态机和障碍感知耦死在一个文件里。
- 避障关闭时可以直接退化回原始路径跟踪。

### 5.2 更具体的 topic 方案

推荐 Claude 做如下改造：

1. 把 path_mission_node 的输出 topic 从当前的 /rmua/control/basic/pose_setpoint 改成一个“原始路径目标”话题，例如 /rmua/control/path/raw_pose_setpoint。
2. 新增 obstacle_avoidance_node：
   - 输入：
     - /rmua/control/path/raw_pose_setpoint
     - /rmua/sensors/drone_1/lidar
     - /rmua/sensors/drone_1/pose_gt
     - /rmua/sensors/drone_1/imu
   - 输出：
     - /rmua/control/basic/pose_setpoint
3. basic_pwm_controller_node 仍然只订阅 /rmua/control/basic/pose_setpoint。

这样最容易回退，也最容易单独调试避障逻辑。

### 5.3 不推荐的方案

不建议 Claude：

- 直接把 lidar 点云解析写进 basic_pwm_controller_node
- 直接在 PWM 力矩层做避障
- 为了避障重写当前 stable point 69 / pending1 / pending2 状态机
- 让避障直接修改 sensor_hub 的 relay 行为

## 6. 当前任务层必须保留的语义

Claude 开发避障时，以下行为必须保留：

1. pending1 到 pending2 的切换点是 stable point 69。
2. 到达 stable point 69 后要 hold 3 秒。
3. hold 期间位置锁在 69，yaw 对准 70。
4. hold 结束后必须继续进入 pending2，而不是停在稳定点附近。
5. 目前 keep_runtime_alive=true，任务失败/完成不应直接结束整栈。

换句话说，避障可以改变飞去目标点的局部轨迹或中间 setpoint，但不能破坏 pending 相位切换和 stable point 的主状态机。

## 7. 当前已知的工程边界和坑

### 7.1 当前没有现成避障模块

仓库里目前没有一个正式接入任务链路的 obstacle avoidance 节点，Claude 需要从零接入。

### 7.2 当前部分参数虽然存在，但实际上没有生效

在 path_mission_node 里，下列 climb_* 参数当前只是读取和裁剪，没有真正参与轨迹采样：

- climb_priority_threshold_m
- climb_horizontal_leash_m
- climb_horizontal_leash_scale
- climb_horizontal_leash_max_m
- climb_vertical_release_distance_m
- climb_vertical_step_m
- climb_preview_waypoints

Claude 不要误以为这些参数已经能直接支持避障。

### 7.3 当前 stable point 69 附近逻辑近期刚修过

近期任务层针对 69 点做过两类修正：

- 靠近稳定点时，前视目标不会再提前跨过 69 去看 70
- hold 结束后会先显式飞向 70，再恢复普通 pending2 lookahead

Claude 开发避障时不要把这一段逻辑回退掉。

## 8. Claude 需要完成的工作

建议 Claude 的交付至少包括以下内容。

### 8.1 感知侧

1. 明确 lidar 的运行时消息类型和 frame_id。
2. 把 lidar 数据转成当前机体可用的障碍表示，例如：
   - 机体前向占据栅格
   - 局部点云障碍簇
   - 前向通道可通行评分

### 8.2 规划 / 决策侧

1. 在原始路径 setpoint 的基础上生成局部绕障目标。
2. 避障目标必须能重新并回主航线，而不是越避越偏。
3. 避障不能破坏 stable point 69 的驻停切相位。
4. 如果障碍消失，应平滑回到原始路径 setpoint。

### 8.3 工程接入侧

1. 新增 obstacle_avoidance_node 或等价模块。
2. 在 launch 中把 path_mission_node、obstacle_avoidance_node、basic_pwm_controller_node 串起来。
3. 给避障节点加最基本的调试日志和参数入口。
4. 让避障可以一键开关，方便回退到纯路径跟踪。

### 8.4 验证侧

1. 无障碍时，飞行效果应接近当前基线。
2. 有障碍时，不能发生撞击。
3. 避障后能回到主航线，不长期漂离。
4. pending1 -> 69 -> pending2 切换不能被破坏。
5. 避障不应导致任务层频繁误判 stalled。

## 9. 建议 Claude 先看的文件

### 9.1 传感器与话题

- src/rmua_sensor_hub/src/sensor_hub_node.cpp
- src/rmua_sensor_hub/launch/sensor_hub.launch
- src/rmua_sensor_hub/launch/competition_stack.launch
- src/rmua_msgs/msg/SensorHubStatus.msg

### 9.2 任务层与控制链

- src/rmua_flight_control/src/path_mission_node.cpp
- src/rmua_flight_control/src/basic_pwm_controller_node.cpp
- src/rmua_flight_control/launch/pwm_path_mission.launch
- src/rmua_flight_control/launch/basic_pwm_controller.launch

### 9.3 文档与运行说明

- doc/README.md
- doc/parameter_tuning_and_startup.md

## 10. 建议 Claude 在文档里先回答的几个问题

在真正开始写代码前，建议 Claude 先用一段简短设计说明回答下面这些问题：

1. lidar 的运行时消息类型到底是什么？
2. 避障节点放在哪一层，为什么？
3. 避障输出是直接改路径 waypoint、改任务 setpoint，还是生成一个中间安全 setpoint？
4. 如何保证 69 点 stable hold 和 pending2 切换不被破坏？
5. 如何定义“绕开障碍后回主航线”的评分或策略？

## 11. 给 Claude 的直接要求

如果把这份文档直接交给 Claude，可以直接附上下面这段要求：

> 请基于当前 RMUA2026 ROS Noetic 工程开发 lidar 避障模块。优先保持现有低层 PWM 控制器不变，在 path_mission_node 与 basic_pwm_controller_node 之间插入避障层。请先确认 /rmua/sensors/drone_1/lidar 的运行时消息类型、frame_id 和数据形态，再设计局部避障逻辑。必须保留 stable point 69 的驻停和 pending1/pending2 切换语义，且避障后能回主航线。请给出需要新增/修改的文件、launch 接线方式、参数设计和验证步骤。