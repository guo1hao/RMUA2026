# RMUA2026 本地工程参数调节与一键启动说明

这份文档只描述当前本地 PWM 路径飞行方案实际在用的内容，不描述官方基础飞控方案。

## 1. 先看启动入口

当前默认验证流程是渲染模式，不把 offscreen 作为默认路径。

### 1.1 最常用的三个脚本

| 脚本 | 作用 | 参数 | 什么时候用 |
| --- | --- | --- | --- |
| `scripts/start_competition_mode.sh` | 启动、停止或重启当前比赛模式寻点控制栈 | `start`、`stop`、`restart`，以及可选 `seed`，`target_waypoint_index`，`path_csv`，`lookahead_distance_m` | 以后统一用这一条进入或结束比赛模式 |
| `scripts/start_stack.sh` | 启动模拟器 + `rmua_sensor_hub` | `seed`，默认 `123` | 只想拉起模拟器和传感器转发时使用 |
| `scripts/start_pwm_stack.sh` | 启动模拟器 + 传感器 + 基础 PWM 控制器 | `seed`，`hover_altitude_m` | 只验证低层飞控时使用 |
| `scripts/start_path_mission.sh` | 启动模拟器 + 传感器 + 低层控制器 + 路径任务 | `seed`，`target_waypoint_index`，`path_csv`，`lookahead_distance_m`，其中 `seed` 省略时会自动随机生成 | 跑整条路径任务时使用 |
| `scripts/stop_stack.sh` | 停止 ROS 栈、模拟器、roscore，并检查残留渲染进程 | 无 | 每轮验证结束都执行 |

### 1.2 一键启动命令

#### 只启动模拟器和本地 ROS 传感器栈

```bash
cd /home/hao/workspace-RosNoetic/rmua2026
./scripts/start_stack.sh 123
```

#### 启动模拟器和基础 PWM 飞控栈

```bash
cd /home/hao/workspace-RosNoetic/rmua2026
./scripts/start_pwm_stack.sh 123 1.0
```

这个脚本拉起后，控制器默认待机。要让它进入悬停控制，执行：

```bash
./scripts/enable_pwm_controller.sh
```

关闭低层控制器：

```bash
./scripts/disable_pwm_controller.sh
```

#### 启动完整路径任务

推荐直接用比赛模式脚本：

```bash
cd /home/hao/workspace-RosNoetic/rmua2026
./scripts/start_competition_mode.sh
```

这个脚本会：

1. 自动生成一个随机种子。
2. 启动渲染模式模拟器。
3. 启动当前寻点任务，也就是现在使用的比赛模式控制栈。

如果你在这个启动终端里直接按 `Ctrl+C`，脚本现在会自动清理：

1. `rmua` 相关 ROS 节点。
2. 模拟器主进程和遗留子进程。
3. 由脚本启动的 `roscore`。
4. 残留渲染进程检查。

现在这套启动脚本的行为是：

1. 比赛失败、本地失败判定或任务完成时，不再因为任务节点退出而自动停掉整栈。
2. 启动脚本仍然保持前台阻塞，所以你在启动终端里按 `Ctrl+C` 时，依然会彻底清理 ROS、模拟器和脚本拉起的 `roscore`。
3. 如果你手动关闭模拟器窗口，脚本会检测到 simulator 进程已经退出，然后自动收栈释放剩余 ROS 进程。
4. 如果你是从另一个终端主动结束，仍然直接执行 `./scripts/start_competition_mode.sh stop` 或 `./scripts/stop_stack.sh`。

如果你是从另一个终端想主动结束，也不需要再记 `stop_stack.sh`，直接运行：

```bash
cd /home/hao/workspace-RosNoetic/rmua2026
./scripts/start_competition_mode.sh stop
```

如果你想一次性“先停再起”，可以用：

```bash
cd /home/hao/workspace-RosNoetic/rmua2026
./scripts/start_competition_mode.sh restart
```

如果你想手动指定种子或目标点，也可以这样用：

```bash
cd /home/hao/workspace-RosNoetic/rmua2026
./scripts/start_competition_mode.sh 123 -1 /home/hao/workspace-RosNoetic/rmua2026/drone_path.csv 30.0
```

原始入口仍然可用：

```bash
cd /home/hao/workspace-RosNoetic/rmua2026
./scripts/start_path_mission.sh
```

`start_path_mission.sh` 不带参数时，会自动随机生成种子，并使用当前默认目标点和路径文件。

手动传参时，4 个位置参数含义如下：

1. `123`：随机种子。
2. `-1`：目标 waypoint index，表示整条路径最后一个点。
3. `drone_path.csv`：路径文件。
4. `30.0`：只覆盖 `lookahead_distance_m`。

#### 每轮验证结束后停栈

```bash
cd /home/hao/workspace-RosNoetic/rmua2026
./scripts/start_competition_mode.sh stop
```

兼容入口 `./scripts/stop_stack.sh` 仍然保留，但日常可以不再单独记它。

### 1.3 启动脚本的重要行为

启动脚本的公共逻辑在 `scripts/common.sh`：

1. 会自动检查模拟器目录是否存在。
2. 如果工作区从未编译过，会自动执行一次 `catkin_make`。
3. 会先清理残留的旧 ROS 进程和旧 simulator 进程。
4. 默认以渲染模式启动模拟器。
5. `stop_stack.sh` 会额外检查是否还有残留渲染器进程。

注意：脚本只会在 `devel/setup.bash` 不存在时自动编译。如果你已经改过 C++ 或 launch 文件，需要自己重新执行：

```bash
cd /home/hao/workspace-RosNoetic/rmua2026
source /opt/ros/noetic/setup.bash
catkin_make --pkg rmua_flight_control rmua_sensor_hub
```

## 2. 平时应该改哪两个文件

日常调参优先改 launch，而不是先改 C++ 源码默认值。

### 2.1 任务层参数入口

文件：`src/rmua_flight_control/launch/pwm_path_mission.launch`

这里决定：

1. 路径跟踪的前视距离。
2. 过弯缩前视的激进程度。
3. waypoint 切换半径。
4. 失败判定阈值。
5. 传给低层控制器的少数关键参数，例如 `max_angle_rad`。

### 2.2 低层控制参数入口

文件：`src/rmua_flight_control/launch/basic_pwm_controller.launch`

这里决定：

1. 位置环、速度环、姿态环增益。
2. 高度积分与推力补偿。
3. 最大速度、最大加速度、最大倾角。
4. 物理模型参数和电机模型参数。

### 2.3 这两个 launch 背后的真实逻辑

如果你想确认某个参数最终在代码里怎么用，看下面两个文件：

1. `src/rmua_flight_control/src/path_mission_node.cpp`
2. `src/rmua_flight_control/src/basic_pwm_controller_node.cpp`

原则上：

1. 改 launch 是“当前这套方案的工作参数”。
2. 改 cpp 默认值是“以后所有 launch 的兜底默认值”。

## 3. 任务层参数：路径跟踪、过弯、失败判定

当前启动整条任务时，参数主要来自 `src/rmua_flight_control/launch/pwm_path_mission.launch`，真正的实现逻辑在 `src/rmua_flight_control/src/path_mission_node.cpp`。

### 3.1 路径与接口参数

| 参数 | 当前默认值 | 改哪里 | 调整后会发生什么 |
| --- | --- | --- | --- |
| `path_csv` | `/home/hao/workspace-RosNoetic/rmua2026/drone_path.csv` | `pwm_path_mission.launch` | 切换要飞的路径文件。 |
| `target_waypoint_index` | `-1` | `pwm_path_mission.launch` 或 `start_path_mission.sh` 第 2 个参数 | 决定任务以哪个 waypoint 为目标终点。`-1` 表示直接用路径文件最后一个点；传具体数值时则只飞到指定点。 |
| `keep_runtime_alive` | `true` | `pwm_path_mission.launch` | 任务完成或触发失败判定后，任务节点仍继续运行并持续保活整套控制栈，不主动退出。 |
| `stable_point_indices` | `69` | `pwm_path_mission.launch` | 定义路径里的“稳定点”。当前默认把 69 设为 pending1 到 pending2 的切换位。 |
| `stable_point_hold_time_s` | `3.0` | `pwm_path_mission.launch` | 到达稳定点后原地保持的时间。这段时间任务层会固定位置，只调 yaw 去对准下一个点。 |
| `stable_point_brake_trigger_distance_m` | `2.0` | `pwm_path_mission.launch` | 稳定点专用急刹触发距离。以 68→69 为例，只有离 69 足够近时才会锁到 69 点并进入 hold，避免提前几米开始缓慢减速。 |
| `publish_rate_hz` | `20.0` | `path_mission_node.cpp` 默认值，launch 里当前固定写死 `20.0` | 任务层 setpoint 发布频率。调大可让 setpoint 更新更密，但不会替代低层控制频率。 |
| `pose_topic` | `/rmua/sensors/drone_1/pose_gt` | launch 或 cpp | 只改接口，不改控制行为。 |
| `rotor_pwm_topic` | `/rmua/sensors/drone_1/debug/rotor_pwm` | launch 或 cpp | 只影响失败检测读取的 PWM 来源。 |
| `pose_setpoint_topic` | `/rmua/control/basic/pose_setpoint` | launch 或 cpp | 只改任务层向低层发目标的位置。 |
| `enable_topic` | `/rmua/control/basic/enable` | launch 或 cpp | 只改任务层启用低层控制器的 topic。 |

### 3.2 高频调参：前视与过弯

| 参数 | 当前默认值 | 改哪里 | 调大效果 | 调小效果 |
| --- | --- | --- | --- | --- |
| `lookahead_distance_m` | `16.0` | `pwm_path_mission.launch`，也可用 `start_path_mission.sh` 第 4 个参数临时覆盖 | 前视更远，路径更平滑，速度保持更容易，但更容易切弯、过弯不够狠 | 前视更近，转向更积极，更贴路径点，但更容易抖动、打满、提前锁死 |
| `min_lookahead_distance_m` | `3.0` | `pwm_path_mission.launch` | 不允许前视缩得太短，整体更稳，但弯道侵略性下降 | 允许前视更短，急弯更容易拐过去，但更容易出现急拉和振荡 |
| `lookahead_error_reduction_gain` | `1.5` | `pwm_path_mission.launch` | 偏离中心线后会更主动缩短前视，更快拉回路径，但可能造成突然修正 | 偏轨时缩前视更弱，飞行更平滑，但回线慢 |
| `turn_preview_distance_m` | `30.0` | `pwm_path_mission.launch` | 更早开始为转弯缩前视，提前准备拐弯，但过大时会让直道也变得保守 | 只有离拐点很近才开始收前视，反应更晚，可能来不及转 |
| `turn_min_lookahead_distance_m` | `3.0` | `pwm_path_mission.launch` | 弯道最小前视更大，转弯更保守、更稳 | 弯道最小前视更小，允许更狠地指向近点，但更容易过激 |
| `turn_lookahead_min_scale` | `0.20` | `pwm_path_mission.launch` | 更接近 `1.0` 时，弯道缩前视更弱，整体更圆滑 | 越小越允许弯道大幅缩前视，转向更急，但更容易把系统推到饱和 |
| `turn_angle_reduction_gain` | `4.5` | `pwm_path_mission.launch` | 同样的拐角会触发更强的前视缩短，急弯更敢拐 | 弯角对前视的影响减弱，过弯更缓 |
| `turn_short_segment_ignore_distance_m` | `4.0` | `pwm_path_mission.launch` | 更大时，短 segment 更容易被视为路径噪声而忽略，能减少异常短段触发的前视突变 | 更小时，短 segment 也会参与弯道缩前视，更容易出现急促指令变化 |
| `terminal_lookahead_distance_m` | `0.0` | `pwm_path_mission.launch` | 最终段仍保留前视，末端收敛更平滑，但容易不贴终点 | 趋近于 0 时，进入最终段后更直接瞄终点，有利于终点收敛 |
| `terminal_slowdown_distance_m` | `30.0` | `pwm_path_mission.launch` | 更早开始把终点段前视从全程值收回到终点值，终点更容易刹住 | 只在离终点很近时才开始减速，末段更容易冲过头 |

这一组里，最直接决定“过弯还能不能更急”的通常是：

1. `turn_min_lookahead_distance_m`
2. `turn_lookahead_min_scale`
3. `turn_angle_reduction_gain`
4. `max_angle_rad`，这个参数在低层，但对过弯上限影响很大

不要同时把这 4 个参数都往激进方向大幅拉满，否则很容易从“转不过去”变成“前段直接饱和锁死”。

### 3.3 waypoint 切换与终点收敛

| 参数 | 当前默认值 | 改哪里 | 调大效果 | 调小效果 |
| --- | --- | --- | --- | --- |
| `waypoint_acceptance_radius_m` | `3.0` | `pwm_path_mission.launch` | 更容易认为已经到达当前点，进度切换更快，但更容易切弯和跳点 | 切换更严格，更贴真实 waypoint，但更容易卡在点附近 |
| `final_acceptance_radius_m` | `3.0` | `pwm_path_mission.launch` | 更容易判定到达终点，末段更宽松 | 终点更严格，需要更贴近目标点 |
| `completion_hold_time_s` | `2.0` | `pwm_path_mission.launch` | 进入终点半径后会多保持一小段时间再正常结束，能避免刚到点就被瞬时扰动带走 | 设成更小会更快结束，设成 `0` 则一进入终点半径就直接成功退出 |
| `descent_release_distance_m` | `0.0` | `pwm_path_mission.launch` | 如果设成正数，会延后下降动作，只有水平更接近 waypoint 时才开始往下掉高度，适合避免太早下沉 | 设小或 0 会更早释放下降，更容易一边前进一边降高 |

补充说明：

1. `descent_release_distance_m` 仍然实际生效。
2. `turn_short_segment_ignore_distance_m` 用来过滤像 46/47 这种异常短段，避免它们把 turn lookahead 压得过短。
3. `terminal_slowdown_distance_m` 只作用在最终目标段，用来让终点前视更早收回来，不负责 69 这种稳定点的刹停。
4. `stable_point_brake_trigger_distance_m` 专门控制稳定点急刹触发距离，68→69 的停车手感优先调这个参数。
5. `descent_release_distance_m` 只在“目标点要求下降”时起作用，用来避免飞机离得还很远就先掉高度。

### 3.4 稳定点与 pending 切换

当前代码已经引入“稳定点”概念：

1. `0 -> 69` 这一段视为 `pending1`。
2. `69` 当前被定义为稳定点，也是 `pending1 -> pending2` 的切换位置。
3. 飞机不会在离 69 还很远时提前锁点，而是要先进入 `stable_point_brake_trigger_distance_m` 设定的近距离区间，才会触发对 69 的急刹和 hold。
4. 触发后会在 69 附近保持 `3` 秒左右。
5. 这 3 秒内位置目标固定在 69 点，但 yaw 会对准下一个点 `70`。
6. 保持完成后，任务自动切换到 `pending2`，继续沿 `70 -> ... -> 最后一个点` 前进。

如果后面还要再扩展更多段，只需要继续往 `stable_point_indices` 里追加新的切换点，比如 `69,120,180`。

### 3.5 失败判定参数

当前代码没有直接订阅到官方“比赛失败”或“碰撞结果”话题，所以这里全部是本地失败判定。

| 参数 | 当前默认值 | 改哪里 | 调大效果 | 调小效果 |
| --- | --- | --- | --- | --- |
| `failure_abort_on_detection` | `false` | `pwm_path_mission.launch` | 只有在 `keep_runtime_alive=false` 时才允许失败后主动结束任务节点；当前默认保活模式下不会自动停栈 | 设成 `false` 时，即使关闭保活模式，也只记日志或维持当前运行，不主动结束整栈 |
| `failure_progress_stall_timeout_s` | `3.0` | `pwm_path_mission.launch` | 允许更久没有进度，误判更少，但失败发现更慢 | 更快认定“卡住了”，但更容易误判 |
| `failure_stall_motion_radius_m` | `1.5` | `pwm_path_mission.launch` | 在超时窗口里，只要位移没超过更大的半径就算停滞，判定更严格 | 更宽松，不容易因为小范围抖动被判失败 |
| `failure_distance_improvement_m` | `1.5` | `pwm_path_mission.launch` | 需要更明显地接近终点才算“有进展”，判定更严格 | 只要略微接近终点就会刷新进展时间，更宽松 |
| `failure_min_distance_to_goal_m` | `12.0` | `pwm_path_mission.launch` | 更接近终点时不再做失败判定，末段更宽松 | 失败判定会继续覆盖到更靠近终点的位置 |
| `failure_zero_pwm_timeout_s` | `1.0` | `pwm_path_mission.launch` | 允许更长时间平均 PWM 很低，误判更少 | 更快把“长时间掉电机输出”认成失败 |
| `failure_zero_pwm_threshold` | `0.08` | `pwm_path_mission.launch` | 更容易把低推力状态当成“接近零 PWM” | 只有更接近完全没推力才算失败 |
| `failure_zero_pwm_min_progress` | `8` | `pwm_path_mission.launch` | 零 PWM 检测更晚启用，起飞初期更宽松 | 更早开始检测零 PWM |
| `failure_grace_period_s` | `8.0` | `pwm_path_mission.launch` | 起飞后更长时间不做失败判定，减少初始化误报 | 更早进入失败监控 |

补充说明：

1. 当前失败后不会主动向飞机再发“关闭控制器”之类的命令。
2. 现在默认 `keep_runtime_alive=true`：识别到失败后只记日志并保持整栈继续运行；按 `Ctrl+C`、执行 `stop`，或者手动关闭模拟器时，脚本才会负责收栈。

### 3.6 当前 launch 里保留但在这版代码里没有实际效果的参数

下面这些参数目前在 `path_mission_node.cpp` 里只加载和裁剪，但没有真正参与轨迹采样逻辑：

1. `climb_priority_threshold_m`
2. `climb_horizontal_leash_m`
3. `climb_horizontal_leash_scale`
4. `climb_horizontal_leash_max_m`
5. `climb_vertical_release_distance_m`
6. `climb_vertical_step_m`
7. `climb_preview_waypoints`

结论：当前版本改这 7 个参数，基本不会对飞行结果产生实际影响。

## 4. 低层 PWM 参数：位置环、速度环、姿态环、推力映射

当前低层主参数在 `src/rmua_flight_control/launch/basic_pwm_controller.launch`，实现逻辑在 `src/rmua_flight_control/src/basic_pwm_controller_node.cpp`。

补充一点：

1. 直接跑 `start_pwm_stack.sh` 时，`max_angle_rad` 走的是 `pwm_hover_stack.launch` 的默认值 `0.5`。
2. 跑 `start_path_mission.sh` 时，`pwm_path_mission.launch` 会把 `max_angle_rad` 覆盖成当前的 `1.05`，所以完整路径任务比单独悬停栈更激进。

### 4.1 最值得优先调的低层参数

| 参数 | 当前默认值 | 改哪里 | 调大效果 | 调小效果 |
| --- | --- | --- | --- | --- |
| `hover_pwm_estimate_override` | `0.265` | `pwm_path_mission.launch` 或 `basic_pwm_controller.launch` 参数入口 | 同样的力会映射成更高 PWM，整机更“热”，更容易拉得起来，但也更容易饱和 | 同样的力会映射成更低 PWM，响应更钝，过低时会显得推力不够 |
| `max_angle_rad` | 路径任务当前 `1.05`，基础 hover 栈默认 `0.5` | `pwm_path_mission.launch` 或 `pwm_hover_stack.launch` | 允许更大滚转/俯仰角，急弯、横向机动更强，但更容易掉高、撞击或推力打满 | 倾角受限，飞行更稳，但转不过急弯的概率上升 |
| `tilt_compensation_min_cos` | `0.6` | `basic_pwm_controller.launch` | 更大时，倾斜时的推力补偿更保守，不容易突然拉大总推力，但大倾角时更容易掉高 | 更小时，允许更强的倾斜补偿，更利于大倾角保持高度，但更容易出现推力峰值和饱和 |
| `max_velocity` | `16.0` | `basic_pwm_controller.launch` | 位置环允许更高水平速度指令，整体更冲 | 最高水平速度更低，整体更稳，但追点更慢 |
| `max_acceleration` | `16.0` | `basic_pwm_controller.launch` | 允许更激烈的横向加速，转向更狠 | 横向动作更柔和，更不容易抖，但会显得转向不足 |

### 4.2 水平位置与速度通道

这一组决定“平面内追点有多凶”。

| 参数 | 当前默认值 | 改哪里 | 调大效果 | 调小效果 |
| --- | --- | --- | --- | --- |
| `kp_x` | `1.0` | `basic_pwm_controller.launch` | x 向位置误差会更快变成速度目标，追点更积极 | x 向追点更慢 |
| `kd_x` | `0.1` | `basic_pwm_controller.launch` | x 向位置误差变化更敏感，修正更急，但更容易抖 | 更平滑，但刹车和纠偏更慢 |
| `kp_y` | `1.0` | `basic_pwm_controller.launch` | y 向同理，更积极 | 更慢、更稳 |
| `kd_y` | `0.1` | `basic_pwm_controller.launch` | y 向同理，更敏感 | 更平滑 |
| `kp_vx` | `0.5` | `basic_pwm_controller.launch` | 速度误差更快转成横向加速度，机体更愿意压坡去追速度 | 横向加速度更保守 |
| `kd_vx` | `1.0` | `basic_pwm_controller.launch` | x 方向的速度变化抑制更强，也更敏感 | 更柔和，但容易拖泥带水 |
| `kp_vy` | `0.5` | `basic_pwm_controller.launch` | y 向同理 | y 向同理 |
| `kd_vy` | `1.0` | `basic_pwm_controller.launch` | y 向同理 | y 向同理 |

实际调参时，这一组和 `max_angle_rad`、`max_acceleration` 是联动的。

如果你把 `kp_x/kp_y/kp_vx/kp_vy` 拉高，但 `max_angle_rad` 太小，最终还是会表现成“命令很激进，但机体拐不过去”。

### 4.3 高度与推力通道

这一组决定“高度能不能稳住、上升/下降会不会拖泥带水”。

| 参数 | 当前默认值 | 改哪里 | 调大效果 | 调小效果 |
| --- | --- | --- | --- | --- |
| `kp_z` | `4.0` | `basic_pwm_controller.launch` | 高度误差会更快转成推力修正，上下更利索，但更容易超调 | 上下更慢、更软 |
| `ki_z` | `0.35` | `basic_pwm_controller.launch` | 更容易消除长期高度偏差，但过大时会积累过冲 | 稳态高度偏差更难消掉 |
| `kv_z` | `6.0` | `basic_pwm_controller.launch` | 纵向速度阻尼更强，上下振荡更少，但爬升会被压住 | 爬升更猛，但更容易上下波动 |
| `z_integral_limit` | `8.0` | `basic_pwm_controller.launch` | 允许更大的高度积分补偿，能抗持续偏差，但 windup 风险更高 | 积分作用被限制得更紧 |
| `z_integral_deadband_m` | `0.25` | `basic_pwm_controller.launch` | 更大时，小高度误差范围内不会继续积积分，更稳但更难抹平细小高度偏差 | 更小则更容易持续积积分，收敛更紧，但更可能抖 |
| `z_integral_horizontal_error_limit_m` | `4.0` | `basic_pwm_controller.launch` | 允许在更大的横向偏差下仍保留高度积分，更有利于边偏航边保高度 | 横向一旦偏太多就会快速卸掉高度积分，更不容易 windup |
| `z_integral_unwind_rate` | `4.0` | `basic_pwm_controller.launch` | 当积分条件不满足时，积分衰减更快 | 积分保留更久 |

### 4.4 大爬升时的横向降权

这一组的目的不是让你飞得更快，而是防止“又想猛爬升，又想大幅横向机动”时直接把推力用满。

代码逻辑是：当总推力需求已经很高，而且 z 误差很大、xy 误差又不算大时，会把横向加速度缩小。

| 参数 | 当前默认值 | 改哪里 | 调大效果 | 调小效果 |
| --- | --- | --- | --- | --- |
| `vertical_priority_force_ratio` | `0.85` | `basic_pwm_controller.launch` | 需要更接近推力上限才触发横向降权，机体更愿意一边爬一边横冲 | 更早触发横向降权，更保守 |
| `vertical_priority_error_z_m` | `8.0` | `basic_pwm_controller.launch` | 只有 z 误差很大时才触发降权 | 稍微需要爬升时就可能压横向 |
| `vertical_priority_horizontal_error_m` | `4.0` | `basic_pwm_controller.launch` | 允许在更大的 xy 误差下仍触发纵向优先 | 只有非常贴近当前 xy 目标时才触发 |
| `vertical_priority_horizontal_scale` | `0.2` | `basic_pwm_controller.launch` | 触发后横向动作保留更多 | 触发后横向动作被压得更狠 |

如果出现“急转的时候高度掉得厉害”，优先联动检查：

1. `max_angle_rad`
2. `tilt_compensation_min_cos`
3. `hover_pwm_estimate_override`
4. `vertical_priority_*`

### 4.5 姿态环稳定性

这一组决定“机体把期望滚转/俯仰/偏航真正执行出来的快慢和稳定性”。

| 参数 | 当前默认值 | 改哪里 | 调大效果 | 调小效果 |
| --- | --- | --- | --- | --- |
| `kp_roll` | `17.0` | `basic_pwm_controller.launch` | 机体更快跟随期望滚转角 | 滚转更软 |
| `kd_roll` | `940.0` | `basic_pwm_controller.launch` | 滚转阻尼更强，能压制抖动，但过大会显得钝 | 更灵活，但可能抖 |
| `kp_pitch` | `20.0` | `basic_pwm_controller.launch` | 俯仰更快跟随 | 更软 |
| `kd_pitch` | `750.0` | `basic_pwm_controller.launch` | 俯仰阻尼更强 | 更灵活但更容易震荡 |
| `kp_yaw` | `5.0` | `basic_pwm_controller.launch` | 朝向修正更快 | 偏航更慢 |
| `kd_yaw` | `250.0` | `basic_pwm_controller.launch` | 偏航更稳，但过大时反应会钝 | 偏航更灵活 |
| `roll_rate_damping` | `8.0` | `basic_pwm_controller.launch` | 用 IMU 滚转角速度直接给姿态环加阻尼，能压制快速抖动 | 阻尼更弱，机体更灵活但更容易甩 |
| `pitch_rate_damping` | `8.0` | `basic_pwm_controller.launch` | 用 IMU 俯仰角速度直接做阻尼，能压制 pitch 来回抽动 | 更容易在大姿态变化时震荡 |
| `yaw_rate_damping` | `4.0` | `basic_pwm_controller.launch` | 用 IMU 偏航角速度直接抑制偏航过冲 | 偏航更灵活，但更容易摆尾 |
| `max_roll_pitch_command_rate_rad_s` | `2.5` | `basic_pwm_controller.launch` | 限制期望滚俯角变化速度，能防止 setpoint 跳变直接把机体打疯 | 允许姿态指令更快变化，转向更猛但更容易抽动 |
| `max_yaw_command_rate_rad_s` | `1.5` | `basic_pwm_controller.launch` | 限制期望偏航变化速度，能减小小短段带来的朝向突变 | 允许偏航指令更快跳变，朝向更激进 |
| `horizontal_velocity_slowdown_distance_m` | `12.0` | `basic_pwm_controller.launch` | 离当前目标较近时更早开始收水平速度指令，终点和悬停都更容易刹住 | 只有非常近才开始减速，更容易冲过 setpoint |
| `horizontal_velocity_min_scale` | `0.15` | `basic_pwm_controller.launch` | 接近目标后仍保留的最小水平速度比例更大，动作更利索但刹车更弱 | 接近目标时更愿意把水平速度收小，更容易停住，但过小会显得拖 |
| `max_roll_pitch_torque` | `2.0` | `basic_pwm_controller.launch` | 允许更大的滚俯力矩，姿态更敢打 | 力矩上限更低，更稳但更慢 |
| `max_yaw_torque` | `1.0` | `basic_pwm_controller.launch` | 允许更大的偏航力矩 | 偏航更保守 |

### 4.6 控制器启用、home 初始化、空闲行为

| 参数 | 当前默认值 | 改哪里 | 调整后会发生什么 |
| --- | --- | --- | --- |
| `auto_enable` | `false` | `basic_pwm_controller.launch` | `false` 时必须通过 `/rmua/control/basic/enable` 开控制；`true` 时上电即开始控制。 |
| `publish_zero_when_inactive` | `true` | `basic_pwm_controller.launch` | 控制器没启用或没准备好时，是否持续发 0 PWM。 |
| `hover_altitude_m` | `1.0` | `basic_pwm_controller.launch` 或 `start_pwm_stack.sh` 第 2 个参数 | 只在没有外部路径 setpoint 时，决定默认悬停高度。 |
| `use_initial_pose_as_home` | `true` | `basic_pwm_controller.launch` | 是否优先用 `initial_pose` 作为 home。 |
| `home_from_first_pose_if_missing` | `true` | `basic_pwm_controller.launch` | 如果 `initial_pose` 不可信或缺失，是否退回到第一帧 `pose_gt` 作为 home。 |
| `initial_pose_home_distance_tolerance_m` | `2.0` | `basic_pwm_controller.launch` | 允许 `initial_pose` 与 `pose_gt` 的最大偏差。调大后更容易接受 `initial_pose`；调小后更容易退回第一帧真值。 |
| `control_rate_hz` | `100.0` | `basic_pwm_controller.launch` | 控制循环频率。调大响应更快，但更吃 CPU，也更容易放大噪声。 |
| `velocity_filter_alpha` | `0.25` | `basic_pwm_controller.launch` | 越大，速度估计越灵敏但噪声更大；越小，速度估计更平滑但滞后更明显。 |
| `min_active_pwm` | `0.0` | `basic_pwm_controller.launch` | 给每个电机设置最小激活 PWM。调大能避免电机过低，但会降低小力矩调节空间。 |

### 4.7 物理模型参数

下面这些参数也能调，但通常不建议把它们当作第一优先级调参项，因为它们描述的是机体和电机模型本身：

1. `mass`
2. `gravity`
3. `arm_length`
4. `ixx`
5. `iyy`
6. `izz`
7. `thrust_coefficient`
8. `torque_coefficient`
9. `max_rotor_rpm`

这些参数在 `basic_pwm_controller.launch` 可以改，在 `basic_pwm_controller_node.cpp` 里参与以下几件事：

1. 把期望总推力换算成 PWM。
2. 计算电机单轴力矩。
3. 计算 hover PWM 和总推力上限。

只有在你确认模拟器机体模型、质量、电机模型常数和当前代码不一致时，才应该动它们。否则更推荐先调增益和限制项。

## 5. 最实用的调参顺序

如果你的目标是“前段急弯再狠一点，但不能直接饱和锁死”，建议按下面顺序微调：

1. 先调任务层的 `turn_min_lookahead_distance_m`、`turn_lookahead_min_scale`、`turn_angle_reduction_gain`。
2. 再调低层的 `max_angle_rad`，确认机体确实有足够倾角权限。
3. 如果大倾角时开始掉高，再联动检查 `tilt_compensation_min_cos` 和 `hover_pwm_estimate_override`。
4. 如果开始出现“动作很猛但摇晃”，先检查 `roll_rate_damping`、`pitch_rate_damping` 和姿态指令限速，再微调 `kd_roll`、`kd_pitch`。
5. 如果出现“并没有真正失控，只是被本地失败判定提前终止”，再放宽 `failure_*`。

如果你的目标是“高度更稳，不要一转弯就掉”，优先顺序建议改成：

1. `kp_z`
2. `ki_z`
3. `kv_z`
4. `tilt_compensation_min_cos`
5. `hover_pwm_estimate_override`

## 6. 快速定位清单

### 6.1 整条路径任务的主要入口

1. `scripts/start_path_mission.sh`
2. `src/rmua_flight_control/launch/pwm_path_mission.launch`
3. `src/rmua_flight_control/launch/pwm_hover_stack.launch`
4. `src/rmua_flight_control/launch/basic_pwm_controller.launch`
5. `src/rmua_flight_control/src/path_mission_node.cpp`
6. `src/rmua_flight_control/src/basic_pwm_controller_node.cpp`

### 6.2 只启动模拟器和本地 ROS 工程

1. `scripts/start_stack.sh`
2. `scripts/common.sh`
3. `scripts/stop_stack.sh`

如果你后面想把这份文档继续扩展成“参数改动建议模板”，最适合继续补充的就是每种典型故障对应先改哪 3 个参数。