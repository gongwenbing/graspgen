# PiPER 震颤基线诊断（只读）

本阶段只增加观测工具，不改变轨迹规划、插值、CAN 指令、伺服参数或输出门控。
诊断节点不连接 SDK/CAN，也不发布任何执行器命令。

## 1. 编译

在工作区根目录：

```bash
cd ~/GraspGen
source /opt/ros/humble/setup.bash
colcon build --base-paths piper_moveit_bridge --executor sequential
source install/setup.bash
```

## 2. 启动条件

先按现有真机说明启动唯一的 CAN gateway、MoveIt 和 trajectory bridge。
不要启动旧驱动或第二个 CAN 控制程序。先确认机械臂处于安全、可观察状态。

另开终端：

```bash
cd ~/GraspGen
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=187
ros2 run piper_moveit_bridge vibration_monitor
```

节点只订阅：

- `/moveit/joint_commands`
- `/joint_states_single`
- `/piper_hardware/status`

并发布 JSON 指标到 `/piper_vibration/metrics`。

## 3. 查看指标

```bash
ros2 topic echo /piper_vibration/metrics
```

建议分别记录以下场景，每个场景至少 20 秒：

1. 机械臂静止、空载（不发运动指令）。
2. 空载执行一段低风险、短距离运动。
3. 抓取流程中不接触物体的运动阶段。
4. 如有必要，再单独观察夹爪动作。

记录时同时保存原始话题：

```bash
ros2 bag record -o ~/piper_vibration_baseline \
  /joint_states_single \
  /moveit/joint_commands \
  /piper_hardware/status \
  /piper_bridge/readiness \
  /piper_vibration/metrics
```

按 Ctrl+C 结束 rosbag。

## 4. 指标解释与限制

- `command_period`：本节点收到关节命令的周期均值、标准差和最大值。
- `feedback_period`：本节点收到关节反馈的周期均值、标准差和最大值。
- `tracking_error`：当前实现将最近收到的目标与反馈位置作差，属于粗略诊断量，不是时间同步后的真实伺服跟踪误差。
- `hardware`：转发网关状态中的命令计数、拒绝计数、门控和故障原因。

ROS 话题接收频率不等于 CAN 总线真实帧频率，也不等于电机内部伺服频率。
若要判断高频机械振动，需要额外的高采样率外部传感器或厂家提供的高速反馈数据；
普通 ROS 关节反馈可能不足以观测高频振动。

## 5. 安全边界

本工具不发送运动、使能、模式切换、复位、回零或夹爪命令。
本阶段不修改 `trajectory_bridge.py`、`can_gateway.py`、URDF 限位或控制器参数。
在诊断数据完成分析前，不要通过提高控制频率、反复发送模式命令或提高速度上限来“试抖动”。
