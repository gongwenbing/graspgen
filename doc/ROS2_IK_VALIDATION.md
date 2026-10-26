# ROS2 Humble 逆解与交互标记验证

日期：2026-10-20（Asia/Shanghai）。范围为默认 `piper_ik_node`、交互标记与启动控制，
不包含视觉抓取、MoveIt 真机桥接或物理运动验收。

## 结果

最终版本编译安装成功。8 个 C++ 回归用例、实际 ROS2 节点通信测试和模拟硬件反馈测试均通过。
测试未启动 CAN 驱动；ROS2 通信测试使用独立域 173、174，与用户实际运行的节点隔离。

| 项目 | 最终结果 |
|---|---|
| URDF 正解、关节限位、工具偏移 | 通过 |
| 从零位求解 120 组确定可达目标 | 120/120 |
| 连续可达路径 | 101/101 个点 |
| 非零工作姿态逐轴平移 ±5 mm、旋转 ±2° | 12/12 |
| 实际交互标记反馈 → 目标 → 逆解 → 模型 TF | 通过 |
| ROS2 中模拟 12 个逐轴拖动 | 12/12；观察到的最大位置误差 0.927 mm、最大姿态误差 0.566° |
| ROS2 中连续发布 20 个标记目标 | 20/20 |
| 夹爪单位换算、两侧夹指 TF/mimic | 通过 |
| 不可达及无效目标 | 被拒绝；可视化模式没有真机指令输出 |
| 真机模式启动、无反馈及未启用 | 无指令输出 |
| 显式启用后仍需新目标、指令变化率限制 | 通过模拟反馈验证 |
| 反馈过期/无效、恢复、重新启用、手动关闭 | 停发正确，恢复后不自动运动 |

数值验收条件保持为：位置误差不超过 1 mm，姿态误差不超过 0.01 rad（约 0.573°）。
位置与姿态最大值不一定出现在同一个目标；上述是测试实际观察值，不是全工作空间精度保证。

## 本轮发现并修复的问题

1. 扩大到 120 个可达样本后，原求解器漏解了其中 1 个。扩大确定性备用初值搜索后通过。
2. 零位保持姿态沿 X 正方向移动 5 mm，独立 SciPy/URDF 搜索找到同时满足两项容差的解，
   但原加权平方误差求解偏向更小姿态误差，导致位置误差略超限。
   现在保留常规优化；常规优化停滞时，增加位置和姿态容差边界修正，再尝试其他初值。
   所有返回结果仍必须通过原容差和 URDF 限位校验。

## 零位的限制

全零姿态中 joint2 和 joint3 位于 URDF 限位边界，不能期待任意独立平移或旋转都可实现。
同样的 12 项逐轴探测，修复前通过 8 项，修复后通过 9 项。
仍被拒绝的是保持其他分量不变时的 X 负向 5 mm、Z 负向 5 mm、绕 Y 负向 2°。
独立数值搜索同样没有找到这三个目标的合格解，但这不构成全局不可达的数学证明。
拒绝这些目标时，不能通过放宽关节限位或忽略姿态误差强行执行。

用户提供的历史日志没有包含完整目标位置与四元数，因此本轮未逐条重放那些历史目标。
本轮使用明确记录、可重复构造的目标验证了启动位姿、非零工作姿态和连续路径。

## 复现

先按 `piper_kinematics/README.md` 编译，C++ 测试需要 `-DBUILD_TESTING=ON`，再加载工作区：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select piper_kinematics
colcon test-result --test-result-base build/piper_kinematics --verbose
python3 piper_kinematics/test/smoke_ros2.py
python3 piper_kinematics/test/hardware_startup_ros2.py
python3 piper_kinematics/test/independent_drag_check.py
```

`probe_drag_workspace` 是启用测试后生成的诊断可执行文件，输出零位和工作姿态的逐轴探测 CSV。
其源码为 `piper_kinematics/test/probe_drag_workspace.cpp`。
独立 SciPy 检查不启动 ROS 节点或硬件，结果保存到 `/tmp/graspgen-independent-drag-check.json`。

新二进制已经安装，但用户此前启动的进程仍需手动停止、重新加载工作区并启动，才能使用修改。
本轮没有停止用户原有进程，也没有使能真实机械臂。停发指令不等同于硬件急停。
