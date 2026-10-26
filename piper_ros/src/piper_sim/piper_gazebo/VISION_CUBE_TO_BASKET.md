# RGB-D 方块识别与放筐

场景沿用 `pick_cube.world`：红、蓝、绿 40 mm 方块，`pick_basket` 为已知固定放置目标。
识别不使用 Gazebo 物体位姿：HSV 颜色分割 + 对齐深度 + 图像时刻 TF 获得顶面，按已知边长估计方块中心和边方向。
这是已知颜色/尺寸的规则识别，不是新增 YOLO 训练类别；confidence=1 表示颜色规则匹配，不代表统计置信度。
Gazebo 位姿只用于规划场景、观察视角、定位误差检查及抓放结果验证。方块抓取位置来自视觉，筐的位置来自仿真配置。

每个终端先运行：

```bash
source /opt/ros/humble/setup.bash
source ~/GraspGen/install/setup.bash
source ~/GraspGen/piper_ros/install_d455/local_setup.bash
export ROS_DOMAIN_ID=188
```

场景只启动一次：

```bash
ros2 launch piper_gazebo piper_pick_scene.launch.py moveit:=true enable_camera:=true
```

停止旧瓶子视觉节点后，在新终端启动方块识别：

```bash
ros2 launch graspgen_vision cube_vision.launch.py
```

观察红色方块（仅视角规划使用仿真位置）：

```bash
ros2 run piper_gazebo observe_bottle.py --ros-args \
  -p target_geometry:=cube -p bottle_name:=pick_cube -p execute:=true
```

检查识别和抓取悬停逆解，不移动：

```bash
ros2 topic echo /graspgen/status --field data --once
ros2 run piper_gazebo vision_cube_to_basket.py --ros-args -p target_class:=cube_red
```

执行单个红色方块抓放：

```bash
ros2 run piper_gazebo vision_cube_to_basket.py --ros-args \
  -p target_class:=cube_red -p execute:=true
```

脚本验证双侧接触、实际抬升和相对滑移后才搬运，释放后检查方块是否落在筐内，再回到起始观察关节位。
`cube_blue`、`cube_green` 可分别选择；空类别配合 `max_objects:=3` 可循环抓取三个颜色，但每次回到观察位都必须实际看得到剩余方块。
不能把方块从筐中重新选作桌面目标。默认不使用稳定插件；可显式指定 `assisted_grasp:=true` 启用现有双侧接触后固定约束，报告会标记 physics_attachment_used=true，这只验证仿真流程，不能证明真机摩擦保持能力。
失败时保持当前状态，不自动释放或重复抓取。报告默认 `~/GraspGen/calibration/sim_vision_cube_to_basket.json`。


搬运默认 `transport_height:=0.09`，表示方块释放位置上方 9 cm（不是夹爪下降深度）。
当前筐底模型原点 z=1.015m，释放中心 z=1.047m，搬运中心 z=1.137m；比桌面上方块中心约高 10.2cm。
原地先试抬 5cm，验证保持后再沿规划轨迹向筐移动并抬高。10cm 释放上方目标曾逆解失败，9cm 已完成实测。
若已成功抬起但搬运失败，可在稳定插件仍确认持有同一方块、且报告中存在 lift_verified 时继续放置：

```bash
ros2 run piper_gazebo vision_cube_to_basket.py --ros-args \
  -p phase:=place -p execute:=true -p assisted_grasp:=true \
  -p transport_height:=0.09 \
  -p report_file:=/home/zlq/GraspGen/calibration/sim_vision_cube_assisted_test.json
```

2026-10-26 实测：红色方块视觉中心误差约 0.9–1.1mm；纯摩擦抓取仍滑脱，程序正确阻止搬运。
显式启用稳定插件后实际抬升49.98mm，相对滑移0.035mm，随后完成抬高搬运、释放和筐内落点验证。
最终筐内局部中心为 (-0.05995, 0.00026, 0.02590)m；报告 `calibration/sim_vision_cube_assisted_test.json` 标记使用物理约束。

## 一键自动流程

关闭旧仿真及旧视觉节点后，在同一终端加载环境并运行：

```bash
source /opt/ros/humble/setup.bash
source ~/GraspGen/install/setup.bash
source ~/GraspGen/piper_ros/install_d455/local_setup.bash
export ROS_DOMAIN_ID=188
ros2 launch piper_gazebo piper_auto_cube.launch.py
```

自动启动 Gazebo、D455、MoveIt、方块识别；等待就绪后逐个寻找红/蓝/绿观察位，使用新鲜 RGB-D 目标抓取、抬升并放到筐内不同位置，回到观察关节位。默认种子455，三个方块，搬运高度参数0.09m。
默认 `assisted_grasp:=true` 使用双侧接触后仿真固定约束，日志和报告明确标记。纯摩擦模式可指定 `assisted_grasp:=false`，当前模型可能滑脱并停止。

可选参数：`count:=1` 仅红色；`count:=2` 红/蓝；`gui:=false rviz:=false` 无界面。
状态：`ros2 topic echo /auto_cube/status --field data`；每次运行报告保存在 `~/GraspGen/calibration/auto_cube/日期_时间/`。
任一观察、识别、规划、接触或放置验证失败时停止任务，保留现场，不自动重试或释放。任务完成后场景保持打开，可查看结果；Ctrl+C 结束整个启动组。
启动前会检查 Gazebo 端口，存在旧场景时直接报明确错误，不再叠加启动。

也可以直接用封装脚本（自动加载三个 setup 文件并设置 ROS_DOMAIN_ID=188）：

```bash
bash ~/GraspGen/piper_ros/start_auto_cube.sh
```

例如仅测试一个：`bash ~/GraspGen/piper_ros/start_auto_cube.sh count:=1`。
筐内三个槽位局部坐标分别为 (-0.06,0)、(0,0)、(-0.06,0.06)m；搬运前检查完整下降路径，释放时先张开到54mm，撤离后再完全张开，避免碰撞相邻方块。

最终一键验证（2026-10-26）：独立域189/端口11346从新场景自动完成三个方块的全部观察、识别、抓放、撤离和回观察位，`summary.json` 为 success=true、count=3，最终筐内位置全部通过。验证报告：`calibration/auto_cube/20261026_175328/summary.json`。
