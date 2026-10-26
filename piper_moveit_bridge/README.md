# PiPER MoveIt 真机桥接（ROS2 Humble）

本包提供 `/arm_controller/follow_joint_trajectory` 和
`/gripper_controller/follow_joint_trajectory`，把 MoveIt 轨迹按时间插值为 PiPER
关节命令，并以真实反馈判定到位。硬件启动不使用 GenericSystem，也不需要旧 PiPER 控制节点。

默认只读，不自动使能电机、不回零、不切换模式。SDK 固定沿用当前环境的 0.6.2，
此次设备实际查询固件为 S-V1.8-2；没有升级 SDK 或固件。

## 编译

在 GraspGen 根目录执行：

```bash
cd ~/GraspGen
source /opt/ros/humble/setup.bash
colcon build --base-paths \
  piper_ros/src/piper_msgs \
  piper_ros/src/piper_description \
  piper_ros/src/piper_moveit/piper_with_gripper_moveit \
  piper_moveit_bridge \
  --executor sequential --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
python3 -c 'import piper_sdk, importlib.metadata as m; print(m.version("piper_sdk"), piper_sdk.__file__)'
```

SDK import 应来自已验证的 0.6.2。不要从下载的 SDK 源码目录启动本流程，避免导入路径改变。

## 真实反馈与规划：先执行本阶段

先用 Ctrl+C 停止之前的 PiPER 驱动、自定义 IK、MoveIt demo 和 RViz。不要同时运行
旧 `piper_single_ctrl`、旧使能脚本或其他对同一 CAN 总线发送控制的程序。
以下各终端统一使用 ROS_DOMAIN_ID=187。

检查 CAN：

```bash
ip -brief link show can0
```

仅当 can0 尚未 UP 时执行：

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

终端 A：

```bash
cd ~/GraspGen
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=187
ros2 launch piper_moveit_bridge moveit_hardware.launch.py \
  can_port:=can0 read_only:=true rviz:=true
```

启动真实 CAN 反馈、模型 TF、MoveIt、RViz、桥接 action。此阶段所有运动出口关闭。
可以观察当前姿态；原始角度保留在 /joint_states_single 和 /piper_hardware/status；规划状态采用下述测量边界规则。
当前设备已读到的示教状态、joint2 约 -1.596° 会阻止执行，不得绕过此检查。

终端 B：

```bash
cd ~/GraspGen
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=187
ros2 topic echo /piper_hardware/status --once
ros2 topic echo /piper_bridge/readiness --once
ros2 topic echo /joint_states_single --once
ros2 action list -t
ros2 param get /move_group use_sim_time
```

应看到两个 FollowJointTrajectory action，`use_sim_time` 为 false。
只读模式输出关闭是正常结果。不要同时再启动 ros2_control 模拟控制器。

小步规划（不会请求执行）：

```bash
ros2 run piper_moveit_bridge small_step --joint joint1 --delta 0.01
```

当前六轴状态越限、碰撞或缺少新鲜反馈时，规划工具会拒绝继续。不要直接改零点、
放宽模型限位、修改原始反馈，或发送回零命令来绕过。

## 真机执行：仅在反馈和实物条件通过后

本阶段由操作者执行。先按设备正常操作退出示教/录制，确认周围留有运动空间。
`/piper_hardware/status` 需满足：ctrl_mode 为 0 或 1；teach_status 为 0、2 或 6；
err_code 为 0；reason 无示教、反馈、限位错误；六轴反馈在 URDF 范围内。
照片中的桌面、额外相机支架和线缆没有完整纳入本模型，需由操作者检查小步运动的间隙。

终端 A 按 Ctrl+C 停止只读启动，再执行：

```bash
ros2 launch piper_moveit_bridge moveit_hardware.launch.py \
  can_port:=can0 read_only:=false rviz:=true
```

即使 read_only=false，启动时两个输出门控也仍关闭，没有自动运动命令。
不要启动旧 IK 或其他轨迹命令发布器。

终端 B 先检查状态：

```bash
ros2 topic echo /piper_hardware/status --once
```

如果 `motors_enabled` 已经全为 true，跳过下面的电机使能命令。
仅在确实需要使能、且实物处于可控状态时，由操作者执行：

```bash
ros2 service call /piper_can_gateway/enable_motors std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /piper_hardware/status --once
```

该服务只发送电机使能请求，不发送夹爪归零/闭合或关节目标；返回 success 只表示请求已发送，
仍需检查状态中的 motors_enabled。不要用旧驱动的 enable_srv 替代本服务。

先规划确认可行：

```bash
ros2 run piper_moveit_bridge small_step --joint joint1 --delta 0.01
```

按顺序开启输出门控，每次都确认 `success: true`；任何失败都停止此阶段并检查 message：

```bash
ros2 service call /piper_can_gateway/enable_outputs std_srvs/srv/SetBool '{data: true}'
ros2 service call /piper_trajectory_bridge/enable_execution std_srvs/srv/SetBool '{data: true}'
```

操作者确认后请求一次真机小步执行：

```bash
ros2 run piper_moveit_bridge small_step --joint joint1 --delta 0.01 --execute
```

此命令会重新从当前实测状态规划，再请求 MoveIt 执行，joint1 目标变化 0.01 rad（约 0.57°）。
工具最大允许单次变化 0.02 rad，默认只规划；`--execute` 才请求执行。
也可在 RViz 先 Plan，再按上述顺序开启门控，最后 Execute；不建议第一次用随机大目标。

成功执行后两个门控保持开启，可连续在 RViz 中 Plan / Execute，无需逐次重新授权。
空闲超过 0.5 秒仅清除上一条指令基准，下一次从新鲜实测状态校验，不发送保持或续发目标。
反馈过期、超限、故障、失能仍关闭网关门控；取消、执行失败仍撤销桥接许可。
节点重启后门控默认关闭；异常恢复后必须手动重新开启。Plan 可在门控关闭时进行。

停止后续指令：

```bash
ros2 service call /piper_trajectory_bridge/enable_execution std_srvs/srv/SetBool '{data: false}'
ros2 service call /piper_can_gateway/enable_outputs std_srvs/srv/SetBool '{data: false}'
```

停发不等于硬件急停/断电，不能撤回控制器已收到的目标。紧急情况使用设备自身停止措施。

## 控制约定

- 六轴使用精确 π/180 转换，不修改关节零点或放宽指令限位。
- 六轴反馈在模型边界外不超过 0.05° 时，仅将规划状态归一到边界；超过仍拒绝。原始反馈及状态 JSON 不变。这是本项目工程容差，不是厂家精度或校准结论。所有轨迹点和指令仍严格在 URDF 限位内。重启后必须重新规划，旧轨迹不自动修正。
- 原始 `/joint_states_single` 中 gripper 是总开口；MoveIt `/joint_states` 中 joint7 是单侧行程。
- 夹爪原始反馈保留负读数。仅在显示和执行前测量校验中允许边界外 0.5 mm 的测量噪声带，
  显示单侧行程限制在模型范围；这不是夹爪标零。超过此带仍拒绝，所有控制目标严格在模型范围内。
- 默认最大六轴指令速度 0.2 rad/s；夹爪单侧 0.005 m/s、总开口 0.01 m/s。
- 硬件 MoveIt 配置采用同一 URDF，设置低速度和加速度规划上限，使用真实时间。
- 桥接按轨迹时间进行线性插值，拒绝超速、非有限值、越限、重复名称和无效时间。
  它不是实时 ros2_control 硬件插件，也不提供经过物理验证的加速度跟踪保证。
- 两个 controller action 串行执行，并发目标会被拒绝。
- 反馈过期、示教模式、未使能、控制器故障、跟踪偏差或超时都会撤销许可。
  取消/失败停止继续发布，成功以实测关节到位判定，不能仅等待轨迹时长后宣布成功。
- 本配置没有相机 Octomap 输入，不包含自动环境避障；保留机器人自碰撞检查。

## 自动验证

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q piper_moveit_bridge/test/test_model.py
python3 piper_moveit_bridge/test/check_bridge_ros2.py
python3 piper_moveit_bridge/test/check_gateway_fake.py
python3 piper_moveit_bridge/test/check_moveit_pipeline.py
python3 piper_moveit_bridge/test/check_moveit_pipeline.py --bridge
```

这些测试使用隔离域；gateway_fake 替换 SDK 和 CAN，pipeline --bridge 显式禁用 CAN 网关。
它们不会执行实际机械臂运动。规划执行测试通过不等于真机运动验收通过。

## 示教录制结束后仍停留在 ctrl_mode=2

旧 agx_arm_ws 的 exit_teach_mode 服务包含全零目标、急停、重置和重新使能，
不能照搬为原地模式切换。本包提供独立 SetBool 服务
`/piper_can_gateway/reset_after_support`，仅调用 SDK ResetPiper。
**该操作可能立即失去电机保持力，机械臂必须先放稳并可靠支撑。**
`data: true` 表示操作者确认允许失电；不表示机械臂已经满足执行条件。
服务要求 read_only=false、输出门控关闭、新鲜状态、ctrl_mode=2、
teach_status=2、err_code=0。启动不会自动调用，不发全零目标、不自动使能夹爪或六轴。

在机械臂可靠支撑、录制已结束后，由操作者执行：

```bash
ros2 service call /piper_trajectory_bridge/enable_execution std_srvs/srv/SetBool '{data: false}'
ros2 service call /piper_can_gateway/enable_outputs std_srvs/srv/SetBool '{data: false}'
ros2 service call /piper_can_gateway/reset_after_support std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /piper_hardware/status --field data --once
ros2 topic echo /piper_bridge/readiness --field data --once
```

必须重新核实控制模式、电机状态及限位；success 只表示 SDK 调用完成，不保证模式切换。
若仍为示教模式，不自动重试或发送其他模式指令。joint2 越界不会被此服务修正。

## 使用本机下载的 SDK 源码

启动参数 sdk_path 可明确选择源码根目录，不修改系统 pip 安装：

```bash
ros2 launch piper_moveit_bridge moveit_hardware.launch.py sdk_path:=/home/zlq/piper_sdk can_port:=can0 read_only:=true rviz:=true
```

启动日志应显示 `/home/zlq/piper_sdk/piper_sdk/__init__.py`。
该目录当前为 0.7.0a1，仍使用 C_PiperInterface/V2，不自动切换 V3 或升级固件。
已检查接口签名并运行替代 CAN/SDK 的门控测试，尚需只读真机反馈验证。

## 预抓取目标：只规划，不执行

### 固定观察姿态：无需当前视觉候选

若已上使能、退出示教，但固件持续停在`ctrl_mode=0`且不响应轨迹，
先停止重试。`scripts/enter_can_control.py`提供独立本机SDK的模式切换诊断：
默认只读取ROS状态，`--switch`才发送一次`MotionCtrl_2(1,1,10)`。
要求两个门控关闭、无活动轨迹、六轴已使能、反馈正常、teach_status=0。
不发送关节/夹爪目标、复位、回零或使能。切换控制权属于真实硬件操作，
现场需留意固件是否恢复缓存指令，不可将“没有新目标”理解为必定不动。

```bash
python3 scripts/enter_can_control.py             # 只检查
python3 scripts/enter_can_control.py --switch    # 明确切CAN模式并验证反馈
```

需在项目根目录并加载ROS环境。脚本同时读取本地0x151总线回显与SDK状态，
未确认mode=1时退出，不循环切换、不自动执行轨迹。先检查输出再继续。

`observation_pose`恢复本机此前深度有效的六轴观察姿态，解决“只有先恢复观察位置才能识别，
但视觉执行又要求已有候选”的问题。它通过MoveIt关节目标重新规划，
不会加载历史轨迹，不会调用SDK回零/复位/示教接口，也不会操作夹爪。
目标保存在`config/observation_pose.yaml`，只适用于当前相机安装和桌面布局，
不保证物体任意位置都有有效深度。

保持机械臂驱动、MoveIt、桌面和相机碰撞模型运行。
新终端先加载Humble、工作空间和ROS_DOMAIN_ID=187。
两个门控关闭时先预览：

```bash
ros2 run piper_moveit_bridge observation_pose
```

默认只规划，不创建执行客户端。MoveIt RViz显示路径，
保存`calibration/observation_plan_preview.json`供查看，不作为执行输入。
规划仍检查当前反馈、URDF限位、桌面和相机附着模型、六轴速度不超过0.02rad/s和120秒时长。
允许当前视觉无候选；只规划也允许示教状态，但不允许执行中或开启门控。

确认完整路径和实物线缆间隙后，必须已退出示教、六轴使能、反馈新鲜，
read_only=false、ctrl_mode=0或1、teach_status=0、reason为空，再明确执行：

```bash
ros2 service call /piper_can_gateway/enable_outputs std_srvs/srv/SetBool '{data: true}'
ros2 service call /piper_trajectory_bridge/enable_execution std_srvs/srv/SetBool '{data: true}'
ros2 run piper_moveit_bridge observation_pose --execute
```

最后一条会实际移动机械臂。它重新规划、检查起始关节未变化，
通过控制器动作执行，监测反馈和门控，Ctrl+C请求取消而非硬件急停。
只到观察位并保持，不会继续预抓取或抓取。
到位后先检查`/graspgen/status`和`/grasp_preview/status`，物体坐标与有效深度仍需确认。
如需回到只规划检查，先关闭两个软件门控：

```bash
ros2 service call /piper_trajectory_bridge/enable_execution std_srvs/srv/SetBool '{data: false}'
ros2 service call /piper_can_gateway/enable_outputs std_srvs/srv/SetBool '{data: false}'
```

### 显式执行预抓取悬停

仅在完整预览路径、相机支架外形及实物线缆间隙已检查后使用。
`execute_pregrasp --execute`会读取新鲜目标重新规划，不加载保存的JSON轨迹，
不自动打开门控、不使能、不退出示教、不发送夹爪命令，也不下降。
当前TCP仍使用暂定的135.8mm，悬停点为可见物体表面中心沿基座+Z上方100mm，
并不保证位于整个物体最高点以上100mm。

先在原真机launch终端Ctrl+C并重新启动可写网关；相机、识别、手眼TF和预览保持运行：

```bash
cd ~/GraspGen
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=187
ros2 launch piper_moveit_bridge moveit_hardware.launch.py \
  sdk_path:=/home/zlq/piper_sdk can_port:=can0 read_only:=false rviz:=true
```

另一个终端也加载以上环境，然后恢复场景。此时两个门控仍应关闭：

```bash
ros2 run piper_moveit_bridge add_table
ros2 run piper_moveit_bridge add_camera
ros2 topic echo /piper_hardware/status --field data --once
```

必须有新鲜反馈，`read_only=false`、`ctrl_mode=0或1`、`teach_status=0`、
`err_code=0`、六轴使能且`reason`为空。若不满足，先处理原因，不继续打开门控。
待机模式0可作为起始状态，网关会在收到第一条校验通过的轨迹指令时一起发送CAN位置模式及对应关节目标；
不是提前单独切模式。执行开始1秒后仍未进入模式1则请求取消。

```bash
ros2 service call /piper_can_gateway/enable_outputs std_srvs/srv/SetBool '{data: true}'
ros2 service call /piper_trajectory_bridge/enable_execution std_srvs/srv/SetBool '{data: true}'
ros2 run piper_moveit_bridge execute_pregrasp --execute --wait-target 20
```

最后一条会实际移动机械臂。执行规划速度缩放固定为0.08、加速度缩放0.05，
还使用桥接相同的轨迹校验器，检查六轴轨迹速度不超过0.02rad/s、时长不超过120秒。
只规划命令仍使用0.05速度缩放。超过120秒的轨迹会在发送执行目标前拒绝。
执行前检查桌面和相机附着物存在、目标位置变化不超过10mm、接近轴变化不超过5度、
起始关节变化不超过0.005rad；不满足时拒绝发送执行目标。
当前桌面配置为base_link Z=0，执行还要求视觉表面中心严格高于桌面。
候选中心在桌面或以下时必须检查视觉深度，不能靠修改TCP补偿。
保存的预览JSON包含规划时的视觉状态、预览状态及起始关节，便于核对实际选中的目标。
悬停阶段锁定本次规划的腕部水平朝向，不因近圆形可见表面的PCA水平朝向抖动而重新跟随；
该策略仅适用于不接触物体的悬停，不代表实际夹持方向已经确认。
提交后锁定本次目标，不随移动相机产生的新候选追踪运动。
若竖直朝下姿态无逆解，可先显式验证小倾角悬停：

```bash
ros2 run piper_moveit_bridge plan_pregrasp --wait-target 20 --tilt-deg 10
```

`--tilt-deg`范围0～20度，默认0，不会自动改变姿态。
夹爪接近轴沿目标远离基座方向倾斜，绕固定TCP旋转并重新计算link6目标，
不会把法兰固定后直接旋转，也不改变悬停TCP位置和高度。
采用当前135.8毫米暂定TCP；若预览配置的TCP距离不同则拒绝。
查看倾斜后的实际规划以MoveIt RViz轨迹为准，原TCP预览窗口仍显示原始竖直候选。
用户确认完整路径后，可对执行命令加相同`--tilt-deg 10`；
仍需退出示教且满足全部门控、反馈和碰撞条件。仅悬停，不代表已验证倾斜下降抓取。
只执行六轴arm轨迹，禁止夹爪及多自由度轨迹。
执行成功须同时有控制器成功结果及末端关节反馈匹配。
途中反馈/门控丢失、超时或Ctrl+C会请求取消；取消不是物理急停，也不下使能。
若执行目标确认超时，先检查控制器状态，不要重复提交运动。

测试结束可主动关闭两个软件门控（不会下使能电机）：

```bash
ros2 service call /piper_trajectory_bridge/enable_execution std_srvs/srv/SetBool '{data: false}'
ros2 service call /piper_can_gateway/enable_outputs std_srvs/srv/SetBool '{data: false}'
```

### 相机与支架碰撞包围盒

保持手眼TF、相机驱动和MoveIt运行，关闭硬件输出及轨迹执行两个门控，然后执行：

```bash
ros2 run piper_moveit_bridge add_camera
```

该命令只修改规划场景，不使能、不切换控制模式、不发送关节或夹爪目标。
使用用户确认的相机及支架总外形125×30×50毫米，每侧增加3毫米余量；
`camera_link`的外壳偏移来自本机官方
`/opt/ros/humble/share/realsense2_description/urdf/_d455.urdf.xacro`。
相机根坐标不是外壳中心。总高50毫米的盒子顶面沿用D455外壳顶面，向下覆盖支架，
这是需要在实物/RViz中核对的建模假设，不是新增的实测安装位置。
读取当前`link6 -> camera_link`变换，并校验实时光学坐标变换与已验收的手眼文件一致。
模型作为`graspgen_camera_and_mount`附着在`link6`，随机械臂运动参与碰撞检查。
只允许与安装连接`link6`、`gripper_base`接触，保留其他关节、两指和桌面的碰撞检查。
重复运行会更新同一ID，不会重复添加。不会清除桌面或改变已有碰撞矩阵。
MoveIt重启后需要重新添加。

在MoveIt RViz的MotionPlanning显示中展开`Scene Robot`，启用`Show Robot Visual`，
查看附着包围盒；仅显示URDF的RobotModel不会自动显示规划场景的附着物。
盒子应完整包住相机和支架，但不应穿入夹指。线缆仍未建模。
配置在`config/workcell_measurements.yaml`，读回及碰撞报告保存为
`calibration/camera_collision_report.json`。物理覆盖是否准确始终需人工核对；
程序不会把添加成功当作几何验收成功。若当前姿态碰撞，命令非零退出，
模型保留在场景中供查看；不要将夹指加入touch_links或关闭碰撞检查来通过。

确认显示位置及当前无碰撞后，再进行只规划：

```bash
ros2 run piper_moveit_bridge plan_pregrasp --wait-target 20
```

先保持相机、真实机械臂反馈、MoveIt、handeye publish_mount、graspgen_vision、
graspgen_handeye preview运行。两个输出门控必须关闭，允许示教/只读状态做规划检查。
先确认真实关节反馈在模型限位内，并让机械臂停稳。

```bash
cd ~/GraspGen
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=187
ros2 service call /piper_trajectory_bridge/enable_execution std_srvs/srv/SetBool '{data: false}'
ros2 service call /piper_can_gateway/enable_outputs std_srvs/srv/SetBool '{data: false}'
ros2 run piper_moveit_bridge plan_pregrasp
```

视觉候选短暂丢失时，该命令默认等待最多10秒，恢复后使用新鲜目标规划；
可用 `--wait-target 20` 调整等待时间（0～60秒）。超时仍退出，不复用过期目标。
提交前会等待3秒让服务请求/响应通道完成发现，然后再检查新鲜目标。
规划响应超时会单独提示 `/plan_kinematic_path` 通信异常，不再误报相机未运行。

该命令接收 `/grasp_preview/pregrasp_link6`，请求 `/plan_kinematic_path`，
以arm组、link6位置和姿态约束规划。位置容差3毫米，姿态各轴0.03弧度，速度/加速度比例5%。
成功后发布 `/display_planned_path`，可在原MoveIt RViz的Planned Path中查看动画。
当前TCP专用预览窗口只显示目标，不自动显示规划路径。
最新轨迹及目标保存到 `calibration/pregrasp_plan_preview.json`。

没有执行客户端或运动指令发布器，不支持 `--execute`。不会打开门控。
规划成功只证明当前规划场景下找到路径；TCP仍是模型预览值，桌面/瓶子/相机支架
尚未完整建模，不能据此执行下降或闭爪。目标只是高于可见表面中心100毫米，
不保证高于整个物体顶部。实际执行需先完成TCP和场景几何验证。

### 按毫米保持姿态下降（不闭爪）

`cartesian_step` 从当前关节状态计算 link6 位姿，沿 `base_link` Z 轴平移，
保持当前末端朝向。固定朝向下 TCP 与法兰的位移相同，无需改变 TCP 偏置。
默认仅规划，允许当前门控保持打开；不创建执行客户端。

```bash
ros2 run piper_moveit_bridge cartesian_step --down-mm 80
# 只有检查 RViz 轨迹和实物间隙后，才执行下面的实际运动：
ros2 run piper_moveit_bridge cartesian_step --down-mm 80 --execute
```

正数下降，负数上升，单次绝对距离不超过80毫米。执行命令总是重新规划，
不读取旧预览轨迹。相同命令每次从当前位姿计算；不要重复执行80毫米下降。
要求只有一个网关和桥接、有效CAN反馈、夹爪总开口至少40毫米、桌面和相机
碰撞模型已添加；实际执行还要求两个门控开启。不会开关门控或发夹爪指令。
使用1毫米笛卡尔采样和碰撞检查，仅接受完整路径；逐点FK验证横向偏差不超过
1毫米、朝向偏差不超过1度，并检查末端高度。逐段关节速度不超过0.01rad/s，
总时长受现有120秒上限约束。仅对现有场景检查，目标物体和线缆未完整建模；
此命令不是自动抓取流程，不保证夹指与瓶盖、瓶身的接触安全。

水平远离基座可使用 `cartesian_step --outward-mm 20`，与 `--down-mm` 互斥。
方向取当前 TCP 在 base_link XY 平面的径向方向，使用当前135.8毫米TCP配置，
保持高度、倾角和夹爪开口。正数向外、负数朝基座，绝对值不超过20毫米且不能为0。
默认仅规划；明确加 `--execute` 才运动。每次均相对当前位置，不能重复执行
同一个20毫米命令来确认结果。瓶子未完整建模，规划成功不保证与瓶身无接触。

夹持后的小幅抬升需显式使用持物模式，保持当前开口，不要求重新张开：

```bash
ros2 run piper_moveit_bridge cartesian_step --down-mm -10 --holding-object
ros2 run piper_moveit_bridge cartesian_step --down-mm -10 --holding-object --execute
```

持物模式只允许单次不超过10毫米的竖直上升，禁止下降或横移；仍要求实时
反馈、门控、场景和起点一致，且不会发送新的夹爪目标。它不检测夹持力或滑落，
也未把瓶子附着为碰撞物体，因此仅用于确认是否能随夹爪小幅离桌，不用于搬运。

### 成功抓取修正与局部自动抓取

中文完整操作见工作区 `doc/AUTO_GRASP_ROS2.md`。新增命令：
`record_grasp`只读记录用户确认的成功夹持；`auto_grasp`默认仅规划，
`auto_grasp --execute`执行局部经验修正抓取和1厘米试抬；`tcp_calibration`
用于固定点样本采集与TCP平移拟合。135.8改为143.8毫米只差8毫米，不能代替
80毫米下降。当前自动流程限定同一瓶子、原观察位、参考位置附近50毫米，
实物接触和滑落仍须人工确认；不自动运输或松爪。

### 当前速度

2026-10-22已统一提速：观察位/靠近使用MoveIt速度缩放0.32、加速度缩放0.15，
程序关节验证上限0.08rad/s；直线移动8mm/s并受0.04rad/s关节限速；自动收爪
总开口8mm/s、至少2秒。网关总上限未提高。共享配置见`speed_settings.py`，
完整说明见`doc/AUTO_GRASP_ROS2.md`末尾。本节覆盖前文早期0.02rad/s等旧数值。
