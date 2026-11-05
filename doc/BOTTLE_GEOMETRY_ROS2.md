# 32 mm 瓶子的几何抓取流程（2026-10-28）

本流程针对用户实测的同一个直立瓶子：直径32 mm、总高120 mm，平底直接放在桌面，选择距桌面60 mm的中间位置夹持。用户最新实测底座底面高于瓶子桌面11.8mm，因此桌面为 `base_link` Z=-0.0118m。不同桌面高度、倾斜瓶子、透明反光、未知物体不在当前验证范围。

2026-10-29 已完成软件修改、118项单元测试、3项隔离ROS检查、三个包的重建与安装文件一致性检查；尚未在真机上验证新TCP/新拟合抓取。不会自动开门控、上使能、回零或启动动作。

## 2026-11-05 新手眼结果与实测桌面基准

已备份旧标定为 `calibration/handeye_result_before_20261105_table11.yaml`，活动文件 `handeye_result.yaml` 选用32样本PARK候选。新标定在此前三个独立观测中的最大空间漂移约4.9mm，当前额外独立姿态离新参考点约1.5mm。当前标记Z=-11.64mm与用户最新实测桌面Z=-11.8mm相符，残差约0.16mm；不保证任意位置精度都达到该值。

没有人为给点云增加24mm或11.8mm；所有点仍按图像时刻TF变换。只是把尺寸、夹持高度和碰撞体的基准改为实际桌面。

`add_table`现在读取workcell_measurements.yaml，而非硬编码Z=0，校验后读回实际顶面。实际桌面顶面为-11.8mm、50mm厚碰撞盒中心Z=-36.8mm。另保留`graspgen_base_height_guard`至基座Z=0的保守禁入层，其他连杆和夹指不能进入；仅固定基座接触及已知目标瓶子与该虚拟支撑层的重叠被允许，避免因未测支撑区域而削弱原有保护。它不是对垫板真实长宽的测量。

生成器检查实时相机TF与选定手眼文件是否一致，并在候选里发布手眼哈希；执行器检查哈希、table_z_m与场景实际顶面。旧进程、旧TF或旧碰撞场景不能静默混用。

**必须停止旧完整系统和标定采集终端，再重新启动完整系统**，使新手眼TF、视觉代码和桌面/相机碰撞体一起更新：

```bash
cd ~/GraspGen
./scripts/start_graspgen.sh generator:=bottle_cylinder
```

启动保持门控关闭。先正常退出示教，确认teach_status=0、反馈新鲜无错误，再检查 `/graspgen/status` 中valid、table_z_m与center_m，运行 `bottle_grasp` 只规划。不要直接运行旧经验auto_grasp；它绑定旧手眼哈希，会要求重录。

新手眼改变了相机位置/方向，旧相机碰撞包围盒也必须重新添加。完整启动会依次加入桌面和相机；若启动日志失败，先处理失败，不能直接执行抓取。

## 小瓶检测与桌面读回修复（2026-11-05）

默认YOLO推理尺寸已从640改为960，采集图像仍是640×480，confidence仍为0.45。对当前现场同一帧，640时瓶子分数约0.274、960时约0.821；这是这一帧的对照，不是成功率保证。增大网络输入能改善本场景小目标检测，但不创造新的相机深度细节。

完整启动可显式指定：`./scripts/start_graspgen.sh generator:=bottle_cylinder imgsz:=960`。若只重启视觉：`ros2 launch graspgen_vision vision.launch.py generator:=bottle_cylinder imgsz:=960`，先停止旧视觉节点避免重复发布。

`add_table`读回校验已修复：MoveIt可能将第一个形状的位置提升到CollisionObject.pose；现在以object.pose × primitive_pose计算实际高度，不再仅检查primitive_pose。已在独立ROS域的真实MoveIt中通过桌面和保守禁入层的写入/读回验证，没有连接硬件或执行运动。

## 最新流程：直接抓取、随机放到同桌面（2026-11-05）

按用户要求取消瓶子正上方100mm悬停以及固定竖直下降。默认 `bottle_grasp` 现在先规划完整流程：

```text
开着的夹爪经MoveIt避障直接到夹持点
 -> 闭合并检查接触 -> 试抬10mm
 -> 再抬高80mm -> 搬运至随机放置点上方
 -> 下降放稳 -> 张开到40mm -> 夹爪向上退出50mm
```

抓取位置是同一个TCP中心，不因倾斜改变。尝试两种竖直闭合方向，以及±10/15/20度倾斜，先检查IK再规划完整路径；不再把距基座50cm当成物理可达上限。是否可达由URDF限位、IK、碰撞和整段轨迹检查决定。直接接近时夹指与瓶子的碰撞检查保持开启，到位后才允许预期的闭爪接触。

放置点不使用2×2m保守桌面盒的边界。视觉新增 `/graspgen/scene_cloud`，随机点从当前图像时刻实际观测到的桌面中选择：整块约50mm半径区域都有足够桌面点、排除可见高物体投影、远离当前瓶子至少120mm、避开基座附近，并通过持物搬运/下降/张开/退出的完整规划。看不到足够空闲桌面则停止，不能凭空随机坐标。实际未观测或未加入MoveIt的其他障碍仍不保证覆盖；工作区需保持清空。

验证结果：151项单元测试通过；独立ROS域的真实MoveIt完整规划通过（合成反馈/桌面点云，无硬件网关、CAN或执行客户端）。测试在同一抓取中心选择+20°姿态，成功规划闭爪、持物抬升/搬运/放置、释放退出。报告：`doc/reports/direct_random_place_plan_test_20261105.txt`。这不代表真机抓取已经验证。MoveIt轨迹会统一放慢至桥接器速度上限，保留位置、速度和加速度之间的时间关系。

默认不创建执行动作客户端：

```bash
ros2 run piper_moveit_bridge bottle_grasp
# 等同：ros2 run piper_moveit_bridge auto_grasp --mode bottle
```

只有检查RViz完整轨迹和日志中的随机落点后，才显式执行；执行命令会重新规划，随机点也可能不同，因此应持续观察现场：

```bash
ros2 run piper_moveit_bridge bottle_grasp --execute
```

可选 `--reach-only`：只规划/执行到夹持位置，不闭爪；`--pick-only`：只到位、闭爪、试抬10mm，不搬运释放。这两个选项互斥。所有实际动作都还需要 `--execute` 和正常的两道门控。

闭爪后每段移动保留夹爪接触反馈检查，失去接触线索会取消，不自动释放。只有到达已规划的放置点后才张爪，张开确认后才把碰撞体从夹爪移到放置位置；异常期间可能仍持物则保留附着模型。完整操作后放置碰撞体继续保留，再次抓取同一瓶子时会校对位置再更新。

**同时定位了旧场景误碰撞**：新手眼下相机包围盒的3mm额外余量与link8重叠约0.39mm，使当前实际姿态也被判碰撞。只读假设检查中2mm余量在0..70mm开口采样点均通过。已改为2mm额外余量，实际125×30×50mm外形没有缩小，夹指碰撞检查没有关闭。重启后须重新添加相机模型。

这些改动需要重启视觉（发布完整场景点云）和重建相机碰撞模型。建议停止旧完整系统，再运行 `./scripts/start_graspgen.sh generator:=bottle_cylinder imgsz:=960`。

下方早期步骤保留作为历史说明；涉及“先悬停再下降”“默认只试抬不搬运”的描述以本节为准。

## 1. 改动后的数据链路

```text
D455 RGB + 对齐深度 + 内参
  -> graspgen_vision / node.py：原有时间同步、YOLO分割、掩膜深度过滤
  -> /graspgen/object_cloud + /graspgen/segmentation/objects
  -> graspgen_grasp_generator：按frame/sec/nanosec严格配对
  -> 图像采集时刻的TF转换到base_link
  -> bottle_geometry.py：已知半径的瓶身圆弧拟合
  -> /graspgen/status、selected_grasp、grasp_poses、markers
  -> bottle_grasp：TCP到link6换算、碰撞场景、MoveIt规划、闭合检测、试抬
```

分割节点不再计算抓取姿态。抓取节点不加载YOLO，也没有运动客户端。仍支持 `generator:=auto`（YOLO用PCA，彩色方块用已知尺寸拟合）、`pca`、`known_cube`；未知算法直接报错，没有假装加载抓取网络。

新增元数据记录同一帧的物体编号、类别、分数、点数及点云有效性。物体编号不是跨帧追踪ID。两话题必须拥有完全一致的坐标系和整数时间戳，缓存有长度限制，不匹配或过期时不生成新目标。

## 2. 已配置的尺寸与TCP

`calibration/tcp_config.json`：

- translation_m = [0, 0, 0.1358]，相对link6。
- 2026-10-29按用户要求恢复指尖高度处两指之间的中心；用户确认实物与模型一致。
- 保留未做多姿态标定的标志，不把模型测量冒充独立标定。

`calibration/bottle_geometry.json`：

- diameter_m = 0.032；total_height_m = 0.120；grasp_height_m = 0.060（距桌面）。
- table_z_m = -0.0118，因此夹持中心在base_link下的Z=0.0482m，瓶顶Z=0.1082m。
- body_bottom_m = 0.045、body_top_m = 0.075 是选取的检查区域，不表示用户测过整段直筒。
- 实际拟合使用抓取高度上下各10 mm。该区域必须在图像中具有足够有效深度且近似直筒。

拟合采用已知16 mm半径，要求至少45度可见圆弧、RMS不超过2 mm、足够内点、上下高度的圆心基本一致，并检查点云是否明显超出瓶子尺寸。未满足时会在状态中说明原因。识别分数不是抓取成功概率。

## 3. 启动与重启

安装文件已构建。当前运行的旧节点不会自动更新；先停止原来的完整系统终端（Ctrl+C），再启动一次，避免重复网关、TF或视觉节点。

```bash
cd ~/GraspGen
./scripts/start_graspgen.sh generator:=bottle_cylinder
```

启动脚本仍要求can0已经启用，仍保持运动门控关闭，自动加入桌面和相机支架碰撞模型。它现在可转发launch参数。

如果MoveIt、相机、手眼TF和预览已经正常运行，只替换视觉终端时，先停止旧视觉launch，再执行：

```bash
cd ~/GraspGen
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=187
ros2 launch graspgen_vision vision.launch.py generator:=bottle_cylinder target_classes:=bottle
```

不要同时运行两份视觉launch。新TCP要求预览节点重启后加载。相机尽量从有效的0.35..1.5米范围观察瓶子中部，保持瓶子直立、无移动、工作区内只有一个目标瓶子。

## 4. 检查候选（不会移动）

```bash
source ~/GraspGen/install/setup.bash
export ROS_DOMAIN_ID=187
ros2 topic echo /graspgen/status --field data --once
ros2 topic echo /grasp_preview/status --field data --once
```

期望 `/graspgen/status`：valid=true、frame=base_link、generator=bottle_cylinder。
目标对象：position_semantics=bottle_grasp_center、diameter_m=0.032、grasp_height_m=0.060。center_m的Z应为0.0482，不再是前表面平均位置。

预览仍保留 `surface_base_m` 字段兼容旧接口，同时通过 `target_semantics` 说明它实际上是瓶身夹持中心。黄色点表示该中心，绿色点表示上方悬停位置。新执行器直接检查生成器状态，不把预览的固定高度当作瓶身拟合。

## 5. 先规划完整抓取

夹爪应为空且实际总开口至少40 mm；张爪仍通过既有MoveIt夹爪流程操作，本命令不自动张爪，以免误松开已持物体。

```bash
ros2 run piper_moveit_bridge bottle_grasp
# 同等入口：ros2 run piper_moveit_bridge auto_grasp --mode bottle
```

它会等5个稳定新帧，检查拟合中心漂移不超过3 mm，然后：

1. 将目标瓶子按直径32 mm、高120 mm圆柱加入场景，半径额外加2 mm。
2. 用OMPL规划上方接近；此阶段目标与夹指仍检查碰撞。
3. 允许目标与两个夹指发生预期接触，保持目标与其他机械臂/相机的碰撞检查。
4. 规划笛卡尔直线下降和闭爪过程中各开口的机器人状态。
5. 临时把瓶子按规划抓取姿态附着到link6，预检持物抬升10 mm，再恢复静止瓶子的场景。
6. 在RViz显示轨迹并保存日志，不创建执行动作客户端。规划结束移除本次临时目标。

半径工作区先限定为离基座轴80..500 mm，还必须通过MoveIt逆解和碰撞检查。此模式不再使用旧的50 mm参考点邻域或固定成功补偿；不能据此认为工作区任意位置都可达。

## 6. 真机执行

只有确认候选位置、RViz轨迹和实际工作区合理，且夹爪为空、已张开后，才运行以下命令。两次SetBool只是授权门控；最后一条会实际运动。

```bash
ros2 service call /piper_can_gateway/enable_outputs std_srvs/srv/SetBool '{data: true}'
ros2 service call /piper_trajectory_bridge/enable_execution std_srvs/srv/SetBool '{data: true}'
ros2 run piper_moveit_bridge bottle_grasp --execute
# 同等入口：ros2 run piper_moveit_bridge auto_grasp --mode bottle --execute
```

执行会重新规划。最终提交前再检查目标及机械臂起点未变化、配置文件未变化；提交后固定目标，不追逐眼在手上相机近距离的抖动或深度丢失，因此运行中不要移动瓶子。

闭合目标先设为30 mm，预期瓶子接触开口32 mm。它使用实际开口32±1.5 mm、负向闭合力矩至少0.25 Nm、反馈新鲜且连续稳定的条件确认接触线索，不改变网关原有力矩命令或速度上限。若实物/编码器开口与测量不一致会停止，不应强行降低阈值跳过。

只有接触检查通过才根据实际关节状态重新规划并执行10 mm试抬。电机、反馈、场景、控制器失败仍会中止；软件动作成功不等于物理夹稳，仍需观察瓶子是否真正离桌且不滑落。

闭合开始后会保留附着瓶子的碰撞模型；失败时可能已经持物，不自动松爪。再次抓取会拒绝覆盖这个模型。确认瓶子已放下且夹爪为空后，在MoveIt RViz的场景/附着物体列表移除 `graspgen_target_bottle`，再开始下一次；不要在持物时删除它来绕过检查。当前不自动搬运或释放。

圆柱模型不表达瓶盖凸出、软瓶变形、线缆和桌面其他物品；这些额外障碍仍需自行添加。没有通过新流程真机测试前，不宣称成功率提升或任意物体可抓。

## 7. 旧模式与日志

原 `auto_grasp` 默认仍是经验模式。旧profile绑定135.8 mm TCP，本次恢复后可通过TCP一致性检查，但其他模型/证据/位置/模式检查继续生效。旧经验模式必须搭配PCA表面候选，不能接收瓶身拟合中心；不得绕过其余检查。

本流程不读取旧经验补偿，日志为 `calibration/grasp_runs/bottle_*.json`，记录源图像状态、TCP、配置哈希、候选中心、轨迹和执行阶段。失败不自动重放下降、闭合或抬升。

## 8. 修改的主要文件

- `graspgen_vision/graspgen_vision/node.py`：分割/图像同步/点云生产者。
- `frame_pairing.py`：有界精确时间配对。
- `grasp_generator_node.py`、`generators.py`：独立候选节点与PCA/方块兼容后端。
- `bottle_geometry.py`：已知半径瓶身拟合和质量检查。
- `piper_moveit_bridge/piper_moveit_bridge/bottle_targets.py`：TCP与link6、悬停、开口计算。
- `bottle_scene.py`：目标圆柱、接触权限、附着物及清理。
- `bottle_workflow.py`：完整规划和可显式执行的动作阶段。
- `grasp_workflow.py`：新增 --mode bottle；防止经验模式误用拟合中心。
- 两个包的setup.py、vision/grasp_system启动文件、start_graspgen.sh、preview.py：入口和接口衔接。


## 2026-11-05 实际瓶身接触修正

用户确认+20°姿态下夹住瓶身，并成功试抬。101帧反馈开口均为39.2mm、闭合力矩约-1.05Nm。瓶身物理直径仍为32mm，不能把夹爪反馈开口与圆柱直径等同。`bottle_geometry.json`新增独立`gripper_contact`，绑定同一32mm瓶子和+20°候选；其他姿态暂不沿用这一个样本。

控制目标由30mm改为37.2mm，预期受阻接触开口39.2mm（仍保留±1.5mm开口、反馈新鲜、闭合方向力矩和稳定时间检查）。力矩数值是判定参考，没有增加硬件力矩指令。预开/释放改为47.2mm，持物碰撞预检按40.7mm最宽接受值进行。物体圆柱仍按32mm尺寸建模，TCP135.8mm不变。配置文件哈希纳入原有执行前一致性检查。

157项单元测试通过，使用新接触配置的独立真实MoveIt完整规划通过，无执行客户端或CAN。

此修改用于下一次新抓取；当前已持物状态不要重跑完整抓取，也不需要重启正在持物的网关。新CLI进程会读取新代码和配置；旧视觉几何参数不变。恢复当前持物搬运需要单独从当前状态规划，不能重放接近/闭爪流程。


## 夹爪开闭计时优化

`speed_settings.py`中的`GRIPPER_OPEN_SPEED`与`GRIPPER_CLOSE_SPEED`分别控制张开/闭合总开口速度，目前均为0.01m/s，`GRIPPER_MIN_DURATION=0.5`。新流程不再使用旧`GRIPPER_TOTAL_SPEED`。动作时间使用sec+nanosec，不再向上取整到整秒。例如39.2→47.2mm张开8mm规划0.8秒，而非原2秒最短等待。总开口请求超过10mm/s或URDF速度上限时会降低实际规划速度并打印提示，未提高桥接器/网关限速。日志显示实际时长和速度上限。闭爪稳定接触检查不变。

164项测试通过；已构建安装。无需重启正在持物的硬件节点，重新source install/setup.bash后启动的新抓取命令读取更新。没有发送真机动作，实际硬件耗时尚未验证。


### 后续实际提速至20mm/s

用户反馈10mm/s仍慢，现将总开口上限`GRIPPER_OPENING_CAP`统一设为0.02m/s，开闭速度均0.02m/s，最短0.2秒；网关默认max_opening_velocity=0.02，桥接器单指默认max_jaw_velocity=0.01。未改变力矩、接触、碰撞或机械臂速度。8mm张开计划0.4秒，10mm闭合计划0.5秒。需要在不持物、物体已支撑时重启完整系统才能更新常驻网关/桥接器；显式ROS参数覆盖会优先于默认值，可用ros2 param get查询。新速度尚未真机验证。


### 搬运分段起点修正

真机闭爪和试抬成功后，提交后续搬运时遇到起点不一致。保留原先完整路径预检，执行carry_lift/transfer/place前分别读取最新反馈，针对固定目标重新规划该段；不再直接提交以前一段预测终点为起点的轨迹。不修改起点容差、不移动固定落点、不跳过碰撞或持物检查。165项回归测试通过，新增断言检查每段搬运前重新获取状态并规划。当前持物失败状态不自动重放完整抓取。


### 多轮抓取与手动移瓶后的场景同步

若保留的瓶子世界模型与当前视觉目标相距超过30mm，新一轮会读取与稳定目标同时间戳的整幅场景点云，检查旧位置周围瓶身半径加15mm的桌面区域。检查范围取10mm栅格向上覆盖：每个圆形覆盖栅格至少2个桌面点（高度误差±5mm），没有高于桌面8mm的障碍投影，才允许更新该ID的瓶子模型。未知、遮挡、视野外、仍占用均不能视为空，提示让旧位置和新瓶子一起进入视野。不是多物体跟踪，场景多瓶仍需明确建模。

只更新graspgen_target_bottle，桌面、相机和其他对象不变；已附着的可能持物对象仍拒绝覆盖。旧位置已确认空时不再恢复旧模型。下一次CLI调用读取新代码，无需重启MoveIt/相机/网关。168项测试通过，独立真实MoveIt中人为保留上一轮旧瓶子模型，自动空桌面验证和完整抓放预规划通过；没有真机动作。
