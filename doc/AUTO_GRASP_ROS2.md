# ROS 2：成功抓取修正记录与局部自动抓取

本流程已记录2026-10-22用户确认成功的那只瓶子。它复用新鲜视觉目标，自动规划
靠近、下降、收爪和10毫米试抬；尚不是任意物体抓取或自动搬运。默认仅规划。

## 135.8毫米和143.8毫米的区别

两者只差8毫米，不是80毫米。TCP是法兰到夹持中心的固定变换，不能充当下降量。
末端倾斜时，修改TCP还会改变水平位置。当前TCP继续按135.8毫米暂定，
悬停高度、抓取高度和TCP分别管理。不能用成功夹住一次来独立标定TCP。

## 已保存的证据

- `calibration/grasp_profile.json`：同一瓶子的经验修正配置。
- `calibration/grasp_success_*.json`：不可被下一次预览覆盖的成功抓取记录。
- 原始`pregrasp_plan_preview.json`仅作为本次成功会话的视觉参照，不作为可执行轨迹。
- `calibration/grasp_runs/*.json`：每次自动运行的目标快照、预规划轨迹、阶段结果和失败原因。

记录过程只读取实时关节、开口和MoveIt FK。将已完成的10毫米试抬扣除后，
以原始视觉表面中心为参考，在水平径向、切向和竖直方向记录偏移，保留成功
抓取的朝向和实测开口。没有把用户输入的35毫米当成实际开口。
参考手眼标定和URDF使用SHA256绑定；TCP变更需要重录，不能沿用旧修正。

当前记录的修正约为：径向向外10.94毫米，切向0.87毫米，表面中心以上19.90毫米；
实测夹爪总开口44.5毫米。向下80毫米是从原来100毫米悬停位置到抓取位置的运动，
不是工具长短误差。单个样本无法区分手眼误差、视觉表面偏差和工具中心误差。

## 每个新终端

```bash
cd ~/GraspGen
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=187
```

相机、识别、手眼TF、preview、MoveIt网关和桥接需保持运行，网关与桥接各只能有一个。
重启MoveIt后先在门控关闭时执行`add_table`和`add_camera`。

## 下一次自动抓取

1. 当前还持瓶时，不运行`auto_grasp`。先把瓶子可靠放回桌面、再松爪。
   本命令不自动放置或释放，不能凭旧截图判断物体已落桌。
2. 用已验证的`observation_pose --execute`回到记录的观察姿态；当前命令
   不自动恢复观察位，避免持物时自动回位。参考关节与记录值相差需小于0.03弧度。
3. 确认空爪，将开口设置到至少43毫米（推荐70毫米）。保持瓶子不动，
   使用同一瓶子，距离原示教位置不超过50毫米，视觉观察宽度应与记录相差不超过8毫米。
4. 先仅规划，不会收爪或移动，也不要求改变当前门控：

```bash
ros2 run piper_moveit_bridge auto_grasp
```

显示`Full local grasp planned`和`PLAN ONLY`后，在MoveIt RViz检查轨迹。
完整预检包含悬停路径、固定姿态下降路径、夹指收拢时的机器人/环境碰撞，以及10毫米抬升路径。
没有任何执行动作会在全段规划和目标二次检查完成之前发出。

5. 空间无障碍、瓶子位于两指目标路径之间时，打开软件门控：

```bash
ros2 service call /piper_can_gateway/enable_outputs std_srvs/srv/SetBool '{data: true}'
ros2 service call /piper_trajectory_bridge/enable_execution std_srvs/srv/SetBool '{data: true}'
```

两条均需`success=True`。下面会实际运动和收爪，每次新执行都会重新识别、重新规划：

```bash
ros2 run piper_moveit_bridge auto_grasp --execute
```

程序先检查至少5个新鲜视觉帧（覆盖至少0.3秒，位置变化不超过5毫米），
随后使用经验修正计算抓取TCP和法兰目标，保持记录朝向相对径向的关系，
在抓取点上方80毫米规划靠近，再固定姿态下降。收爪到记录的实测开口，
控制器成功且反馈一致后才允许竖直试抬10毫米。不会把`sample_count=1`
当成通用标定，也不会迁移到远处或其他类型物体。

6. 看到控制器序列完成后仍须目视确认瓶子真的离桌、没有滑落。
   程序只确认位置和动作状态，不具备可靠接触/滑移检测。日志始终保留
   `physical_grasp_verified: false`，不以动作返回成功冒充抓取成功。

失败时停止后续阶段，取消尚未完成的动作；不会自动松爪、复位或下使能。
瓶子、线缆未完整建模，已检查桌面和相机包围盒。接触瓶身不是已建模碰撞证明，
因此仅支持局部重复抓取和1厘米试抬，不自动横向搬运。

## 记录更多成功抓取

必须先实际抓稳并验证试抬。保持静止、提供实际试抬距离；不要对空爪运行确认参数。
原始视觉参照文件需来自同一次成功会话、同一瓶子，不能在失败试验后覆盖而不核对。

```bash
ros2 run piper_moveit_bridge record_grasp --confirmed-held --lift-mm 10 --close-command-mm 35 \
  --reference ~/GraspGen/calibration/pregrasp_plan_preview.json \
  --output ~/GraspGen/calibration/grasp_profile_second.json
```

旧profile默认不覆盖；新样本独立保存，不会把不同物体和误操作自动平均成一个偏移。
测试其他profile用`auto_grasp --profile /绝对路径/grasp_profile_second.json`。
当前记录命令限定原始10度倾角会话；一般物体、多朝向学习仍需新增验证数据。

## 真正的TCP平移标定（可选、需要实测）

此工具不发送运动指令。需要固定工装，让**同一个待标定工具点**始终接触
**同一个空间固定点**，通过不同末端姿态采样。不要把不同桌面位置或可移动瓶子
作为固定点，也不要用夹指尖的标定结果冒充夹持中心。

至少8个姿态，绕两个以上轴产生足够旋转，机械臂每次停稳后执行：

```bash
ros2 run piper_moveit_bridge tcp_calibration --capture --fixed-point-contact
```

采样要求实时CAN反馈、无活动轨迹；不会切示教、上使能或移动机械臂。
采集后离线求解：

```bash
ros2 run piper_moveit_bridge tcp_calibration --solve
```

拟合方程为`R_i * tcp + p_i = 固定空间点`。要求至少8组、满秩、条件数不超过100、
残差RMS不超过2毫米、最大残差不超过5毫米；仅拟合TCP平移，不宣称工具坐标朝向已标定。
默认只写`tcp_pivot_result.json`，不改变现有135.8毫米配置。

只有工装和结果均核对后，才写入共享配置：

```bash
ros2 run piper_moveit_bridge tcp_calibration --solve --apply
```

这会写`calibration/tcp_config.json`，仍不会发送运动。需要重启**preview终端**
加载新变换，再重录成功抓取profile。旧profile会因TCP不一致被拒绝。
preview公布`tcp_translation_m`与`tcp_translation_verified`；整体TCP仍标为暂定，
因为固定点拟合不能验证夹爪坐标朝向。请勿仅因一次抓取偏差修改长度。

## 当前提速设置（2026-10-22）

共享设置位于 `piper_moveit_bridge/piper_moveit_bridge/speed_settings.py`。
命令每次启动时加载，已运行的运动不会被中途修改。观察位、悬停靠近、自动抓取
靠近和小步规划统一使用MoveIt速度缩放0.32、加速度缩放0.15；在当前模型
0.2rad/s限速下，请求关节速度上限0.064rad/s，程序验证上限0.08rad/s。
相对旧观察位的0.016rad/s约4倍；相对旧自动靠近0.01rad/s约6.4倍，短距离
仍受加速度约束，不能保证用时恰好按比例下降。

直线下降、平移和试抬按8毫米/秒排时，关节同时限速0.04rad/s，原来分别为
2毫米/秒和0.01rad/s。自动收爪总开口变化速度为8毫米/秒，至少2秒，原来为
4毫米/秒、至少4秒。网关原有0.2rad/s和总开口10毫米/秒上限不变，所有碰撞、
门控、反馈和起点检查继续生效。仅速度变化，不改变TCP、位移或夹爪目标。

旧的手工 `ros2 action send_goal` 命令使用显式 `time_from_start`，不会因为这次
代码更新自动变快。RViz已有窗口的Velocity/Accel Scaling也是独立设置，可分别
设置0.32/0.15；新launch默认值同步更新，既有窗口不要为提速重复启动第二套网关。

## 下降已完成但收爪前因阶段状态交接退出

动作结果可能早于桥接10Hz状态里的`active=false`到达。运行时现已在每段成功
返回后等待三类新反馈和空闲状态，最多3秒，超时停止而不是忽略活动标志。
不要重跑完整`auto_grasp`，也不要从桌边强制回观察位。

新增`resume_grasp --log /绝对路径/原运行日志.json`默认仅验证剩余收爪和试抬。
明确加`--execute`才执行。要求日志只完成hover/descent、profile未变、记录未超过
30分钟、当前位置与下降终点一致、夹爪仍张开。它不重走靠近/下降，且每个日志
只允许一次实际续执行尝试；失败后必须检查现场，不自动重试或松爪。

## 35毫米闭爪目标与接触开口修正（2026-10-23）

当前profile的闭爪**指令**已修正为35毫米，不再使用44.5毫米作为目标。
用户确认夹稳时，独立只读SDK反馈为实际开口39.5毫米、夹爪力矩约-1.044N·m，
驱动已使能且无故障。证据保存在`calibration/grasp_contact_35mm.json`，旧profile已归档。
这是电机力矩反馈，不是已标定的指尖夹持力；不会据此声称抓取/滑移已经完全检测。

只针对这只示教瓶子：闭爪动作显式允许窄范围的位置差，之后要求实际开口接近
39.5毫米（±1.5毫米）、力矩方向一致、幅值至少max(0.25N·m,示教幅值的30%)、
夹爪已使能且无故障、反馈新鲜，并持续稳定至少0.3秒/5次更新，才规划试抬。
抬升用实测夹指开口重新规划，不再用35毫米假设夹指能穿过瓶身。
缺失遥测、状态交接失败或接触证据不足均停止，不抬升，不自动松爪。

网关新增`gripper_effort_nm`、`gripper_enabled`、`gripper_fault`和
`gripper_feedback_age_sec`；桥接新增接触位置容差能力标识。使用这版自动抓取前，
需要停止旧的MoveIt launch并只启动一套新实例，重新添加桌面/相机场景、再开启门控。
若仍持瓶，先保持，不要因更新而复位、下使能或直接松爪；软件更新本身不发送目标。
先确认实物状态后再安排重启和下一次完整抓取。

下一次记录成功抓取必须同时给出实际使用的指令目标，例如：

```bash
ros2 run piper_moveit_bridge record_grasp --confirmed-held --lift-mm 10 \
  --close-command-mm 35 --output ~/GraspGen/calibration/grasp_profile_second.json
```

不能从反馈位置自动推断闭爪指令。当前新抓取只要求空爪开口至少43毫米，
推荐70毫米；前文旧profile的至少53毫米要求不再适用。

## 电脑重启后一键启动（2026-10-23）

```bash
cd ~/GraspGen
./scripts/start_graspgen.sh
```

脚本自动加载Humble/工作区和ROS_DOMAIN_ID=187，使用系统ROS Python；视觉节点
单独使用`.venv-vision`。检测到已运行的相机会复用，检测到已有网关/桥接/MoveIt/
手眼/识别节点则拒绝重复启动。启动相机、识别、手眼TF、预览、MoveIt及真机网关，
随后依次加入桌面和相机碰撞模型。保持此启动终端运行，Ctrl+C停止本次启动的节点，
复用的原相机不会被停止。

启动本身不启用电机、不回零、不打开软件门控、不操作夹爪、不执行运动。
若`can0`未UP，先按脚本提示启用接口；若接口UP但RX不增加，先检查机械臂电源和
物理CAN连接，不能把无反馈时的全零数值当作真实关节状态。CAN恢复后需确认
`reason`为空且状态持续新鲜，再进入原有门控与规划操作。

### 机械臂运动不重写夹爪目标

已修复接触后试抬时的门控撤销：夹持指令35毫米与实际接触开口39.5毫米
不能在动作切换时互相替换。机械臂动作现在只发布六轴关节目标，网关不会
因此发送GripperCtrl；只有夹爪动作才携带显式开口。保留原夹爪速度和步长
保护，不通过放宽网关阈值绕过。更换网关/桥接代码后需要停止旧launch并重新
启动一套流程。若瓶子仍夹持，先确认保持稳定，不松爪、不复位、不下使能；
不要重跑完整auto_grasp，应在新实例反馈正常后仅做1厘米持物试抬。
