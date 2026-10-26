# 眼在手上标定：ID 0，95毫米

相机固定在末端，标记固定在桌面。标记边长为**黑色图案外边缘95毫米，不含白边**。
默认 `DICT_4X4_50`、ID0；OpenCV的4×4容量50/100/250/1000字典共享ID0图案，已通过测试核对。
本工具不移动机械臂、不使能、不重置、不写零点、不触发夹爪。

## 1. 准备

- 标记贴在平整硬板上并固定；整个采集期间不能移动。
- 相机支架固定牢靠，整个采集和使用期间不能调整。
- 保持 D455 驱动运行：`ros2 launch graspgen_camera rgbd.launch.py`。
- 保持真实机械臂反馈及 robot_state_publisher 运行（现有真机桥接启动已包含）。
- 所有终端统一加载下面环境。无需激活视觉虚拟环境，使用系统 OpenCV。

```bash
cd ~/GraspGen
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=187
```

## 2. 启动观测和采集

```bash
ros2 launch graspgen_handeye collect.launch.py
```

RViz 添加Image，Topic设为 `/handeye/image`，Reliability Policy设为Best Effort。
绿色框及坐标轴表示检测到标记；橙色框/AMBIGUOUS表示单平面位姿不明确，换一个斜视角。
查看状态：

```bash
ros2 topic echo /handeye/status --field data --once
```

`capture_ready: true` 才能采集。需要标记清晰、有效的同步末端TF、机械臂停稳。
`marker_pose` 是标记到相机的观测，不是机器人运动目标。

## 3. 手动采集20～25个不同姿态

用已验证的MoveIt小范围规划改变相机的位置与朝向，不使用自动采集运动。
优先小幅改变观察角度，始终看得到整张标记，避开桌面、标记和线缆。
每次停稳约1～2秒，确认 `capture_ready: true` 后执行一次：

```bash
ros2 service call /graspgen_handeye/capture std_srvs/srv/Trigger '{}'
```

每次应返回 `success: true` 和样本数量。多个姿态需覆盖至少两个不同旋转轴，
并有距离/平移变化；不要只绕一个轴转或只平移。正对标记容易产生平面歧义，
可尝试15～40度斜视。重复姿态（相差不足10毫米且不足5度）会被拒绝。

撤销最后一个样本：

```bash
ros2 service call /graspgen_handeye/undo std_srvs/srv/Trigger '{}'
```

文件自动保存到 `~/GraspGen/calibration/handeye_samples.json`，重启会继续加载。
若板或支架移动，不能混用旧样本，应换新文件：

```bash
ros2 launch graspgen_handeye collect.launch.py samples_file:=$HOME/GraspGen/calibration/session2.json
```

## 4. 离线求解

至少16个有效样本，推荐20～25个。采集完成并停止移动后执行：

```bash
ros2 run graspgen_handeye solve \
  ~/GraspGen/calibration/handeye_samples.json \
  --output ~/GraspGen/calibration/handeye_result.yaml
```

已有输出文件不会覆盖，请另选名称。求解输入为 `base_link←link6` 和 `相机←标记`，
结果为 `link6←camera_color_optical_frame`。每4个样本留1个不用来拟合。
通过条件：留出样本平移RMS≤5毫米、旋转RMS≤2度，全部样本最大误差≤15毫米/5度；
相机偏移≤0.5米，旋转激励充分。这些是项目工程门槛，不是厂家精度规格。
`accepted: false` 只保存诊断报告，不得发布或用于抓取。

## 5. 发布相机安装变换：仅求解通过后

```bash
ros2 run graspgen_handeye publish_mount --ros-args \
  -p calibration_file:=$HOME/GraspGen/calibration/handeye_result.yaml
```

实际发布 `link6 → camera_link`，保留相机驱动已有的 `camera_link → 光学坐标系` 子树，
避免给光学帧设置第二个父节点。保持此节点运行以供后启动的订阅者接收静态TF。
启动前需移除其他人为设置的相机安装TF，不能同时存在两个安装变换发布者。

求解后还需在**新的、未参加求解的姿态**检查标记在base_link中的位置是否稳定，
并对照实测位置验证尺度。单标记平面估计、机器人模型误差和打印尺寸都会影响结果。
标定不是机器人TCP标定，不直接授权自动抓取。

## 测试和接口依据

```bash
PYTHONPATH="$PWD/graspgen_handeye:$PYTHONPATH" python3 -m pytest -q graspgen_handeye/test/test_core.py
```

[OpenCV手眼标定方向定义](https://docs.opencv.org/4.7.0/d9/d0c/group__calib3d.html)。

## 6. 独立验证：记录3～5个新姿态的漂移

保持相机、机械臂真实反馈和 `publish_mount` 运行，标记板保持原位。
原 `collect` 节点可以停止；下面工具自带标记观测，不会修改原25组样本或标定结果。

另开已加载项目环境、`ROS_DOMAIN_ID=187` 的终端：

```bash
ros2 run graspgen_handeye validate \
  --calibration ~/GraspGen/calibration/handeye_result.yaml \
  --training-samples ~/GraspGen/calibration/handeye_samples.json
```

在示教模式下，手动选择未参加标定的新观察姿态，完整看到标记并停稳1～2秒。
保持板不动。每个新姿态在另一个已加载相同环境的终端调用一次：

```bash
ros2 service call /handeye_validation/capture std_srvs/srv/Trigger '{}'
```

成功返回验证样本数量、标记在base_link下的XYZ（米）和目前最大位置漂移（毫米）。
姿态太接近原标定姿态、重复验证姿态、未停稳、PnP歧义、TF失效都会拒绝保存。
状态可查看 `/handeye_validation/status`，预览图为 `/handeye_validation/image`。
录错最后一组可调用 `/handeye_validation/undo`（Trigger）。

采集至少3组、建议5组后生成报告：

```bash
ros2 service call /handeye_validation/report std_srvs/srv/Trigger '{}'
```

记录为 `calibration/handeye_validation.json`，报告为 `calibration/handeye_validation_report.json`。
报告包含任意两组之间最大位置/角度差、相对均值RMS，以及相对原标定固定标记位置的误差。
本项目通过门槛为至少3组、最大位置漂移≤10毫米、角度漂移≤3度、
相对原标记参考的最大误差≤15毫米/5度；这不是厂家规格，也不代表抓取精度。
`success: false` 时查看报告，不能仅凭箭头目视稳定认定通过。
发布的相机TF也会与标定文件核对，防止验证时使用另一份安装变换。

不得把这些验证记录作为重新拟合的样本，否则失去独立验证意义。

## 8. TCP与物体上方预抓取：先只显示

保持相机、机械臂真实反馈/robot_state_publisher和已验证的 `publish_mount` 运行。
该阶段不需要打开机械臂运动门控；机械臂桥接可以保持只读。
先启动识别（不要重复启动已有实例）：

```bash
ros2 launch graspgen_vision vision.launch.py
```

另一个已加载相同环境的终端启动预览：

```bash
ros2 launch graspgen_handeye preview.launch.py
```

将打开专用RViz，固定坐标系为base_link，包含机器人、分割图与预览标记。
- 青色球：当前模型TCP预览，位置为link6沿+Z偏移135.8毫米。
- 黄色球：相机抓取候选可见表面中心，变换到base_link后的位置。
- 绿色球及坐标轴：该中心沿base_link的+Z方向上方100毫米处的预抓取TCP。
- 黄色连线：从表面中心到预抓取TCP的垂直抬升距离。
- 预抓取TCP蓝色+Z轴朝基座下方，绿色+Y轴为水平闭合方向。

```bash
ros2 topic echo /grasp_preview/status --field data --once
```

`current_tcp_valid` 表示真实模型TF新鲜；`target_valid` 表示视觉/TF通过预览检查。
它们都不代表IK可达、碰撞检查通过或可以执行。

135.8毫米来自当前URDF的joint7/8原点和指端网格边界，**不是实物TCP标定结果**。
网格在指节局部Y方向延伸到-76.5毫米，经joint7的Rx(π/2)转回安装坐标后，
指端最前方约为link6 +Z 135.8毫米；开合轴沿link6 ±Y。
这个模型中心会显示在两指前端中间，不能只凭显示正常就认定实物偏移正确。
需通过实测或TCP标定核实真正夹持中心后再配置执行用TCP。
可显式覆盖预览值：

```bash
ros2 launch graspgen_handeye preview.launch.py tcp_z:=0.1358 hover_height:=0.10
```

这只是模型核对参数，不要为让球落到物体上随意调整偏移。
新TF名为 `piper_tcp_preview`，不会改URDF、MoveIt执行末端或相机标定。
物体上方的TCP位置会换算成相应的link6位置：
`T_base_link6 = T_base_TCP × inverse(T_link6_TCP)`，不能把TCP目标直接当作link6目标。

位置话题：`/grasp_preview/current_tcp`、`/grasp_preview/pregrasp_tcp`、
`/grasp_preview/pregrasp_link6`；显示MarkerArray为 `/grasp_preview/markers`。
这些话题只供预览，后续消费者必须验证时间戳及状态，不应直接连接旧控制脚本。

**当前节点没有运动指令发布器、MoveIt执行客户端或夹爪控制调用。**
未检查环境碰撞、相机支架间隙、真实TCP精度或IK可达性，禁止据此直接执行下降/闭爪。
下一阶段才是在TCP实测确认和碰撞环境配置后做只规划、再由操作者确认小步运动。
