# ROS 2 物体分割、物体点云与抓取候选姿态使用说明

> 2026-11-05：启用32样本新手眼标定；实测桌面Z=-11.8mm、瓶子夹持中心Z=48.2mm，TCP仍为135.8mm。需重启完整系统以同步TF和碰撞场景，详见瓶身几何抓取说明。

> 2026-10-28 更新：已拆分独立抓取生成节点，新增32mm直径/120mm高度瓶子的圆柱拟合与MoveIt抓取。2026-10-29按用户要求恢复135.8mm指尖高度TCP；旧经验模式与新瓶身拟合模式仍分别使用。完整启动、参数、规划/执行和限制见[瓶身几何抓取说明](../doc/BOTTLE_GEOMETRY_ROS2.md)。

本包已替换旧 ROS 1 的 `tensorrtx_yolov8_ros`，完成以下链路：

**D455 彩色图和对齐深度 → YOLOv8 实例分割 → 物体点云 → PCA 抓取候选姿态。**

当前使用官方 YOLOv8n-seg 权重，推理环境位于项目 `.venv-vision`，版本为
Ultralytics 8.3.221、PyTorch 2.7.1 CPU。系统 ROS Python 环境不受影响。
旧 TensorRT 自定义插件和引擎已删除，当前尚未实现 TensorRT/GPU 加速。
CPU 推理不需要安装 CUDA Toolkit、cuDNN 或 TensorRT。

## 1. 安装与编译

本机已经配置完成，正常使用无需重复安装。重新部署时执行：

```bash
cd ~/GraspGen
bash scripts/setup_vision.sh
source /opt/ros/humble/setup.bash
colcon build --base-paths graspgen_camera graspgen_vision --executor sequential
```

安装脚本需要网络，推理依赖安装到项目虚拟环境。

## 2. 启动相机：终端 A

```bash
cd ~/GraspGen
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=187

ros2 launch graspgen_camera rgbd.launch.py
```

如果相机已经启动，不要重复启动。

## 3. 启动物体识别：终端 B

```bash
cd ~/GraspGen
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=187

ros2 launch graspgen_vision vision.launch.py
```

默认处理以下 COCO 类别，参数需要使用对应英文名称：

| 中文 | 参数名称 |
|---|---|
| 瓶子 | `bottle` |
| 杯子 | `cup` |
| 鼠标 | `mouse` |
| 手机 | `cell phone` |
| 橙子 | `orange` |

只识别指定类别：

```bash
ros2 launch graspgen_vision vision.launch.py target_classes:='bottle,cup'
```

处理模型支持的所有类别：

```bash
ros2 launch graspgen_vision vision.launch.py target_classes:=''
```

当前为通用预训练模型，未训练的自定义类别不能保证识别。
默认有效深度范围为 **0.35～1.5 米**。D455在默认640×480模式的名义最小深度约0.32米，
因此留出3厘米余量；不同分辨率和相机不能直接套用此下限。
建议先把物体放在相机前约0.4～0.8米处，
确认深度有效；透明、反光材质可能有彩色检测结果却没有可靠点云。
掩膜腐蚀后至少60%的像素需要有范围内有效深度，主连通深度区域需覆盖至少30%的掩膜。
少量有效点或背景漏点不再足以生成抓取候选。该覆盖检查仍不能证明所有深度都属于物体。
参考：[RealSense D400数据表，表4-11](https://www.realsenseai.com/download/21345/)。

## 4. 可视化：终端 C

```bash
source /opt/ros/humble/setup.bash
source ~/GraspGen/install/setup.bash
export ROS_DOMAIN_ID=187

rviz2 -d "$(ros2 pkg prefix graspgen_vision)/share/graspgen_vision/config/vision.rviz"
```

配置已包含分割画面、彩色物体点云、包围盒和姿态坐标轴。
固定坐标系为 `camera_color_optical_frame`，当前没有相机到 `base_link` 的手眼标定变换。

## 5. 查看结果与话题

```bash
ros2 topic echo /graspgen/status --field data --once
```

| 话题 | 内容 |
|---|---|
| `/graspgen/segmentation/image` | 带实例分割和类别标注的图像 |
| `/graspgen/object_cloud` | 物体点云，包含XYZ、RGB及物体编号 |
| `/graspgen/markers` | 包围盒、文字和姿态坐标轴 |
| `/graspgen/grasp_poses` | 当前有效抓取候选姿态数组 |
| `/graspgen/selected_grasp` | 有效候选中置信度最高者的姿态 |
| `/graspgen/status` | JSON状态：类别、置信度、点数、开口估计及无效原因 |

`valid: true` 表示有有效的几何候选，不表示真机已经可以安全执行抓取。
物体编号只在当前帧内有效，不是跨帧跟踪编号。

没有候选时发布空姿态数组、清除可视化，并更新 `valid: false`。
存在物体点云但几何检查失败时，仍可显示点云以便排查。
`selected_grasp` 不锁存，也不会在识别丢失时重复发布旧目标；后续使用者必须检查状态和时间戳。

## 6. 算法与限制

处理过程包括掩码腐蚀、有效深度过滤、中位深度范围过滤、最大连通区域选择、
限量采样、邻域离群点过滤，以及可见表面的PCA方向和包围盒估计。

候选姿态约定：局部 **+X为表面主方向，+Y为夹爪闭合方向，+Z为从相机朝向表面的接近方向**。
此约定不等同于机械臂TCP坐标轴，后续必须显式转换。

当PCA退化、方向不明确，或可见宽度加5毫米余量超过70毫米时，不生成抓取候选。
单视角深度无法看到物体背面，因此不能证明实际厚度、夹持稳定性或碰撞间隙。

**当前仅完成视觉候选生成，不会驱动机械臂。**
旧 `arm_action.py` 和 `item_confer.py` 自动运动脚本已删除。
真正抓取前仍需手眼标定、TCP对齐、接近间隙、环境碰撞模型及抓取动作流程。

## 7. 测试

加载ROS环境后，在项目根目录执行：

```bash
PYTHONPATH="$PWD/graspgen_vision:$PYTHONPATH" python3 -m pytest -q graspgen_vision/test/test_geometry.py
python3 graspgen_vision/test/check_ros2_pipeline.py
```

第二项使用隔离ROS域、合成RGB-D及模拟推理，不访问相机或机械臂。
实机验证记录见 [视觉验证记录](../doc/ROS2_VISION_VALIDATION_2026-10-21.md)。


Gazebo 彩色方块识别：`ros2 launch graspgen_vision cube_vision.launch.py`。使用颜色分割、对齐深度和图像时刻 TF，按已知 4cm 边长估计中心；不依赖 YOLO 方块类别。完整抓放步骤见 `piper_ros/src/piper_sim/piper_gazebo/VISION_CUBE_TO_BASKET.md`。不要与瓶子识别节点同时发布 `/graspgen/status`。
