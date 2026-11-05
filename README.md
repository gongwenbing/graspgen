# GraspGen：ROS 2 Humble 物体分割与抓取候选生成

> 2026-11-05：启用32样本新手眼标定；实测桌面Z=-11.8mm、瓶子夹持中心Z=48.2mm，TCP仍为135.8mm。需重启完整系统以同步TF和碰撞场景，详见瓶身几何抓取说明。

> 2026-10-28 更新：已拆分独立抓取生成节点，新增32mm直径/120mm高度瓶子的圆柱拟合与MoveIt抓取。2026-10-29按用户要求恢复135.8mm指尖高度TCP；旧经验模式与新瓶身拟合模式仍分别使用。完整启动、参数、规划/执行和限制见[瓶身几何抓取说明](doc/BOTTLE_GEOMETRY_ROS2.md)。

当前运行环境：Ubuntu 22.04、ROS 2 Humble、PiPER、RealSense D455。
ROS 1 相机和视觉包已被替换，旧 catkin / roslaunch / rosrun 启动流程不再适用。

当前链路：

```text
D455 彩色图 + 对齐深度 + 内参
  → YOLOv8 实例分割
  → 掩码内有效深度 / 物体点云 / 离群点过滤
  → PCA 表面方向 / 包围盒 / 抓取候选（相机坐标系）
```

候选姿态不等于已验证的抓取动作。手眼标定、TCP 方向、环境碰撞模型以及
预抓取/接近/闭爪/抬起动作仍需完成；视觉节点不会发送机械臂运动指令。

## 1. 环境和安装

已安装 ROS 2 Humble 后：

```bash
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description \
  ros-humble-cv-bridge ros-humble-image-transport ros-humble-message-filters \
  ros-humble-sensor-msgs-py ros-humble-visualization-msgs
cd ~/GraspGen
bash scripts/setup_vision.sh
source /opt/ros/humble/setup.bash
colcon build --base-paths graspgen_camera graspgen_vision --executor sequential
```

推理依赖在项目 `.venv-vision`，不会替换系统 Python 或显卡驱动。
当前使用 YOLOv8n-seg / PyTorch CPU 后端，尚未提供 TensorRT/GPU 加速。
无需照旧文档安装驱动470、CUDA11.4、TensorRT8.6或重新编译 OpenCV。
官方预训练模型为 COCO 类别；未训练的自定义物体类别不能保证识别。
模型下载到 `models/yolov8n-seg.pt`。Ultralytics 的使用遵循其上游许可。

## 2. 每个新终端加载环境

```bash
cd ~/GraspGen
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=187
```

## 3. 相机（终端 A）

```bash
ros2 launch graspgen_camera rgbd.launch.py
```

独立检查：`ros2 run graspgen_camera check_rgbd`。
[相机说明](graspgen_camera/README.md)。

## 4. 视觉（终端 B）

```bash
ros2 launch graspgen_vision vision.launch.py
```

默认处理 bottle、cup、mouse、cell phone、orange；更改示例：

```bash
ros2 launch graspgen_vision vision.launch.py target_classes:='bottle,cup'
```

默认有效深度 0.15..1.5 m。请放置支持类别的物体，检查其深度是否有效。
无目标、深度无效、宽度超过70 mm或PCA方向不明确时不生成候选。

## 5. 可视化（终端 C）

```bash
rviz2 -d "$(ros2 pkg prefix graspgen_vision)/share/graspgen_vision/config/vision.rviz"
```

已配置分割画面、彩色物体点云、包围盒及姿态坐标轴。
固定坐标系为 `camera_color_optical_frame`，未伪造相机到 `base_link` 的变换。

```bash
ros2 topic echo /graspgen/status --field data --once
```

状态包括类别、置信度、点数、候选有效性及原因、开口估计等。
[视觉算法及完整话题说明](graspgen_vision/README.md)。

## 6. 机械臂控制

[MoveIt 真机桥接与门控](piper_moveit_bridge/README.md)。
[URDF 逆解与交互标记](piper_kinematics/README.md)。
视觉调试阶段请关闭机械臂运动门控；当前视觉输出不会自动触发抓取。
旧 `arm_action.py` / `item_confer.py` 按 s 自动运动流程已删除。

## 7. 眼在手上标定

已配置 ID0、黑色外框边长95毫米的ArUco标记。相机固定在末端，标记固定在桌面。

```bash
ros2 launch graspgen_handeye collect.launch.py
```

RViz Image话题为 `/handeye/image`，状态话题为 `/handeye/status`。
工具只观测并在手动调用服务时保存样本，不自动移动机械臂。
采集、求解及相机安装TF发布步骤见 [中文标定说明](graspgen_handeye/README.md)。
**目前配置的是标定工具，尚未采集实物样本并得到手眼标定结果。**

## 8. 验证

```bash
PYTHONPATH="$PWD/graspgen_vision:$PYTHONPATH" python3 -m pytest -q graspgen_vision/test/test_geometry.py
python3 graspgen_vision/test/check_ros2_pipeline.py
```

第二项使用隔离 ROS 域及合成输入，不访问相机或机械臂。
真实模型的离线验证见 `graspgen_vision/test/check_real_inference.py`，其深度为合成数据。

## 上游

- [原 GraspGen 仓库](https://github.com/vanstrong12138/GraspGen)
- [Ultralytics YOLO](https://github.com/ultralytics/ultralytics)
- [RealSense ROS 2](https://github.com/realsenseai/realsense-ros)
