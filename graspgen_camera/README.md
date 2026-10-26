# ROS 2 Humble RealSense 相机使用说明

本包使用已安装的官方 ROS 2 `realsense2_camera` 驱动，旧 ROS 1 相机源码已删除。
已在 RealSense D455 上验证，默认彩色和深度配置均为 **640×480、30 FPS**。
相机启动不会驱动机械臂，也不会自动建立相机到机械臂的标定变换。

## 启动相机

```bash
cd ~/GraspGen
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=187

ros2 launch graspgen_camera rgbd.launch.py
```

请使用USB 3数据线，并确认 `lsusb` 能识别相机；同一设备不要重复启动驱动。

## 检查彩色图、深度与内参

另开终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/GraspGen/install/setup.bash
export ROS_DOMAIN_ID=187

ros2 run graspgen_camera check_rgbd
```

输出 `PASS` 表示彩色图、对齐深度、内参、尺寸和时间戳等检查通过。
显示的深度中位数是整幅图有效深度的统计值，不是待抓物体距离。
此检查不代表物体识别、物理标定精度或手眼标定已经完成。

## 相机话题

| 话题 | 内容 |
|---|---|
| `/camera/color/image_raw` | 彩色图像 |
| `/camera/color/camera_info` | 彩色相机内参 |
| `/camera/aligned_depth_to_color/image_raw` | 对齐到彩色图的深度图 |
| `/camera/aligned_depth_to_color/camera_info` | 对齐深度内参 |

默认关闭驱动的整幅点云，降低USB和CPU负载；视觉节点会单独生成物体点云。

## 图像可视化

在加载相同环境的终端运行 `rviz2`，固定坐标系设为 `camera_link`。
添加 `Image`，Topic选择上述彩色或深度图，Reliability Policy设为 `Best Effort`。
如果提示 `Empty topic name`，需要在Image的Topic属性中填写话题名称。

带物体分割、点云和候选姿态的完整界面见 [视觉使用说明](../graspgen_vision/README.md)。
