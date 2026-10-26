# piper_sim

[EN](README(EN).md)

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)

|ROS |STATE|
|---|---|
|![ros](https://img.shields.io/badge/ROS-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

## 1 gazebo仿真

### 0 环境配置

```bash
sudo apt update
sudo apt install gazebo ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers
```

### 1.1 piper gazebo仿真(有夹爪)

gazebo仿真运行

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 launch piper_gazebo piper_gazebo.launch.py
```

### 1.2 piper gazebo仿真(无夹爪)

```bash
ros2 launch piper_gazebo piper_no_gripper_gazebo.launch.py
```

注：**若通过moveit控制时需要先启动gazebo，再启动moveit，并且使用piper_moveit.launch.py而不是demo.launch.py**

### 1.3 桌面方块场景（有夹爪）

关闭之前的 Gazebo 和 MoveIt，重新编译：

```bash
cd ~/piper_ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select piper_description piper_gazebo piper_with_gripper_moveit
source install/setup.bash
ros2 launch piper_gazebo piper_pick_scene.launch.py
```

新终端启动 MoveIt：

```bash
cd ~/piper_ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch piper_with_gripper_moveit piper_moveit.launch.py base_height:=1.015
```

也可以用一个命令同时启动 Gazebo 和 MoveIt，两者共享底座高度参数：

```bash
ros2 launch piper_gazebo piper_pick_scene.launch.py moveit:=true
```

机械臂底座位于高 1.015 m 的桌面上。桌面尺寸为 1.5 × 0.80 × 0.03 m，
桌面中心世界坐标为 (0.458599, 0.097424, 1.0)，四条桌腿长 1.0 m、半径 0.02 m。
红色方块边长 0.04 m，质量 0.05 kg，落稳后中心世界坐标约为 (0.40, 0, 1.035)，
对应 base_link 坐标约为 (0.40, 0, 0.02)。

`sync_pick_scene.py` 等待 Gazebo 和 MoveIt 服务就绪后，将桌面和方块以
`pick_table`、`pick_cube` 两个碰撞物体加入规划场景，每秒更新两次实际位置。
RViz 的 MotionPlanning 中启用 Scene Geometry 可查看碰撞物体。
桌子和方块的尺寸统一读取 `piper_gazebo/worlds/pick_cube.world`。
同步程序读取各 link 下全部碰撞几何及局部位姿，支持盒子、圆柱和球体；
桌子以包含桌面和四条腿的复合碰撞物体加入 MoveIt。

此入口添加物理场景和规划避障物体，不会自动开始运动。
如果更改桌面高度，需要相应调整 `base_height` 和方块初始高度。
使用一键启动时，`base_height` 会传给 Gazebo 和 MoveIt；分开启动时须保持参数一致。
原有 `piper_gazebo.launch.py` 仍默认从地面启动。
在 Gazebo 保存 world 后，应移除保存进去的 `piper` 模型及 `<state>` 快照，
让启动文件生成机械臂，避免重复模型和旧状态覆盖新的初始位置。

### 1.4 自动抓取方块

更新后先关闭原有 Gazebo / MoveIt，再编译并按 1.3 节启动两者：

```bash
cd ~/piper_ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select piper_description piper_gazebo piper_with_gripper_moveit
source install/setup.bash
```

等待场景同步完成，在第三个终端执行一次抓取：

```bash
cd ~/piper_ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run piper_gazebo pick_cube.py --ros-args -p use_sim_time:=true
```

程序依次执行：张开夹爪 → 移动到方块上方 → 竖直下降 → 闭合夹爪 → 竖直抬起。
为满足 Piper 腕部限位，默认接近距离为 0.04 m，抬起距离为 0.05 m。
末端目标使用 `link6`，抓取中心相对该连杆的轴向距离为 0.125 m。
方块位置从 Gazebo 实时读取，下降和抬起使用 MoveIt 笛卡尔路径；
路径不完整时不执行，任一步失败会退出并报告原因。

双指采用 `joint8 = -joint7` 的 mimic 联动，不再启动单独的镜像控制节点和
`gripper8_controller`。规划和 Gazebo 使用相同的简化夹指碰撞体，夹指接触摩擦系数为 1.5。
抓取阶段仅允许 `pick_cube` 与两侧夹指接触，保留桌子和其他机械臂连杆的碰撞检查。
方块配置了接触传感器；闭合后必须检测到两侧夹指接触。
默认使用 `stabilize_grasp:=true`：Gazebo 插件再次确认连续双指接触后，
在当前位置建立固定约束，稳定位置控制模式下的夹持；张开夹爪时自动解除。
这是用于抓取流程演示的约束辅助夹持，不用于评估真实摩擦夹持能力。
添加 `-p stabilize_grasp:=false` 可测试纯接触摩擦，但当前默认位置控制配置下可能滑脱。
程序在抬起后读取方块实际高度，检测到空抓会报错；成功后保持夹爪闭合，不自动放下。
再次从初始位置测试时，重启仿真以复位机械臂和方块。

可调整参数（例）：

```bash
ros2 run piper_gazebo pick_cube.py --ros-args -p use_sim_time:=true \
  -p velocity_scale:=0.08 -p closed_position:=0.0193
```

`closed_position` 是单侧夹指位移，单位为米；0.035 为张开，4 cm 方块的接触点约为 0.020。
本演示的闭合目标略小于接触点以产生夹持压力。增大抬起高度可能超出腕部限位，
不能以执行部分轨迹作为成功。无界面验证可分别给 Gazebo / MoveIt 启动命令添加
`gui:=false` / `rviz:=false`。

### 1.5 三色方块放入篮筐并堆叠

关闭旧仿真，编译后启动场景和 MoveIt：

```bash
cd ~/piper_ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select piper_description piper_gazebo piper_with_gripper_moveit
source install/setup.bash
ros2 launch piper_gazebo piper_pick_scene.launch.py moveit:=true
```

每次启动都会为蓝色和绿色方块重新随机选择位置。随机区域限制在可达桌面范围内，
方块中心至少相隔 10.5 cm，并与篮筐分开。红色方块保留在 (0.40, 0)；三个方块均为
4 cm、50 g。需要复现布局时添加 `seed:=12`，例如：

```bash
ros2 launch piper_gazebo piper_pick_scene.launch.py moveit:=true seed:=12
```

篮筐固定在桌面 (0.22, 0.16)，外尺寸为 24 × 24 cm，内侧长宽各 22 cm，
底板厚 6 mm，侧壁高 4 cm；底面与桌面齐平。篮筐低侧壁便于观察堆叠。
桌子、篮筐和三个有颜色的方块都同步到 MoveIt。

另开终端执行整个任务：

```bash
source /opt/ros/humble/setup.bash
source ~/piper_ros/install/setup.bash
ros2 run piper_gazebo stack_cubes.py --ros-args -p use_sim_time:=true
```

顺序为：红色放入篮筐底部 → 蓝色放到红色上面 → 绿色放到蓝色上面 →
张开夹爪并回到程序启动时记录的机械臂位置。方块中心的目标世界高度分别约为
1.041、1.081、1.121 m。每放一层都会验证实际位置、层高和倾斜角度，结束时再次检查堆叠和回位。

夹持沿用双指接触确认后的 Gazebo 固定约束；张开夹爪自动解除。
搬运时方块同时附着到 MoveIt 的末端，参与携物避障；场景同步暂停，避免重复加入世界物体。
放下后恢复世界碰撞物体和同步。定位、路径、夹持或堆叠验证失败时停止任务并报告当前阶段。
场景使用更高精度的接触求解和较低的接触纠正速度，减少轻质方块堆叠时的弹跳及横向滑动。
最后保持夹爪朝向，先沿桌面向基座方向退让 8 cm，再规划回初始位置，避免夹指扫到顶层。
发生失败时可先检查日志，若仍夹持物体，可通过 RViz 打开夹爪释放；重新完整测试应重启仿真复位。
原来的 `pick_cube.py` 仍可单独测试红色方块抓起。

## 2 mujoco仿真

### 2.1 mujoco210和mujoco-py的安装

#### 2.1.1 安装mujoco

1、[下载mujoco210](https://github.com/google-deepmind/mujoco/releases/download/2.1.0/mujoco210-linux-x86_64.tar.gz)

2、解压

```bash
mkdir ~/.mujoco
cd (压缩包所在目录)
tar -zxvf mujoco210-linux-x86_64.tar.gz -C ~/.mujoco
```

3、添加环境变量

```bash
echo "export LD_LIBRARY_PATH=~/.mujoco/mujoco210/bin:\$LD_LIBRARY_PATH" >> ~/.bashrc
source ~/.bashrc
```

4、测试

```bash
cd ~/.mujoco/mujoco210/bin
./simulate ../model/humanoid.xml
```

#### 2.1.2 安装mujoco-py

1、下载源码

```bash
git clone https://github.com/openai/mujoco-py.git
```

2、安装(这一部可以在conda环境中进行)

```bash
cd mujoco-py
pip3 install -U 'mujoco-py<2.2,>=2.1'
pip3 install -r requirements.txt
pip3 install -r requirements.dev.txt
python3 setup.py install
sudo apt install libosmesa6-dev
sudo apt install patchelf
```

3、添加环境变量

```bash
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia" >> ~/.bashrc
source ~/.bashrc
```

4、测试

注：**有时会在导入mujoco_py报错，按照报错要求更新numpy、cmake等版本即可**

python运行

```python
import mujoco_py
import os
mj_path = mujoco_py.utils.discover_mujoco()
xml_path = os.path.join(mj_path, 'model', 'humanoid.xml')
model = mujoco_py.load_model_from_path(xml_path)
sim = mujoco_py.MjSim(model)
print(sim.data.qpos)
sim.step()
print(sim.data.qpos)
```

### 2.2 piper mujoco仿真(有夹爪)

mujoco仿真运行

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 run piper_mujoco piper_mujoco_ctrl.py
```

注：**若出现不能控制mujoco机械臂，请重新启动mujoco**

通过rviz_gui控制有夹爪机械臂(新终端运行)

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 launch piper_description display_urdf.launch.py
```

#### 2.3 piper mujoco仿真(无夹爪)

mujoco仿真运行

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 run piper_mujoco piper_no_gripper_mujoco_ctrl.py
```

通过rviz_gui控制无夹爪机械臂(新终端运行)

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 launch piper_description display_no_gripper_urdf.launch.py
```

注：**如果不能控制，请在运行rviz_gui后运行mujoco**

#### 控制参数介绍

[有夹爪控制参数](../piper_description/mujoco_model/piper_description.xml)

[无夹爪控制参数](../piper_description/mujoco_model/piper_no_gripper_description.xml)

- damping="100 更改关节阻尼

- kp="10000" 更改关节控制增益

- forcerange="-100 100" 更改关节控制力矩
