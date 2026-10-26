## ROS2 Humble：模型一致性与启动控制

以下说明适用于默认节点 `piper_ik_node`，命令在 GraspGen 根目录执行。

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths piper_kinematics piper_ros/src/piper_msgs piper_ros/src/piper_description piper_ros/src/piper --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch piper_kinematics piper_ik.launch.py
```

另开终端，加载同一工作区环境后显示模型：

```bash
ros2 launch piper_description display_urdf_follow.launch.py
```

RViz2 的 Fixed Frame 为 `base_link`。添加 `InteractiveMarkers`，将
`Interactive Markers Namespace` 设为 `/interactive_marker`，使用 Interact 拖动。
标记首次接收 `/ik/current_pose` 时对齐当前末端，不会把这次对齐作为运动目标。

### 单一模型来源

默认读取 `piper_description/urdf/piper_description.urdf`，直接使用 URDF 的关节
原点、轴、上下限和速度限制计算正解、雅可比与逆解。旧 DH 头文件保留作历史算法资料，
默认节点不再使用其中的 `0.211 m` 末端长度或硬编码限位。

连续拖动优先以上一有效目标解为初值，局部求解失败后尝试确定性的备用初值，
并使用加权误差、回溯步长及自适应阻尼改善限位和奇异位形附近的收敛。
常规优化停滞时再修正位置/姿态容差边界，避免过度优化一项误差而误拒绝另一项可以达标的目标。
所有结果仍必须满足原 URDF 限位和位置/姿态容差。
重复的失败目标会短暂去重，拒绝日志最多每两秒输出一次，并报告最佳位置/姿态残差。
该残差是数值求解结果，不构成目标绝对不可达的证明；固定位置旋转或固定姿态平移
仍可能超出可达范围。

目标 `/target_pose` 默认表示 **`link6` 法兰坐标系**在 `base_link` 中的位姿，
不代表夹爪抓取中心。`tip_link` 可选择同一模型中含六个旋转关节的固定末端链。
自定义 TCP 时，可直接启动节点并设置 `tool_xyz`、`tool_rpy`（均为三个浮点数，
分别为米、弧度；变换相对于 `tip_link`）。例如：

```bash
ros2 run piper_kinematics piper_ik_node --ros-args -p 'tool_xyz:=[0.0, 0.0, 0.12]'
```

`model_path` 可指定 URDF 文件，或通过节点参数 `robot_description` 提供 XML。
更换模型时必须让 robot_state_publisher 使用同一文件；显示启动文件支持 `model:=...`。
可视化和测试一致不等于模型已经通过真实机械臂标定。

### 状态与指令分开

| 接口 | 含义 |
|---|---|
| `/target_pose` | 目标 TCP 位姿，必须明确填写 `base_frame`，默认为 `base_link` |
| `/ik/joint_states` | 只供模型显示；第七项 `joint7` 为单侧夹指行程 |
| `/ik/joint_commands` | 真机专用关节指令；第七项 `gripper` 为夹爪总开口 |
| `/joint_states_single` | 驱动的真实关节反馈，按名称匹配 |
| `/gripper_cmd_topic` | 夹爪总开口，单位米；当前 URDF 允许 0～0.07 |
| `/ik/current_pose` | 当前状态的 TCP 位姿，用于标记初始对齐 |
| `/ik_status` | 最近目标是否求解成功；**不表示机械臂已经到达** |
| `/piper_ik_node/enable_commands` | SetBool 服务，启用/关闭本节点的真机指令输出 |

TF `ik_current` 表示当前状态，`ik_solution` 表示限速后的指令状态，`ik_target`
表示最近求解成功的目标。默认 `hardware_mode:=false` 仅驱动模型显示，
永远不向 `/ik/joint_commands` 或 `/pos_cmd` 发送指令。

夹爪反馈中的 `gripper` 是总开口；`joint7` 是单侧行程，显示时为总开口的一半。
`joint8` 按 URDF mimic 关系反向跟随。数值非有限或超过模型行程的夹爪命令会被拒绝。

### 真机模式的启动顺序

本轮未运行 CAN 驱动，也未验证物理运动。后续只读反馈阶段可以分别启动：

```bash
ros2 launch piper start_single_piper.launch.py auto_enable:=false
ros2 launch piper_kinematics piper_ik.launch.py hardware_mode:=true
```

两个单臂驱动启动文件 `start_single_piper.launch.py` 与
`start_single_piper_rviz.launch.py` 已改为默认不自动使能，控制输入为
`/ik/joint_commands`，夹爪倍率为 1。旧终端中的驱动必须停止并重新启动才能加载改动。
本轮不修改双臂启动方式。

真机模式启动时不发送零位或夹爪归零命令。收到六个具名关节及夹爪的完整反馈后，
先同步当前状态；反馈需有限、在 URDF 限位内，且带有效时间戳。
默认 `feedback_timeout:=0.5` 秒，按消息时间戳和本地单调时钟共同检查新鲜度。
没有有效反馈时，启用输出的请求会失败。

后续进行受控运动测试时，先确认反馈及模型一致，再允许本节点输出：

```bash
ros2 service call /piper_ik_node/enable_commands std_srvs/srv/SetBool '{data: true}'
```

这项服务**不会使能机械臂电机**。成功后仍需收到新的可达目标或有效夹爪命令才开始输出；
启用前的目标不会缓存执行。关节指令默认以不超过 `max_joint_velocity:=0.2 rad/s`
变化，并受 URDF 速度上限约束；夹爪总开口变化不超过 `max_gripper_velocity:=0.01 m/s`。
这些是指令变化率约束，不是经过验证的轨迹加速度控制或碰撞规划。

反馈过期、缺失、无效或越限会撤销输出许可；反馈恢复后不会自动恢复，
必须重新启用并提供新目标。手动关闭输出：

```bash
ros2 service call /piper_ik_node/enable_commands std_srvs/srv/SetBool '{data: false}'
```

停止发布不等同于硬件急停或断使能，不能撤回驱动已经接收的命令。
本轮暂时禁用旧 `use_analytic_ik:=true` 的 `/pos_cmd` 输出路径，
因为固件末端坐标与当前 URDF/TCP 尚未校准，不能混用。

完整测试结果与已修复问题见 [验证记录](../doc/ROS2_IK_VALIDATION.md)。

### 无硬件回归验证

```bash
colcon build --base-paths piper_kinematics piper_ros/src/piper_msgs piper_ros/src/piper_description piper_ros/src/piper --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select piper_kinematics
colcon test-result --test-result-base build/piper_kinematics --verbose
python3 piper_kinematics/test/smoke_ros2.py
python3 piper_kinematics/test/hardware_startup_ros2.py
```

两个 Python 测试分别使用独立 ROS 域 173、174。它们只启动模型、逆解、标记节点，
并模拟硬件反馈；不启动 CAN 驱动。覆盖模型/限位、多姿态求解、工具偏移、
模型 TF、标记链路、夹爪单位、无反馈与未启用时不输出、指令变化率、
过期/无效反馈停发和恢复后不自动运动。

`piper_ik_node_use_yaik` 仍是保留的实验节点，未纳入新的 URDF 与硬件控制入口，
不能用于本次真机验证；`fixed_point_draw_cross_and_circle.py` 仍是未迁移的 ROS1 脚本。
原 `.launch` 文件不再安装。后文为历史算法资料，参数、话题和旧命令以本节说明为准。

---

## **机械臂实现Eigen线代库解算正逆运动学**

## 摘要
本项目实现基于线性代数库Eigen实现松灵PIPER机械臂的正解，逆解的雅各比方法，自定义交互式标记`interactive_marker_utils`的实现。涉及运动学正解、运动学逆解的雅各比方法、RVIZ仿真、机械臂DH、交互式标记、松灵PIPER

## 代码仓库
github链接：[**https://github.com/agilexrobotics/Agilex-College.git**](https://github.com/agilexrobotics/Agilex-College.git)

# 使用前准备
## 硬件准备
+ AgileX robotics Piper机械臂

## 软件环境配置
1. PIPER机械臂驱动部署请参考：[https://github.com/agilexrobotics/piper_sdk/blob/1_0_0_beta/README(ZH).MD](https://github.com/agilexrobotics/piper_sdk/blob/1_0_0_beta/README(ZH).MD)
2. PIPER机械臂ROS控制节点部署参考：[https://github.com/agilexrobotics/piper_ros/blob/noetic/README.MD](https://github.com/agilexrobotics/piper_ros/blob/noetic/README.MD)
3. 安装Eigen线性代数库

```bash
sudo apt install libeigen3-dev
```

## 准备松灵PIPER的DH参数表以及关节限位
查阅松灵PIPER用户手册可以找到PIPER的改进DH参数表与关节限位：

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753784251698-89bd82ef-c2e2-4e98-b1ed-371a697f2e13.png)

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753784668745-70afd1e0-f734-41af-b7bd-1b3ebfa44538.png)

---

# 正向运动学计算FK
正向运动学FK的计算过程实际上是<font style="background-color:#FBDE28;">从每个关节的角度值</font>----计算---->><font style="background-color:#FBDE28;">机械臂某一关节在三维世界的位姿</font>，本文以机械臂最后一个旋转关节joint6为例

## 准备DH参数
1. 根据PIPER的DH参数表构建正向运动学计算程序，由1.3小结的松灵PIPER的改进DH参数表，可以得到

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753784251698-89bd82ef-c2e2-4e98-b1ed-371a697f2e13.png)

```c
// 改进DH参数 [alpha, a, d, theta_offset]
dh_params_ = {
    {0,         0,          0.123,      0},                     // Joint 1
    {-M_PI/2,   0,          0,          -172.22/180*M_PI},      // Joint 2 
    {0,         0.28503,    0,          -102.78/180*M_PI},      // Joint 3
    {M_PI/2,    -0.021984,  0.25075,    0},                     // Joint 4
    {-M_PI/2,   0,          0,          0},                     // Joint 5
    {M_PI/2,    0,          0.091,      0}                      // Joint 6
};
```

转换为标准DH，可以参考以下的转换规则：

        * **<font style="color:rgb(64, 64, 64);">标准DH到改进DH</font>**<font style="color:rgb(64, 64, 64);">：</font>

<font style="color:rgb(64, 64, 64);">αᵢ₋₁(标准) = αᵢ(改进)</font>

<font style="color:rgb(64, 64, 64);">aᵢ₋₁(标准) = aᵢ(改进)</font>

<font style="color:rgb(64, 64, 64);">dᵢ(标准) = dᵢ(改进)</font>

<font style="color:rgb(64, 64, 64);">θᵢ(标准) = θᵢ(改进)</font>

        * **<font style="color:rgb(64, 64, 64);">改进DH到标准DH</font>**<font style="color:rgb(64, 64, 64);">：</font>

<font style="color:rgb(64, 64, 64);">αᵢ(标准) = αᵢ₊₁(改进)</font>

<font style="color:rgb(64, 64, 64);">aᵢ(标准) = aᵢ₊₁(改进)</font>

<font style="color:rgb(64, 64, 64);">dᵢ(标准) = dᵢ(改进)</font>

<font style="color:rgb(64, 64, 64);">θᵢ(标准) = θᵢ(改进)</font>

得到转换后的标准DH：

```c
// 标准DH参数 [alpha, a, d, theta_offset]
dh_params_ = {
    {-M_PI/2,   0,          0.123,      0},                     // Joint 1
    {0,         0.28503,    0,          -172.22/180*M_PI},      // Joint 2 
    {M_PI/2,    -0.021984,  0,          -102.78/180*M_PI},      // Joint 3
    {-M_PI/2,   0,          0.25075,    0},                     // Joint 4
    {M_PI/2,    0,          0,          0},                     // Joint 5
    {0,         0,          0.091,      0}                      // Joint 6
};
```

2. 准备DH变换矩阵
    - 改进DH变换矩阵：

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753866379644-a68c80dc-8d6c-4b0a-8877-287fd877f6e7.png)

    - 用Eigen改写为改进DH的变换矩阵：

```c
T << cos(theta),            -sin(theta),            0,             a,
     sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d,
     sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d,
     0,                     0,                      0,             1;
```

    - 标准DH变换矩阵：

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753866510048-392be307-9e24-4735-85d4-0e61ba2955c4.png)

    - 用Eigen改写为标准DH的变换矩阵：

```c
T << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta),
     sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
     0,           sin(alpha),             cos(alpha),            d,
     0,           0,                      0,                     1;
```

3. 实现正向运动学运算的关键函数`computeFK()`，完整代码见代码仓库[**https://github.com/agilexrobotics/Agilex-College.git**](https://github.com/agilexrobotics/Agilex-College.git)

```cpp
Eigen::Matrix4d computeFK(const std::vector<double>& joint_values) {
    //检查输入关节值数量是否足够（至少6个）
    if (joint_values.size() < 6) {
        throw std::runtime_error("Piper arm requires at least 6 joint values for FK");
    }

    //初始化单位矩阵作为初始变换
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    //对每个关节：
    //    计算实际关节角度 = 输入值 + 偏移量
    //    获取固定参数d值
    //    计算当前关节的变换矩阵并累积到总变换
    for (size_t i = 0; i < 6; ++i) {
        double theta = joint_values[i] + dh_params_[i][3];  // θ = joint_value + θ_offset
        double d = dh_params_[i][2];                       // d = d_fixed (如果是旋转关节)

        T *= computeTransform(
            dh_params_[i][0],  // alpha
            dh_params_[i][1],  // a
            d,                 // d
            theta              // theta
            );
    }

    //返回最终变换矩阵
    return T;
}
```

## 验证FK计算准确性
1. 启动正向运动学验证程序

```bash
ros2 launch piper_kinematics test_fk.launch.py
```

2. 启动RVIZ仿真程序，开启显示TF树，观察计算的<font style="background-color:#FBDE28;">FK所得的机械臂末端</font>`link6_from_fk`的姿态是否和机器人原本的`link6`（由joint_state_publisher计算）是否重合

```bash
ros2 launch  piper_description display_piper_with_joint_state_pub_gui..launch.py 
```

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753868404196-18aa0d44-eeb5-4137-9977-b526d399b806.png)

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753868434120-16763e64-c292-491b-9ad5-813b38665b09.png)

可以看见重合度很高，且从`link6_from_fk`和`link6`的姿态可以看出误差基本在小数点后四位。

# 逆向运动学解算IK
逆向运动学IK的计算过程实际上是<font style="background-color:#FBDE28;">给定一个目标点</font>----计算---->><font style="background-color:#FBDE28;">机械臂每个关节要在什么位置才能使机械臂末端到达目标点</font>

## 确认关节限位
+ 需要确定PIPER机械臂每个关节的限位，确保IK解算出来的路径不会超过机械臂的限位，从而导致机械臂损坏或者其他危险情况
+ 由1.3节可以得知PIPER机械臂每个关节的限位为：

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753784668745-70afd1e0-f734-41af-b7bd-1b3ebfa44538.png)

+ 从而得到机械臂关节限位的矩阵

```cpp
std::vector<std::pair<double, double>> limits = {
    {-154/180*M_PI, 154/180*M_PI},    	// Joint 1
    {0, 			195/180*M_PI},      // Joint 2
    {-175/180*M_PI, 0},         		// Joint 3
    {-102/180*M_PI, 102/180*M_PI},      // Joint 4
    {-75/180*M_PI,  75/180*M_PI},       // Joint 5
    {-120/180*M_PI, 120/180*M_PI}       // Joint 6
};
```

## IK的雅各比矩阵方法实现的简要步骤
求解过程

1. **<font style="color:rgb(64, 64, 64);">计算误差e</font>**<font style="color:rgb(64, 64, 64);">：当前位姿与目标位姿的差异(6维向量：3位置+3姿态)</font>
2. **<font style="color:rgb(64, 64, 64);">误差e是否小于阈值？</font>**
    - **<font style="color:rgb(64, 64, 64);">是</font>**<font style="color:rgb(64, 64, 64);"> </font><font style="color:rgb(64, 64, 64);">→ 返回当前θ作为解</font>
    - **<font style="color:rgb(64, 64, 64);">否</font>**<font style="color:rgb(64, 64, 64);"> → 进入迭代优化步骤</font>
3. **<font style="color:rgb(64, 64, 64);">计算雅可比矩阵J</font>**<font style="color:rgb(64, 64, 64);">：6×6矩阵</font>
4. **<font style="color:rgb(64, 64, 64);">计算阻尼伪逆</font>**<font style="color:rgb(64, 64, 64);">：</font>

$ J⁺ = Jᵀ(JJᵀ + λ²I)⁻¹ $

<font style="color:rgb(64, 64, 64);">λ是阻尼系数，避免奇异位形时数值不稳定</font>

5. **<font style="color:rgb(64, 64, 64);">计算关节角度增量</font>**<font style="color:rgb(64, 64, 64);">：</font>

$ Δθ = J⁺e $

<font style="color:rgb(64, 64, 64);">通过误差e和伪逆计算关节角度的调整量</font>

6. **<font style="color:rgb(64, 64, 64);">更新关节角度</font>**<font style="color:rgb(64, 64, 64);">：</font>

$ θ = θ + Δθ $

<font style="color:rgb(64, 64, 64);">应用调整量到当前关节角度</font>

7. **<font style="color:rgb(64, 64, 64);">应用关节限制：</font>**
8. **<font style="color:rgb(64, 64, 64);">角度归一化</font>**
9. **<font style="color:rgb(64, 64, 64);">达到最大迭代?</font>**
    - **<font style="color:rgb(64, 64, 64);">否</font>**<font style="color:rgb(64, 64, 64);"> </font><font style="color:rgb(64, 64, 64);">→ 回到步骤2继续迭代</font>
    - **<font style="color:rgb(64, 64, 64);">是</font>**<font style="color:rgb(64, 64, 64);"> → 抛出未收敛错误</font>

关键函数`computeIK()`

```cpp
std::vector<double> computeIK(const std::vector<double>& initial_guess, 
                                 const Eigen::Matrix4d& target_pose,
                                 bool verbose = false,
                                 Eigen::VectorXd* final_error = nullptr) {
    //初始化一个猜测姿态（初始姿态）
    if (initial_guess.size() < 6) {
        throw std::runtime_error("Initial guess must have at least 6 joint values");
    }

    std::vector<double> joint_values = initial_guess;
    Eigen::Matrix4d current_pose;
    Eigen::VectorXd error(6);
    bool success = false;

    //开始迭代计算
    for (int iter = 0; iter < max_iterations_; ++iter) {
        //先计算初始状态的FK获取初始状态的位置与姿态
        current_pose = fk_.computeFK(joint_values);
        //计算初始状态的位置与姿态相较于目标点的误差
        error = computePoseError(current_pose, target_pose);

        if (verbose) {
            std::cout << "Iteration " << iter << ": error norm = " << error.norm() 
                      << " (pos: " << error.head<3>().norm() 
                      << ", orient: " << error.tail<3>().norm() << ")\n";
        }

        //检查误差是否小于阈值，分为位置误差阈值与姿态误差阈值
        if (error.head<3>().norm() < position_tolerance_ && 
            error.tail<3>().norm() < orientation_tolerance_) {
            success = true;
            break;
        }

        //计算雅各比矩阵（默认使用解析雅各比）
        Eigen::MatrixXd J = use_analytical_jacobian_ ? 
            computeAnalyticalJacobian(joint_values, current_pose) :
            computeNumericalJacobian(joint_values);

        //采用阻尼最小二乘法(Levenberg-Marquardt)
        //Δθ = Jᵀ(JJᵀ + λ²I)⁻¹e
        //θ_new = θ + Δθ
        Eigen::MatrixXd Jt = J.transpose();
        Eigen::MatrixXd JJt = J * Jt;
        //lambda_: 阻尼系数(默认0.1)，避免奇异位形时数值不稳定
        JJt.diagonal().array() += lambda_ * lambda_;
        Eigen::VectorXd delta_theta = Jt * JJt.ldlt().solve(error);

        //更新
        for (int i = 0; i < 6; ++i) {
            //应用调整量到当前关节角度
            double new_value = joint_values[i] + delta_theta(i);
            //确保更新后的θ在机械臂的物理限制范围内（关节角度限位）
            joint_values[i] = std::clamp(new_value, joint_limits_[i].first, joint_limits_[i].second);
        }

        //将关节角度规范到[-π,π]等标准范围内（避免不必要的多圈旋转）
        normalizeJointAngles(joint_values);
    }

    //如果超过最大迭代次数(100)还未求解出结果，抛出异常
    if (!success) {
        throw std::runtime_error("IK did not converge within maximum iterations");
    }

    //计算误差
    if (final_error != nullptr) {
        current_pose = fk_.computeFK(joint_values);
        *final_error = computePoseError(current_pose, target_pose);
    }

    return joint_values;
    }
```

## 使用interactive_marker以实现发布机械臂三维空间目标点
1. 安装ROS2依赖包

```cpp
sudo apt install ros-${ROS_DISTRO}-interactive-markers ros-${ROS_DISTRO}-tf2-ros
```

2. 启动interactive_marker_utils实现三维空间目标点发布

```cpp
ros2 launch interactive_marker_utils marker.launch.py 
```

3. 启动RVIZ2观察Marker

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753872321648-69a6f2ba-681b-4a26-81e2-9ac9a753f463.png)

4. 拖动Marker，并用ros2 topic echo 观察Marker发布的目标点是否有变化

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753872475114-5c107f79-12bd-4eed-a82d-4b3329bcc190.png)

## 在RVIZ中通过interactive_marker验证IK是否正确
1. 启动松灵PIPER的RVIZ仿真demo，因为此时没有`joint_state_publisher`，所以模型没有正确显示

```cpp
ros2 launch piper_description display_piper.launch.py 
```

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753872773158-ea4982f3-9280-4978-86ff-0452e233d76b.png)

2. 接下来启动IK节点和`interactive_marker`节点（在同一个launch文件里）,启动成功后可以看到机械臂正常显示

```cpp
ros2 launch piper_kinematics piper_ik.launch.py
```

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753873508356-b36b3048-463d-428d-909b-3b55f47c3cc3.png)

3. 使用`interactive_marker`控制机械臂进行IK解算  
![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753873573382-f4d0b195-f9f0-46f5-befb-ff6aca74e1c7.png)

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753873620366-fbd691ad-50ea-4585-9730-98193a5565df.png)

4. 拖动`interactive_marker`可以看见IK成功解算出每个关节的角度

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753873809425-e7d7617d-1284-4b2c-b66a-9a4bab7459de.png)

5. 如果拖动`interactive_marker`到达无法解算的地方则会抛出异常

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753873898378-0ad9897d-b22d-4093-845e-a91c0d730e92.png)

# 在真实PIPER机械臂上验证IK
1. 首先启动连接PIPER的CAN通信的脚本

```cpp
cd piper_ros
./find_all_can_port.sh 
./can_activate.sh 
```

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753874234843-d776e6b7-0a07-4e3d-a93a-939388f15544.png)

2. 启动PIPER真机控制节点

```cpp
ros2 launch piper my_start_single_piper_rviz.launch.py 
```

3. 接下来启动IK节点和`interactive_marker`节点（在同一个launch文件里），可以看到机械臂运动到HOME点

```cpp
ros2 launch piper_kinematics piper_ik.launch.py
```

4. 拖动`interactive_marker`并观察PIPER机械臂运动情况

[此处为语雀卡片，点击链接查看](https://www.yuque.com/docs/230096875#FvOGu)





