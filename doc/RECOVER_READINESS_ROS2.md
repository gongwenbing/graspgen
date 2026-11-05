# 闭合夹爪反馈为负、瓶子深度不足时的恢复（2026-11-05）

本次只读采样发现：夹爪222次反馈均为-1.1mm，超出正常模型±0.5mm测量容差；视觉中瓶子掩膜约81.8%无深度、12.6%低于0.35m、有效仅5.6%，正深度中位数约0.318m。未自动移动机器人或夹爪，未调大容差、未降低深度门槛。

## 1. 修复软件对待机模式的不一致

网关和执行器原本允许ctrl_mode=0（待机）与1（CAN控制），但共享抓取Runtime只接受1，导致健康待机也不能规划。现已统一允许0/1；教导、错误码、反馈时效、活动轨迹、执行门控和实际执行后的模式检查继续生效。

## 2. 独立慢速张开空夹爪

因为负反馈被正常网关拒绝，MoveIt夹爪动作也可能无法提交。新增`scripts/open_empty_gripper.py`作为受限恢复工具。默认只读，不加载SDK或连接CAN。

```bash
cd ~/GraspGen
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=187
python3 scripts/open_empty_gripper.py
```

只有确认夹爪完全没有夹持物体时，以下命令才可使用：它会实际张开夹爪。请先确保两道软件门控关闭，没有活动动作：

```bash
ros2 service call /piper_can_gateway/enable_outputs std_srvs/srv/SetBool '{data: false}'
ros2 service call /piper_trajectory_bridge/enable_execution std_srvs/srv/SetBool '{data: false}'
python3 scripts/open_empty_gripper.py --execute --confirm-empty
```

脚本仅发送夹爪0x159目标，最大总开口40mm，5mm/s，SDK力矩参数0.5Nm。要求夹爪本来已使能；不会切机械臂控制模式、发送关节目标、使能机械臂、清故障、设置零点、复位或打开门控。SDK连接使用piper_init=False，禁用连接时初始化查询。

只接受-2..45mm的当前原始开口，仅容许已知的夹爪测量越界这一反馈异常；其他反馈故障、示教、未关闭门控、活动动作、反馈超时、机械臂漂移、夹爪不跟随均中止。原有正常测量容差没有改变。

结束要求实际反馈至少39.5mm并稳定。失败时不自动闭合或失能，最后发出的目标可能仍有效，需要检查状态后处理。这个操作让夹爪离开闭合端点，但不是零点标定；若每次闭合都出现-1.1mm，后续应核对夹爪编码器零位。

## 3. 恢复有效观察距离

目前瓶子前表面正深度约0.30..0.32m，且大部分像素无深度。不能靠将min_depth调低或降低60%覆盖门槛解决。应增加相机到瓶子的距离，先约增加8..12cm，观察目标深度是否进入0.4..0.5m且有效深度覆盖恢复。

可在空夹爪恢复后，用既有observation_pose先规划固定观察位；执行需用户显式加--execute和正常门控。调整观察姿态或瓶子位置后，先查看状态：

```bash
ros2 topic echo /piper_hardware/status --field data --once
ros2 topic echo /graspgen/status --field data --once
```

视觉新增depth_diagnostics：missing_fraction、too_near_fraction、too_far_fraction、valid_fraction和positive_depth_median_m。更新后需重启旧视觉launch。它帮助区分无深度、太近和太远，不更改几何过滤规则。

瓶身几何模式使用 `bottle_grasp` 或 `auto_grasp --mode bottle`。原 `auto_grasp` 默认仍是旧经验模式。当前TCP保持用户指定135.8mm。之前观察到的桌面视觉Z约-19.5mm问题没有被抹除；恢复有效深度后仍需复核该坐标偏差，不能直接执行自动抓取。

## 本次后续发现

检查过程中夹爪反馈过期至136秒，网关报告Incomplete or stale CAN feedback；系统查询can0不存在。只读恢复检查按预期拒绝，没有发送夹爪命令。先恢复USB-CAN连接和can0，再重启使用该接口的网关/完整启动终端，取得新鲜反馈后才考虑空夹爪恢复。不要在通信异常时运行--execute。
