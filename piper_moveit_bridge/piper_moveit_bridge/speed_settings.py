"""Shared commissioning speeds; all remain below the existing gateway caps."""

# MoveIt model arm limit is 0.2rad/s; 32% requests up to 0.064rad/s.
APPROACH_VELOCITY_SCALE = 0.32
APPROACH_ACCELERATION_SCALE = 0.15
ARM_EXECUTION_CAP = 0.08

CARTESIAN_JOINT_SPEED = 0.04
CARTESIAN_LINEAR_SPEED = 0.01  # 10 mm/s

# 总开口限速统一供计时、轨迹桥接器和CAN网关使用。
GRIPPER_OPENING_CAP = 0.02 # 总开口20 mm/s；单指为其一半
GRIPPER_OPEN_SPEED = 0.02
GRIPPER_CLOSE_SPEED = 0.02
GRIPPER_MIN_DURATION = 0.2
GRIPPER_TOTAL_SPEED = GRIPPER_CLOSE_SPEED  # 兼容旧引用
