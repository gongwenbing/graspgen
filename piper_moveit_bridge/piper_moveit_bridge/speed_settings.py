"""Shared commissioning speeds; all remain below the existing gateway caps."""

# MoveIt model arm limit is 0.2rad/s; 32% requests up to 0.064rad/s.
APPROACH_VELOCITY_SCALE = 0.32
APPROACH_ACCELERATION_SCALE = 0.15
ARM_EXECUTION_CAP = 0.08

CARTESIAN_JOINT_SPEED = 0.04
CARTESIAN_LINEAR_SPEED = 0.01  # 10 mm/s

GRIPPER_TOTAL_SPEED = 0.008  # 总开口 8 mm/s
GRIPPER_MIN_DURATION = 2.0
