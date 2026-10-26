#!/usr/bin/env python3
"""One-shot MoveIt/Gazebo grasp with optional contact-triggered stabilization."""
import copy
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningSceneComponents,
    RobotState,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetPlanningScene,
    GetPositionIK,
    GetStateValidity,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


def moveit_state(message: JointState) -> RobotState:
    """Map hardware joint names to the planning model, excluding non-finite effort."""
    names = [f'joint{i}' for i in range(1, 8)]
    values = dict(zip(message.name, message.position))
    if not all(name in values and math.isfinite(values[name]) for name in names):
        raise RuntimeError('Joint state contains missing or non-finite positions.')
    state = RobotState()
    state.joint_state.header = copy.deepcopy(message.header)
    state.joint_state.name = names + ['joint8']
    state.joint_state.position = [values[name] for name in names] + [-values['joint7']]
    state.is_diff = True
    return state


def grasp_pose(cube: Pose, offset: float) -> Pose:
    pose = Pose()
    pose.position = copy.deepcopy(cube.position)
    pose.position.z += offset
    # Set all quaternion components: Quaternion defaults to w=1 in Humble.
    pose.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
    return pose


class PickCube(Node):
    def __init__(self):
        super().__init__('pick_cube')
        for name, value in {
            'approach_height': 0.04, 'lift_height': 0.05,
            'grasp_offset': 0.125, 'closed_position': 0.0193,
            'velocity_scale': 0.10,
            'stabilize_grasp': True,
            'cube_name': 'pick_cube',
        }.items():
            self.declare_parameter(name, value)
        self.joints: JointState | None = None
        self.joint_wall_time = 0.0
        self.cube_name = self.get_parameter('cube_name').get_parameter_value().string_value
        self.contacts: ContactsState | None = None
        self.contact_subscription = self.create_subscription(
            ContactsState, f'/{self.cube_name}/contacts', self.contact_callback, 10)
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.entity = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.cartesian = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.get_scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.apply_scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.validity = self.create_client(GetStateValidity, '/check_state_validity')
        self.stabilizer = self.create_client(SetBool, '/piper_grasp/enable')
        self.grasp_status = self.create_client(Trigger, '/piper_grasp/status')
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.execute = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.grip = ActionClient(
            self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.arm_names = [f'joint{i}' for i in range(1, 7)]

    def parameter(self, name: str) -> float:
        value = self.get_parameter(name).value
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise RuntimeError(f'{name} must be a numeric parameter.')
        return float(value)

    def bool_parameter(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if not isinstance(value, bool):
            raise RuntimeError(f'{name} must be a boolean parameter.')
        return value

    def joint_callback(self, message: JointState) -> None:
        self.joints = message
        self.joint_wall_time = time.monotonic()

    def contact_callback(self, message: ContactsState) -> None:
        self.contacts = message

    def wait(self, future, timeout=30.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            future.cancel()
            raise RuntimeError('Service/action request timed out; check Gazebo and MoveIt logs.')
        return future.result()

    def service(self, client, request):
        if not client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError(f'Service unavailable: {client.srv_name}')
        return self.wait(client.call_async(request))

    def action(self, client, goal, timeout=90.0):
        if not client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError(f'Action unavailable: {client._action_name}')
        handle = self.wait(client.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('Motion goal rejected.')
        future = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        except KeyboardInterrupt:
            if rclpy.ok():
                self.wait(handle.cancel_goal_async(), timeout=5.0)
            raise
        if not future.done():
            self.wait(handle.cancel_goal_async(), timeout=5.0)
            raise RuntimeError('Motion timed out; cancellation requested.')
        result = future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            error = result.result.error_code
            raise RuntimeError(
                f'Motion failed (status={result.status}, error={getattr(error, "val", error)}). '
                f'{getattr(result.result, "error_string", "")}')
        return result.result

    def current_state(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if (
                self.joints is not None
                and time.monotonic() - self.joint_wall_time < 1.0
                and set(self.arm_names + ['joint7']).issubset(self.joints.name)
            ):
                # gazebo_ros2_control exports joint8 as joint8_mimic. Never
                # send that hardware-only name (or NaN efforts) to MoveIt.
                return moveit_state(self.joints)
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError('No fresh complete /joint_states; check active controllers.')

    def entity_pose(self, name):
        request = GetEntityState.Request()
        request.name = name
        request.reference_frame = 'world'
        response = self.service(self.entity, request)
        if not response.success:
            raise RuntimeError(f'Gazebo entity {name} was not found.')
        return response.state.pose

    def cube_pose(self):
        return self.entity_pose(self.cube_name)

    def gripper(self, position):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint7']
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=2)
        goal.trajectory.points = [point]
        result = self.action(self.grip, goal, timeout=15.0)
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f'Gripper failed: {result.error_string}')

    def finger_contact(self, allow):
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        matrix = self.service(self.get_scene, request).scene.allowed_collision_matrix
        for name in [self.cube_name, 'link7', 'link8']:
            if name not in matrix.entry_names:
                matrix.entry_names.append(name)
                for row in matrix.entry_values:
                    row.enabled.append(False)
                matrix.entry_values.append(AllowedCollisionEntry(
                    enabled=[False] * len(matrix.entry_names)))
        cube = matrix.entry_names.index(self.cube_name)
        for link in ['link7', 'link8']:
            finger = matrix.entry_names.index(link)
            matrix.entry_values[cube].enabled[finger] = allow
            matrix.entry_values[finger].enabled[cube] = allow
        change = ApplyPlanningScene.Request()
        change.scene.is_diff = True
        change.scene.robot_state.is_diff = True
        change.scene.allowed_collision_matrix = matrix
        if not self.service(self.apply_scene, change).success:
            raise RuntimeError('Could not set finger/cube contact permissions.')

    def approach(self, pose):
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.ik_link_name = 'link6'
        request.ik_request.pose_stamped.header.frame_id = 'world'
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.robot_state = self.current_state()
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(sec=3)
        response = self.service(self.ik, request)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'Approach IK failed ({response.error_code.val}); cube may be unreachable.')
        positions = dict(zip(response.solution.joint_state.name,
                             response.solution.joint_state.position))
        self.move_arm(positions)

    def move_arm(self, positions):
        target = Constraints()
        target.joint_constraints = [JointConstraint(
            joint_name=name, position=positions[name],
            tolerance_above=0.002, tolerance_below=0.002, weight=1.0)
            for name in self.arm_names]
        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [target]
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = self.parameter('velocity_scale')
        goal.request.max_acceleration_scaling_factor = self.parameter('velocity_scale')
        goal.planning_options.plan_only = False
        result = self.action(self.move, goal)
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'Approach planning/execution failed ({result.error_code.val}).')

    def straight(self, pose):
        request = GetCartesianPath.Request()
        request.header.frame_id = 'world'
        request.start_state = self.current_state()
        request.group_name = 'arm'
        request.link_name = 'link6'
        request.waypoints = [pose]
        request.max_step = 0.003
        # Use an absolute per-step bound. Relative averaging can reject smooth
        # short paths when the first point corrects small tracking errors.
        request.jump_threshold = 0.0
        request.revolute_jump_threshold = 0.2
        request.prismatic_jump_threshold = 0.01
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = self.parameter('velocity_scale')
        request.max_acceleration_scaling_factor = self.parameter('velocity_scale')
        response = self.service(self.cartesian, request)
        if response.error_code.val != MoveItErrorCodes.SUCCESS or response.fraction < 0.995:
            check = GetPositionIK.Request()
            check.ik_request.group_name = 'arm'
            check.ik_request.ik_link_name = 'link6'
            check.ik_request.pose_stamped.header.frame_id = 'world'
            check.ik_request.pose_stamped.pose = pose
            check.ik_request.robot_state = request.start_state
            check.ik_request.timeout = Duration(sec=1)
            ik = self.service(self.ik, check)
            if ik.error_code.val == MoveItErrorCodes.SUCCESS:
                validity = self.service(self.validity, GetStateValidity.Request(
                    robot_state=ik.solution, group_name='arm'))
                for contact in validity.contacts:
                    self.get_logger().error(
                        f'Target collision: {contact.contact_body_1} / {contact.contact_body_2}')
            raise RuntimeError(f'Cartesian path incomplete ({response.fraction:.1%}); no motion executed.')
        # Stretch timing uniformly to keep contact moves slow, even on older Humble servers.
        trajectory = response.solution
        for point in trajectory.joint_trajectory.points:
            duration = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            nanoseconds = round(duration * 3.0 * 1e9)
            point.time_from_start = Duration(
                sec=nanoseconds // 1000000000, nanosec=nanoseconds % 1000000000)
            point.velocities = [v / 3.0 for v in point.velocities]
            point.accelerations = [a / 9.0 for a in point.accelerations]
        if len(trajectory.joint_trajectory.points) < 2:
            raise RuntimeError('Empty Cartesian trajectory.')
        result = self.action(self.execute, ExecuteTrajectory.Goal(trajectory=trajectory))
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'Cartesian execution failed ({result.error_code.val}).')

    def settle(self, seconds=1.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        if not (0.0 < self.parameter('velocity_scale') <= 0.3):
            raise RuntimeError('velocity_scale must be in (0, 0.3].')
        if not (0.0 < self.parameter('closed_position') < 0.02):
            raise RuntimeError('closed_position must be between 0 and 0.02 for the 4 cm cube.')
        for name in ['approach_height', 'lift_height', 'grasp_offset']:
            if not (0.0 < self.parameter(name) <= 0.2):
                raise RuntimeError(f'{name} must be in (0, 0.2] metres.')
        self.current_state()
        if self.bool_parameter('stabilize_grasp'):
            self.service(self.stabilizer, SetBool.Request(data=False))
        self.get_logger().info('1/5 Open gripper')
        self.gripper(0.035)
        self.settle(0.5)
        cube = self.cube_pose()
        if not all(math.isfinite(v) for v in [cube.position.x, cube.position.y, cube.position.z]):
            raise RuntimeError('Invalid cube pose.')
        grasp = grasp_pose(cube, self.parameter('grasp_offset'))
        above = copy.deepcopy(grasp)
        above.position.z += self.parameter('approach_height')
        self.get_logger().info('2/5 Plan and move above cube')
        self.approach(above)
        self.finger_contact(True)  # Only cube/fingers; table collisions remain enabled.
        self.get_logger().info('3/5 Descend vertically')
        self.straight(grasp)
        wrist = self.entity_pose('piper::link6')
        error = math.dist(
            [wrist.position.x, wrist.position.y, wrist.position.z],
            [grasp.position.x, grasp.position.y, grasp.position.z])
        # The target quaternion is (0, 1, 0, 0), so its dot product is q.y.
        if error > 0.005 or abs(wrist.orientation.y) < math.cos(0.03 / 2.0):
            raise RuntimeError('Gazebo wrist did not reach the commanded grasp pose.')
        self.get_logger().info('4/5 Close gripper')
        if self.bool_parameter('stabilize_grasp'):
            # Arm while closing: capture first stable bilateral contact before
            # continued position control can tilt the cube against the table.
            self.service(self.stabilizer, SetBool.Request(data=True))
        self.gripper(self.parameter('closed_position'))
        self.settle()
        names = ' '.join(s.collision1_name + ' ' + s.collision2_name
                         for s in self.contacts.states) if self.contacts else ''
        if not all(f'::link{i}::' in names for i in (7, 8)):
            for name in [self.cube_name, 'piper::link6', 'piper::link7', 'piper::link8']:
                self.get_logger().error(f'Contact diagnostic {name}: {self.entity_pose(name)}')
            raise RuntimeError(f'Both finger contacts not detected: {names}')
        if self.bool_parameter('stabilize_grasp'):
            self.service(self.stabilizer, SetBool.Request(data=True))
            deadline = time.monotonic() + 5.0
            while not self.service(self.grasp_status, Trigger.Request()).success:
                if time.monotonic() >= deadline:
                    raise RuntimeError('Grasp constraint not confirmed; no lift executed.')
                self.settle(0.1)
            self.get_logger().info('Both contacts confirmed; grasp stabilization enabled.')
        self.get_logger().info('5/5 Lift vertically')
        raised = copy.deepcopy(grasp)
        raised.position.z += self.parameter('lift_height')
        self.straight(raised)
        self.settle(2.0)
        actual = self.cube_pose()
        rise = actual.position.z - cube.position.z
        lateral = math.hypot(actual.position.x - cube.position.x,
                             actual.position.y - cube.position.y)
        if not (self.parameter('lift_height') * 0.6 <= rise <=
                self.parameter('lift_height') * 1.4) or lateral > 0.03:
            raise RuntimeError(f'Arm lifted but physical grasp failed: cube rose only {rise:.3f} m.')
        self.get_logger().info(f'SUCCESS: Gazebo cube lifted {rise:.3f} m; gripper stays closed.')


def main(args=None):
    rclpy.init(args=args)
    node = PickCube()
    code = 0
    try:
        node.run()
    except (RuntimeError, KeyboardInterrupt) as exc:
        node.get_logger().error(f'Grasp stopped: {exc}')
        code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
