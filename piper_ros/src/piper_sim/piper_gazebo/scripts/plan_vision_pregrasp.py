#!/usr/bin/env python3
"""Plan/display a fresh Gazebo vision hover; movement requires execute:=true."""
import copy
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from piper_moveit_bridge.reachability import wrist_reach_bound
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    CollisionObject, Constraints, DisplayTrajectory, OrientationConstraint,
    PlanningSceneComponents, PositionConstraint, JointConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPlanningScene, GetPositionIK, GetStateValidity
from moveit_msgs.action import ExecuteTrajectory
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header, String
import numpy as np
from scipy.spatial.transform import Rotation

from pick_cube import moveit_state
from sync_pick_scene import compose_pose, sdf_pose


def goal_constraints(target):
    p = target.pose.position
    q = target.pose.orientation
    if target.header.frame_id != 'base_link':
        raise ValueError('Preview target must be in base_link.')
    if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
        raise ValueError('Non-finite preview pose.')
    if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) > .001:
        raise ValueError('Preview quaternion must be normalized.')
    c = Constraints(name='gazebo_vision_hover')
    position = PositionConstraint()
    position.header = Header(frame_id='base_link')
    position.link_name = 'link6'
    position.weight = 1.
    sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[.003])
    region = Pose()
    region.position = copy.deepcopy(p)
    region.orientation.w = 1.
    position.constraint_region.primitives = [sphere]
    position.constraint_region.primitive_poses = [region]
    orientation = OrientationConstraint()
    orientation.header = Header(frame_id='base_link')
    orientation.link_name = 'link6'
    orientation.orientation = copy.deepcopy(q)
    orientation.absolute_x_axis_tolerance = .03
    orientation.absolute_y_axis_tolerance = .03
    orientation.absolute_z_axis_tolerance = .03
    orientation.weight = 1.
    c.position_constraints = [position]
    c.orientation_constraints = [orientation]
    return c


def bottle_collision(name, link_pose):
    # Mirror actual compound collision geometry; a full-width cylinder around
    # the neck creates false camera/gripper collisions during close approach.
    obj = CollisionObject()
    obj.header.frame_id = 'world'
    obj.id = name
    obj.operation = CollisionObject.ADD
    local_path = Path(__file__).resolve().parents[1]/'worlds/bottle.sdf'
    path = local_path if local_path.is_file() else Path(get_package_share_directory('piper_gazebo'))/'worlds/bottle.sdf'
    root = ET.parse(path).getroot()
    for collision in root.findall('model/link/collision'):
        geometry = collision.find('geometry')
        shape = SolidPrimitive()
        if geometry.find('cylinder') is not None:
            shape.type = SolidPrimitive.CYLINDER
            shape.dimensions = [float(geometry.findtext('cylinder/length')), float(geometry.findtext('cylinder/radius'))]
        elif geometry.find('sphere') is not None:
            shape.type = SolidPrimitive.SPHERE
            shape.dimensions = [float(geometry.findtext('sphere/radius'))]
        else:
            raise ValueError('Unsupported bottle collision shape.')
        obj.primitives.append(shape)
        obj.primitive_poses.append(compose_pose(link_pose, sdf_pose(collision.find('pose'))))
    if not obj.primitives:
        raise ValueError('Bottle model has no collision geometry.')
    return obj


def adjusted_hover(pose, offset, tilt_deg, yaw_deg):
    if not math.isfinite(tilt_deg) or not 0 <= tilt_deg <= 45:
        raise ValueError('Simulation hover tilt must be within 0..45 degrees.')
    result = copy.deepcopy(pose)
    q = result.pose.orientation
    rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
    p = result.pose.position
    tcp = np.array([p.x, p.y, p.z]) + rotation.apply(offset)
    rotated = Rotation.from_euler('z', yaw_deg, degrees=True) * rotation
    if tilt_deg:
        radial = np.array([tcp[0], tcp[1], 0.])
        if np.linalg.norm(radial) < .05:
            raise ValueError('TCP too close to base axis to define radial hover tilt.')
        radial /= np.linalg.norm(radial)
        axis = np.cross(radial, [0., 0., 1.])
        rotated = Rotation.from_rotvec(axis*math.radians(tilt_deg)) * rotated
    position = tcp - rotated.apply(offset)
    result.pose.position.x, result.pose.position.y, result.pose.position.z = map(float, position)
    q.x, q.y, q.z, q.w = map(float, rotated.as_quat())
    return result


class VisionPlan(Node):
    def __init__(self):
        super().__init__('gazebo_vision_pregrasp_plan', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        if not self.get_parameter('use_sim_time').value:
            raise ValueError('This planner requires use_sim_time=true.')
        self.bottle = self.declare_parameter('bottle_name', 'vision_bottle').value
        self.execute_requested = self.declare_parameter('execute', False).value
        self.tilt_deg = float(self.declare_parameter('tilt_deg', 0.).value)
        self.yaw_deg = float(self.declare_parameter('yaw_offset_deg', 0.).value)
        if not math.isfinite(self.tilt_deg) or not 0 <= self.tilt_deg <= 45:
            raise ValueError('Simulation tilt_deg must be within 0..45 degrees.')
        if not math.isfinite(self.yaw_deg) or not -180 <= self.yaw_deg <= 180:
            raise ValueError('yaw_offset_deg must be within -180..180 degrees.')
        self.output = self.declare_parameter('output_file',
            str(Path.home() / 'GraspGen/calibration/sim_pregrasp_plan.json')).value
        self.inputs = {}
        self.subscriptions_kept = [self.create_subscription(kind, topic,
            lambda msg, key=key: self.inputs.update({key: (time.monotonic(), msg)}), 10)
            for key, kind, topic in [
                ('joints', JointState, '/joint_states'),
                ('target', PoseStamped, '/grasp_preview/pregrasp_link6'),
                ('status', String, '/grasp_preview/status')]]
        self.display = self.create_publisher(DisplayTrajectory, '/display_planned_path', 10)

    def spin_wait(self, predicate, seconds=20.):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            if predicate():
                return
        raise RuntimeError('Timed out waiting for MoveIt or fresh simulation inputs.')

    def call(self, kind, name, request):
        client = self.create_client(kind, name)
        try:
            if not client.wait_for_service(timeout_sec=10.):
                raise RuntimeError(f'Service unavailable: {name}')
            future = client.call_async(request)
            self.spin_wait(future.done, 25.)
            return future.result()
        finally:
            self.destroy_client(client)

    def fresh_inputs(self):
        if set(self.inputs) != {'joints', 'target', 'status'}:
            return False
        if any(time.monotonic() - received > .8 for received, _ in self.inputs.values()):
            return False
        status = json.loads(self.inputs['status'][1].data)
        if not status.get('target_valid') or not status.get('current_tcp_valid'):
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        for key, max_age in [('joints', .5), ('target', 1.)]:
            stamp = self.inputs[key][1].header.stamp
            t = stamp.sec + stamp.nanosec / 1e9
            if t <= 0 or not -.1 <= now - t <= max_age:
                return False
        return True

    def run(self):
        self.spin_wait(self.fresh_inputs)
        # Gazebo ground truth is used only to model the collision obstacle.
        # The motion goal always comes from the vision/TF preview topic.
        query = GetEntityState.Request()
        query.name = f'{self.bottle}::bottle_link'
        query.reference_frame = 'world'
        entity = self.call(GetEntityState, '/gazebo/get_entity_state', query)
        if not entity.success:
            raise RuntimeError(f'Bottle link not found: {query.name}; expected supplied bottle.sdf.')
        apply = ApplyPlanningScene.Request()
        apply.scene.is_diff = True
        apply.scene.robot_state.is_diff = True
        apply.scene.world.collision_objects = [bottle_collision(self.bottle, entity.state.pose)]
        if not self.call(ApplyPlanningScene, '/apply_planning_scene', apply).success:
            raise RuntimeError('MoveIt rejected bottle collision geometry.')
        scene_request = GetPlanningScene.Request()
        scene_request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        scene = self.call(GetPlanningScene, '/get_planning_scene', scene_request).scene
        names = {obj.id for obj in scene.world.collision_objects}
        if not {'pick_table', self.bottle}.issubset(names):
            raise RuntimeError('Planning scene must contain pick_table and the bottle; keep scene synchronization running.')
        self.spin_wait(self.fresh_inputs)
        target = copy.deepcopy(self.inputs['target'][1])
        start = moveit_state(self.inputs['joints'][1])
        preview = json.loads(self.inputs['status'][1].data)
        target = adjusted_hover(target, preview.get('tcp_translation_m', [0., 0., .1358]),
                                self.tilt_deg, self.yaw_deg)
        self.check_reach(target)
        # Distinguish an invalid start, no IK and a colliding goal before
        # asking OMPL to search. Do not disable collision checks to proceed.
        check = GetStateValidity.Request()
        check.group_name = 'arm'
        check.robot_state = start
        validity = self.call(GetStateValidity, '/check_state_validity', check)
        if not validity.valid:
            contacts = [(c.contact_body_1, c.contact_body_2) for c in validity.contacts]
            raise RuntimeError(f'Current robot state is invalid; contacts={contacts}. No execution requested.')
        ik = GetPositionIK.Request()
        ik.ik_request.group_name = 'arm'
        ik.ik_request.ik_link_name = 'link6'
        ik.ik_request.robot_state = start
        ik.ik_request.pose_stamped = target
        ik.ik_request.avoid_collisions = True
        ik.ik_request.timeout.sec = 1
        solution = self.call(GetPositionIK, '/compute_ik', ik)
        if solution.error_code.val != 1:
            # A non-collision IK query is diagnostic only; its answer is never
            # used as an execution/planning goal without collision validation.
            ik.ik_request.avoid_collisions = False
            diagnostic = self.call(GetPositionIK, '/compute_ik', ik)
            if diagnostic.error_code.val != 1:
                raise RuntimeError(f'No IK found for this hover orientation (error={diagnostic.error_code.val}, tilt_deg={self.tilt_deg:g}, yaw_offset_deg={self.yaw_deg:g}). Change explicit hover orientation or observation/object configuration; increasing planning time alone will not fix an unreachable goal.')
            check.robot_state = diagnostic.solution
            validity = self.call(GetStateValidity, '/check_state_validity', check)
            contacts = [(c.contact_body_1, c.contact_body_2) for c in validity.contacts]
            raise RuntimeError(f'Collision-aware IK failed; diagnostic goal valid={validity.valid}, contacts={contacts}. No execution requested.')
        request = GetMotionPlan.Request()
        m = request.motion_plan_request
        m.group_name = 'arm'
        m.start_state = start
        # Planning to the collision-checked IK solution avoids repeated random
        # sampling of a narrow pose-constrained goal region by OMPL.
        values = dict(zip(solution.solution.joint_state.name, solution.solution.joint_state.position))
        constraints = Constraints(name='vision_hover_validated_ik')
        constraints.joint_constraints = [JointConstraint(joint_name=f'joint{i}',
            position=values[f'joint{i}'], tolerance_above=.0001, tolerance_below=.0001,
            weight=1.) for i in range(1, 7)]
        m.goal_constraints = [constraints]
        m.allowed_planning_time = 10.
        m.num_planning_attempts = 5
        m.max_velocity_scaling_factor = .10
        m.max_acceleration_scaling_factor = .10
        p = target.pose.position
        self.get_logger().info(f'PLAN ONLY: link6 base XYZ = {p.x:.4f}, {p.y:.4f}, {p.z:.4f} m')
        plan = self.call(GetMotionPlan, '/plan_kinematic_path', request).motion_plan_response
        if plan.error_code.val != 1 or not plan.trajectory.joint_trajectory.points:
            raise RuntimeError(f'MoveIt planning failed: error_code={plan.error_code.val}; inspect move_group logs for reachability/collisions.')
        out = Path(self.output).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps({
            'plan_only': True, 'simulation': True,
            'bottle_collision_is_snapshot': True,
            'preview_at_plan': preview,
            'hover_tilt_deg': self.tilt_deg,
            'hover_yaw_offset_deg': self.yaw_deg,
            'target_link6': message_to_ordereddict(target),
            'start_state': message_to_ordereddict(plan.trajectory_start),
            'trajectory': message_to_ordereddict(plan.trajectory),
        }, indent=2) + '\n')
        display = DisplayTrajectory()
        display.model_id = 'piper'
        display.trajectory_start = plan.trajectory_start
        display.trajectory = [plan.trajectory]
        end = time.monotonic() + 2.
        while time.monotonic() < end:
            self.display.publish(display)
            rclpy.spin_once(self, timeout_sec=.1)
        if not self.execute_requested:
            self.get_logger().info(f'Plan succeeded; saved {out}. View Planned Path in RViz. No execution requested.')
            return
        self.execute_hover(plan, query, entity.state.pose)

    def execute_hover(self, plan, bottle_query, original_bottle):
        # Execute the fresh in-memory plan only; saved JSON is never replayed.
        self.spin_wait(self.fresh_inputs)
        current = dict(zip(self.inputs['joints'][1].name, self.inputs['joints'][1].position))
        before = dict(zip(plan.trajectory_start.joint_state.name, plan.trajectory_start.joint_state.position))
        if any(abs(current[f'joint{i}']-before[f'joint{i}']) > .01 for i in range(1, 7)):
            raise RuntimeError('Arm moved since planning; rerun for a fresh plan.')
        actual_bottle = self.call(GetEntityState, '/gazebo/get_entity_state', bottle_query)
        if not actual_bottle.success:
            raise RuntimeError('Bottle state unavailable; execution skipped.')
        a, b = original_bottle.position, actual_bottle.state.pose.position
        if math.dist([a.x, a.y, a.z], [b.x, b.y, b.z]) > .005:
            raise RuntimeError('Bottle moved since planning; execution skipped.')
        client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        try:
            if not client.wait_for_server(timeout_sec=10.):
                raise RuntimeError('MoveIt execute action unavailable.')
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = plan.trajectory
            send = client.send_goal_async(goal)
            self.spin_wait(send.done)
            handle = send.result()
            if not handle.accepted:
                raise RuntimeError('Hover execution rejected.')
            result = handle.get_result_async()
            duration = plan.trajectory.joint_trajectory.points[-1].time_from_start
            try:
                self.spin_wait(result.done, duration.sec+duration.nanosec/1e9+20.)
            except (RuntimeError, KeyboardInterrupt):
                cancel = handle.cancel_goal_async()
                self.spin_wait(cancel.done, 5.)
                raise
            response = result.result()
            if response.status != GoalStatus.STATUS_SUCCEEDED or response.result.error_code.val != 1:
                raise RuntimeError(f'Hover execution failed: status={response.status}, error={response.result.error_code.val}.')
            # Allow feedback to arrive before verifying endpoint positions.
            finish = plan.trajectory.joint_trajectory
            expected = dict(zip(finish.joint_names, finish.points[-1].positions))
            def arrived():
                if 'joints' not in self.inputs or time.monotonic()-self.inputs['joints'][0] > .5:
                    return False
                values = dict(zip(self.inputs['joints'][1].name, self.inputs['joints'][1].position))
                return all(abs(values.get(n, float('inf'))-v) < .01 for n, v in expected.items())
            self.spin_wait(arrived, 5.)
            self.get_logger().info('HOVER REACHED: controller succeeded and joint feedback matches. No descent or gripper command was sent.')
        finally:
            client.destroy()

    def check_reach(self, target):
        # Use actual link dimensions and target orientation, including the
        # wrist offset. Passing this necessary bound does not guarantee IK.
        goal_constraints(target)
        model = Path(get_package_share_directory('piper_description')) / 'urdf/piper_description_gazebo.xacro'
        bound = wrist_reach_bound(target, model)
        if bound and bound['outside_geometric_bound']:
            raise ValueError(
                f"Target outside geometric reach for this orientation: wrist needs at least "
                f"{bound['wrist_distance_min_m']:.3f} m; main arm bound is "
                f"{bound['main_arm_length_sum_m']:.3f} m. Move the bottle closer to "
                'the base, obtain a fresh vision target and rerun. No planning/execution requested.')


def main():
    rclpy.init()
    node = None
    try:
        node = VisionPlan()
        node.run()
    except (RuntimeError, ValueError, KeyError) as exc:
        if node:
            node.get_logger().error(str(exc))
        raise SystemExit(1)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
