#!/usr/bin/env python3
"""Find a collision-checked camera observation pose for the supplied Gazebo bottle."""
import math
import time
from collections import Counter
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from gazebo_msgs.srv import GetEntityState
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import DisplayTrajectory, JointConstraint, Constraints, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPositionIK, GetPlanningScene, GetStateValidity
from rclpy.action import ActionClient
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from scipy.optimize import differential_evolution
from tf2_ros import Buffer, TransformListener

from pick_cube import moveit_state
from plan_vision_pregrasp import VisionPlan, bottle_collision
from bottle_model import dimensions, centered_surface_points


def transform_matrix(transform):
    p, q = transform.translation, transform.rotation
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    t[:3, 3] = [p.x, p.y, p.z]
    return t


def camera_view(center, offset, roll=0.):
    """Optical +Z faces the object; image +Y points down for roll=0."""
    position = np.asarray(center) + offset
    z = np.asarray(center) - position
    z /= np.linalg.norm(z)
    x = np.cross(z, [0., 0., 1.])
    if np.linalg.norm(x) < 1e-6:
        x = np.array([1., 0., 0.])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    t = np.eye(4)
    t[:3, :3] = np.column_stack((x, y, z)) @ Rotation.from_euler('z', roll).as_matrix()
    t[:3, 3] = position
    return t


def pose_message(t):
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, t[:3, 3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(
        float, Rotation.from_matrix(t[:3, :3]).as_quat())
    return p


def bottle_points(center):
    # Sample the supplied opaque bottle surfaces; a full-height bounding box
    # incorrectly treats the narrow cap as having the body's full width.
    return centered_surface_points() + np.asarray(center)


def bottle_fits(camera, center, surface_points=None):
    points = bottle_points(center) if surface_points is None else surface_points+np.asarray(center)
    optical = (points-camera[:3, 3]) @ camera[:3, :3]
    z = optical[:, 2]
    # Matches d455_gazebo.xacro's ideal projection (640x480, hfov 1.211).
    return bool(np.all((z > .35) & (z < 1.2)) and
                np.all(np.abs(optical[:, 0] / z) < .98 * math.tan(1.211/2)) and
                np.all(np.abs(optical[:, 1] / z) < .98 * .75 * math.tan(1.211/2)))


def image_pose_cost(cameras, center):
    """Prefer upright, centered bottles; geometric visibility alone is not enough."""
    mid = np.einsum('nji,nj->ni', cameras[:, :3, :3], np.asarray(center)-cameras[:, :3, 3])
    # World +Z (bottle up) expressed in optical coordinates. Optical +Y is down.
    up = cameras[:, 2, :2]
    norm = np.maximum(np.linalg.norm(up, axis=1), 1e-6)
    sideways = (up[:, 0]/norm)**2
    inverted = np.maximum(up[:, 1]/norm, 0.)**2
    off_center = np.sum((mid[:, :2]/np.maximum(mid[:, 2, None], .01))**2, axis=1)
    return sideways + 4.*inverted + .25*off_center


def sample_joint_views(model_path, mount, center, current, count=60000, surface_points=None):
    """Search feasible joint limits directly, without fixing camera roll/pose.

    Vectorized URDF FK screens the complete bottle box against image/depth
    margins. MoveIt still checks collisions and plans every accepted endpoint.
    """
    root = ET.parse(model_path).getroot()
    joints = [root.find(f"joint[@name='joint{i}']") for i in range(1, 7)]
    low = np.array([float(j.find('limit').get('lower')) for j in joints])
    high = np.array([float(j.find('limit').get('upper')) for j in joints])
    rng = np.random.default_rng(455)
    values = rng.uniform(low, high, (count, 6))
    values[0] = np.clip(current, low, high)
    # Include local variations for shorter paths near an already useful view.
    local = count//4
    values[1:local] = np.clip(rng.normal(current, .5, (local-1, 6)), low, high)
    fixed_transforms = []
    for index, joint in enumerate(joints):
        if joint.find('axis').get('xyz').split() != ['0', '0', '1']:
            raise ValueError('Observation sampler requires standard PiPER Z joint axes.')
        origin = joint.find('origin')
        fixed = np.eye(4)
        fixed[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        fixed[:3, :3] = Rotation.from_euler('xyz', [float(v) for v in origin.get('rpy', '0 0 0').split()]).as_matrix()
        fixed_transforms.append(fixed)
    def cameras_for(q):
        size = len(q)
        transforms = np.broadcast_to(np.eye(4), (size, 4, 4)).copy()
        for index, fixed in enumerate(fixed_transforms):
            moving = np.broadcast_to(np.eye(4), (size, 4, 4)).copy()
            c, s = np.cos(q[:, index]), np.sin(q[:, index])
            moving[:, 0, 0] = c; moving[:, 1, 1] = c
            moving[:, 0, 1] = -s; moving[:, 1, 0] = s
            transforms = transforms @ fixed @ moving
        return transforms @ mount
    corners = bottle_points(center) if surface_points is None else surface_points+np.asarray(center)
    def optimize_cost(q):
        cameras = cameras_for(np.asarray(q).reshape(1, 6))
        p = np.einsum('nji,nkj->nki', cameras[:, :3, :3], corners[None, :, :] - cameras[:, None, :3, 3])
        z = p[:, :, 2]
        loss = np.sum(np.maximum(.352-z, 0)**2 + np.maximum(z-1.19, 0)**2, axis=1)
        loss += np.sum(np.maximum(np.abs(p[:, :, 0])-.97*math.tan(1.211/2)*z, 0)**2, axis=1)
        loss += np.sum(np.maximum(np.abs(p[:, :, 1])-.97*.75*math.tan(1.211/2)*z, 0)**2, axis=1)
        # Prefer viewing from above the bottle center, not from beneath the table.
        loss += np.maximum(center[2]+.12-cameras[:, 2, 3], 0)**2
        loss += .00002*image_pose_cost(cameras, center)
        return float(loss[0])
    # Random samples miss narrow feasible regions near wrist joint limits.
    # Optimize the framing inequalities directly instead of prescribing an
    # exact camera pose or roll. Collision checks remain in MoveIt below.
    optimized = []
    for seed in [455, 456, 457]:
        result = differential_evolution(optimize_cost, list(zip(low, high)), seed=seed,
            popsize=12, maxiter=180, updating='deferred',
            polish=True, tol=1e-7)
        optimized.append(result.x)
    values = np.vstack((values, optimized))
    cameras = cameras_for(values)
    optical = np.einsum('nji,nkj->nki', cameras[:, :3, :3], corners[None, :, :] - cameras[:, None, :3, 3])
    depth = optical[:, :, 2]
    # Use multiplication instead of division so behind-camera points never pass.
    visible = np.all((depth > .35) & (depth < 1.2) &
        (np.abs(optical[:, :, 0]) < .98*math.tan(1.211/2)*depth) &
        (np.abs(optical[:, :, 1]) < .98*.75*math.tan(1.211/2)*depth), axis=1)
    visible &= cameras[:, 2, 3] > center[2]+.12
    indices = np.flatnonzero(visible)
    if not len(indices):
        return []
    mid = optical[indices].mean(axis=1)
    score = (mid[:, 0]/mid[:, 2])**2 + (mid[:, 1]/mid[:, 2])**2
    score += .08*(mid[:, 2]-.52)**2 + .001*np.sum((values[indices]-current)**2, axis=1)
    score += image_pose_cost(cameras[indices], center)
    # Preserve distinct joint configurations instead of checking nearly identical
    # samples from one collision basin over and over.
    ordered = indices[np.argsort(score)]
    selected = []
    for index in ordered:
        if all(np.linalg.norm(values[index]-q) > .3 for q in selected):
            selected.append(values[index])
        if len(selected) == 120:
            break
    return selected


class ObserveBottle(VisionPlan):
    def __init__(self):
        super().__init__()
        self.execute_requested = self.get_parameter('execute').value
        self.target_geometry = self.declare_parameter('target_geometry', 'bottle').value
        if self.target_geometry not in ('bottle', 'cube'):
            raise ValueError('target_geometry must be bottle or cube')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def joints_fresh(self):
        if 'joints' not in self.inputs:
            return False
        received, msg = self.inputs['joints']
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        age = self.get_clock().now().nanoseconds/1e9 - stamp
        return time.monotonic()-received < .8 and stamp > 0 and -.1 <= age < .5

    def plan_joints(self, start, values):
        request = GetMotionPlan.Request()
        m = request.motion_plan_request
        m.group_name = 'arm'
        m.start_state = start
        c = Constraints(name='bottle_camera_observation')
        c.joint_constraints = [JointConstraint(joint_name=f'joint{i}', position=float(values[f'joint{i}']),
            tolerance_above=.001, tolerance_below=.001, weight=1.) for i in range(1, 7)]
        m.goal_constraints = [c]
        m.allowed_planning_time = 3.
        m.num_planning_attempts = 3
        m.max_velocity_scaling_factor = .1
        m.max_acceleration_scaling_factor = .1
        return self.call(GetMotionPlan, '/plan_kinematic_path', request).motion_plan_response

    def run(self):
        self.spin_wait(self.joints_fresh)
        self.spin_wait(lambda: self.buffer.can_transform('base_link', 'world', Time()) and
                       self.buffer.can_transform('link6', 'camera_color_optical_frame', Time()))
        query = GetEntityState.Request()
        query.name = f'{self.bottle}::' + ('body' if self.target_geometry == 'cube' else 'bottle_link')
        query.reference_frame = 'world'
        entity = self.call(GetEntityState, '/gazebo/get_entity_state', query)
        if not entity.success:
            raise RuntimeError('Cannot find supplied bottle; check bottle_name parameter.')
        apply = ApplyPlanningScene.Request()
        apply.scene.is_diff = True
        apply.scene.robot_state.is_diff = True
        if self.target_geometry == 'bottle':
            apply.scene.world.collision_objects = [bottle_collision(self.bottle, entity.state.pose)]
        if self.target_geometry == 'bottle' and not self.call(ApplyPlanningScene, '/apply_planning_scene', apply).success:
            raise RuntimeError('Bottle collision update failed.')
        scene_request = GetPlanningScene.Request()
        scene_request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        scene = self.call(GetPlanningScene, '/get_planning_scene', scene_request).scene
        if 'pick_table' not in {obj.id for obj in scene.world.collision_objects}:
            raise RuntimeError('Table collision model missing; keep pick scene synchronization running.')
        # Observation does not need any visual detection: simulation ground truth
        # selects the viewpoint only. Subsequent grasp goals still use vision.
        world_base = transform_matrix(self.buffer.lookup_transform('base_link', 'world', Time()).transform)
        pose = entity.state.pose
        world_bottle = np.eye(4)
        q = pose.orientation
        world_bottle[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        world_bottle[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        base_bottle = world_base @ world_bottle
        if np.dot(base_bottle[:3, 2], [0, 0, 1]) < .98:
            raise RuntimeError('Bottle must be upright for this observation search.')
        center = (base_bottle @ [0., 0., 0. if self.target_geometry == 'cube' else dimensions()['center_z'], 1.])[:3]
        surface_points = (np.array([[x, y, z] for x in [-.02, .02] for y in [-.02, .02]
                                   for z in [-.02, .02]]) if self.target_geometry == 'cube' else None)
        mount = transform_matrix(self.buffer.lookup_transform(
            'link6', 'camera_color_optical_frame', Time()).transform)
        radial = np.array([center[0], center[1], 0.])
        radial /= max(np.linalg.norm(radial), 1e-6)
        side = np.cross([0., 0., 1.], radial)
        self.get_logger().info('Searching camera viewpoints with >0.35m depth; no vision candidate required.')
        plan = None
        goal_values = None
        failures = Counter()
        for back, height, lateral in [(.32, .36, 0), (.36, .40, 0), (.28, .44, 0),
                                      (.32, .40, .15), (.32, .40, -.15), (.40, .36, 0)]:
            for roll in [0., math.pi/2, -math.pi/2, math.pi]:
                camera = camera_view(center, -back*radial + lateral*side + [0., 0., height], roll)
                if not bottle_fits(camera, center, surface_points):
                    failures['outside_image_or_depth'] += 1
                    continue
                self.spin_wait(self.joints_fresh)
                start = moveit_state(self.inputs['joints'][1])
                ik = GetPositionIK.Request()
                ik.ik_request.group_name = 'arm'
                ik.ik_request.ik_link_name = 'link6'
                ik.ik_request.robot_state = start
                ik.ik_request.avoid_collisions = True
                ik.ik_request.timeout.nanosec = 200000000
                target = PoseStamped()
                target.header.frame_id = 'base_link'
                target.pose = pose_message(camera @ np.linalg.inv(mount))
                ik.ik_request.pose_stamped = target
                solution = self.call(GetPositionIK, '/compute_ik', ik)
                if solution.error_code.val != 1:
                    failures[f'IK_error_{solution.error_code.val}'] += 1
                    continue
                values = dict(zip(solution.solution.joint_state.name, solution.solution.joint_state.position))
                result = self.plan_joints(start, values)
                if result.error_code.val == 1 and result.trajectory.joint_trajectory.points:
                    plan, goal_values = result, values
                    break
                failures[f'planning_error_{result.error_code.val}'] += 1
            if plan is not None:
                break
        if plan is None:
            self.get_logger().info(f'Fixed-view search: {dict(failures)}. Searching joint configurations with free camera orientation.')
            self.spin_wait(self.joints_fresh)
            start = moveit_state(self.inputs['joints'][1])
            current = dict(zip(start.joint_state.name, start.joint_state.position))
            model = Path(get_package_share_directory('piper_description')) / 'urdf/piper_description_gazebo.xacro'
            candidates = sample_joint_views(model, mount, center,
                np.array([current[f'joint{i}'] for i in range(1, 7)]), surface_points=surface_points)
            self.get_logger().info(f'Joint-space search found {len(candidates)} distinct image/depth candidates; checking collisions.')
            attempts = 0
            for candidate in candidates:
                values = {f'joint{i}': float(candidate[i-1]) for i in range(1, 7)}
                check = GetStateValidity.Request()
                check.group_name = 'arm'
                check.robot_state = moveit_state(self.inputs['joints'][1])
                check.robot_state.is_diff = False
                check.robot_state.joint_state.position = [values.get(n, current.get(n, 0.))
                    for n in check.robot_state.joint_state.name]
                valid = self.call(GetStateValidity, '/check_state_validity', check)
                if not valid.valid:
                    failures['sample_collision_or_invalid'] += 1
                    for contact in valid.contacts[:2]:
                        failures[f'contact:{contact.contact_body_1}/{contact.contact_body_2}'] += 1
                    continue
                self.spin_wait(self.joints_fresh)
                start = moveit_state(self.inputs['joints'][1])
                result = self.plan_joints(start, values)
                if result.error_code.val == 1 and result.trajectory.joint_trajectory.points:
                    plan, goal_values = result, values
                    break
                failures[f'planning_error_{result.error_code.val}'] += 1
                attempts += 1
                if attempts >= 5:
                    break
        if plan is None:
            raise RuntimeError(f'No collision-free observation path found. Bottle center base={center.tolist()}; diagnostics={dict(failures)}')
        self.get_logger().info('Observation joints (degrees): ' + ', '.join(
            f'joint{i}={math.degrees(goal_values[f"joint{i}"]):.2f}' for i in range(1, 7)))
        display = DisplayTrajectory()
        display.model_id = 'piper'
        display.trajectory_start = plan.trajectory_start
        display.trajectory = [plan.trajectory]
        end = time.monotonic()+2.
        while time.monotonic() < end:
            self.display.publish(display)
            rclpy.spin_once(self, timeout_sec=.1)
        if not self.execute_requested:
            self.get_logger().info('PLAN ONLY: view Planned Path. To request a fresh simulation plan and movement use -p execute:=true.')
            return
        self.spin_wait(self.joints_fresh)
        actual = dict(zip(self.inputs['joints'][1].name, self.inputs['joints'][1].position))
        before = dict(zip(plan.trajectory_start.joint_state.name, plan.trajectory_start.joint_state.position))
        if any(abs(actual[f'joint{i}']-before[f'joint{i}']) > .01 for i in range(1, 7)):
            raise RuntimeError('Robot moved during planning; rerun to obtain a fresh path.')
        client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        if not client.wait_for_server(timeout_sec=10.):
            raise RuntimeError('MoveIt execute action unavailable.')
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = plan.trajectory
        send = client.send_goal_async(goal)
        self.spin_wait(send.done)
        handle = send.result()
        if not handle.accepted:
            raise RuntimeError('Observation execution rejected.')
        result = handle.get_result_async()
        duration = plan.trajectory.joint_trajectory.points[-1].time_from_start
        try:
            self.spin_wait(result.done, duration.sec+duration.nanosec/1e9+20.)
        except (RuntimeError, KeyboardInterrupt):
            cancel = handle.cancel_goal_async()
            self.spin_wait(cancel.done, 5.)
            raise
        if result.result().status != GoalStatus.STATUS_SUCCEEDED or result.result().result.error_code.val != 1:
            raise RuntimeError('Observation execution failed; inspect MoveIt logs.')
        self.get_logger().info('Observation motion completed. Check actual image and /graspgen/status; recognition is not guaranteed.')


def main():
    rclpy.init()
    node = None
    try:
        node = ObserveBottle()
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
