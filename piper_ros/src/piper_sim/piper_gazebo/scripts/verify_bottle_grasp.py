#!/usr/bin/env python3
"""Validate Gazebo descent, bilateral pinch and a 10 mm physical lift."""
import copy
import json
import math
import subprocess
import time
import xml.etree.ElementTree as ET
from gazebo_msgs.srv import SetEntityState
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningSceneComponents, AllowedCollisionEntry
from moveit_msgs.srv import GetCartesianPath, GetPlanningScene, ApplyPlanningScene
from rclpy.parameter import Parameter
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from pick_cube import PickCube
from plan_vision_pregrasp import bottle_collision
from piper_moveit_bridge.cartesian_step import retime
from bottle_model import dimensions


def pose_matrix(p):
    t = np.eye(4)
    q = p.orientation
    t[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    t[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return t


def matrix_pose(t):
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, t[:3, 3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, Rotation.from_matrix(t[:3, :3]).as_quat())
    return p


def level_opening_direction(wrist, tcp_z=.1358):
    """Make gripper +Y (jaw opening) horizontal while preserving the TCP.

    This is not a vertical-down tool constraint: keep the closest approach
    direction perpendicular to the leveled jaw axis, retaining tool tilt.
    World Z is the table normal in this Gazebo scene.
    """
    t = pose_matrix(wrist)
    tcp = (t @ [0., 0., tcp_z, 1.])[:3]
    y = t[:3, 1].copy()
    y[2] = 0.
    if np.linalg.norm(y) < .2:
        raise ValueError('Jaw axis too vertical to infer its horizontal heading.')
    y /= np.linalg.norm(y)
    z = t[:3, 2]-y*np.dot(t[:3, 2], y)
    if np.linalg.norm(z) < .2:
        raise ValueError('Cannot preserve approach direction while leveling jaws.')
    z /= np.linalg.norm(z)
    x = np.cross(y, z)
    result = np.eye(4)
    result[:3, :3] = np.column_stack((x, y, z))
    result[:3, 3] = tcp-result[:3, :3]@[0., 0., tcp_z]
    return matrix_pose(result)


def contact_target(wrist, bottle, body_height, tcp_z=.1358, pad_length=.076):
    """Place the pad midpoint at the cylindrical body midpoint, not the tip.

    Preserve the actual hover orientation. The 38mm tip-to-pad-center offset
    must rotate with the tilted gripper; a blind 100mm descent is incorrect.
    """
    grasp = pose_matrix(wrist)
    body_center = (pose_matrix(bottle) @ [0., 0., body_height/2, 1.])[:3]
    grasp[:3, 3] = body_center - grasp[:3, :3] @ [0., 0., tcp_z-pad_length/2]
    tcp = grasp[:3, 3] + grasp[:3, :3] @ [0., 0., tcp_z]
    if tcp[2] - bottle.position.z < .02:
        raise ValueError('Predicted fingertip too close to table; no contact move allowed.')
    return matrix_pose(grasp)


def lift_verified(rise, drift, bilateral, distance=.01):
    return bool(math.isfinite(rise) and math.isfinite(drift) and
                .6*distance <= rise <= 1.5*distance and
                drift <= min(.005, distance/2+.0005) and bilateral)


class VerifyBottle(PickCube):
    def __init__(self):
        super().__init__()
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.cube_name = self.declare_parameter('bottle_name', 'vision_bottle').value
        self.execute_requested = self.declare_parameter('execute', False).value
        self.phase = self.declare_parameter('phase', 'all').value
        if self.phase not in ('all', 'lift', 'preload', 'preloaded_lift', 'restore'):
            raise ValueError('Unsupported verification phase.')
        self.preload_mm = float(self.declare_parameter('preload_mm', 1.).value)
        self.contact_preload_mm = float(self.declare_parameter('contact_preload_mm', 0.).value)
        if not math.isfinite(self.contact_preload_mm) or not 0 <= self.contact_preload_mm <= .2:
            raise ValueError('contact_preload_mm must be within 0..0.2 mm total jaw closure.')
        self.model_dimensions = dimensions()
        # Upper straight body gives the gripper base clearance from the shoulder.
        self.grasp_height = float(self.declare_parameter('grasp_height', self.model_dimensions['body_height']-.001).value)
        self.result_path = Path(self.declare_parameter('report_file', str(
            Path.home()/'GraspGen/calibration/sim_bottle_grasp_validation.json')).value)
        self.monitor = None
        self.monitor_log = None
        self.contact_state = None
        self.contact_sub = self.create_subscription(String, '/bottle_grasp/contacts', self.bottle_contacts_callback, 10)
        self.report = {'simulation': True, 'physics_attachment_used': False, 'model_dimensions_m': self.model_dimensions, 'stages': [], 'success': False}
        self.original_acm = None
        self.attached = False

    def bottle_contacts_callback(self, msg):
        self.contact_state = (time.monotonic(), json.loads(msg.data))

    def bilateral(self):
        return bool(self.contact_state and time.monotonic()-self.contact_state[0] < .2 and
                    self.contact_state[1].get('fresh') and self.contact_state[1].get('bilateral'))

    def record(self, stage, **details):
        self.report['stages'].append({'stage': stage, **details})
        self.result_path.parent.mkdir(parents=True, exist_ok=True)
        self.result_path.write_text(json.dumps(self.report, indent=2)+'\n')
        self.get_logger().info(stage + ': ' + json.dumps(details))

    def snapshot(self):
        wrist = self.entity_pose('piper::link6')
        bottle = self.entity_pose(f'{self.cube_name}::bottle_link')
        tcp = (pose_matrix(wrist) @ [0., 0., .1358, 1.])[:3]
        return wrist, bottle, tcp

    def cartesian(self, poses, distance, state=None):
        req = GetCartesianPath.Request()
        req.header.frame_id = 'world'
        req.group_name = 'arm'
        req.link_name = 'link6'
        req.start_state = state or self.current_state()
        req.waypoints = poses
        req.max_step = .001
        req.jump_threshold = 0.
        req.revolute_jump_threshold = .05
        req.prismatic_jump_threshold = .005
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = .05
        req.max_acceleration_scaling_factor = .05
        response = self.service(self.cartesian_client, req)
        self.record('cartesian_plan', fraction=response.fraction, error=response.error_code.val)
        if response.error_code.val != 1 or response.fraction < .999:
            raise RuntimeError('Incomplete Cartesian path; no motion from this segment was sent.')
        measured = dict(zip(req.start_state.joint_state.name, req.start_state.joint_state.position))
        retime(response.solution.joint_trajectory, measured, min_duration=distance/.008, joint_speed=.04)
        if not response.solution.joint_trajectory.points:
            raise RuntimeError('Empty Cartesian path.')
        return response.solution

    def execute_arm(self, trajectory, expected):
        result = self.action(self.execute, ExecuteTrajectory.Goal(trajectory=trajectory), timeout=150.)
        if result.error_code.val != 1:
            raise RuntimeError(f'Arm controller failed ({result.error_code.val}).')
        self.settle(.3)
        actual = pose_matrix(self.entity_pose('piper::link6'))
        target = pose_matrix(expected)
        error = np.linalg.norm(actual[:3, 3]-target[:3, 3])
        angle = Rotation.from_matrix(target[:3, :3].T@actual[:3, :3]).magnitude()
        if error > .005 or angle > math.radians(3):
            raise RuntimeError(f'Wrist endpoint mismatch: {error:.4f}m, {math.degrees(angle):.2f}deg.')

    def change_table_contact(self, allow):
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        matrix = self.service(self.get_scene, req).scene.allowed_collision_matrix
        for name in [self.cube_name, 'pick_table']:
            if name not in matrix.entry_names:
                matrix.entry_names.append(name)
                for row in matrix.entry_values:
                    row.enabled.append(False)
                matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
        a, b = matrix.entry_names.index(self.cube_name), matrix.entry_names.index('pick_table')
        matrix.entry_values[a].enabled[b] = allow
        matrix.entry_values[b].enabled[a] = allow
        change = ApplyPlanningScene.Request()
        change.scene.is_diff = True; change.scene.robot_state.is_diff = True
        change.scene.allowed_collision_matrix = matrix
        if not self.service(self.apply_scene, change).success:
            raise RuntimeError('Could not update initial bottle/table support contact.')

    def attach_planning_geometry(self, wrist, bottle):
        # Planning-scene attachment only: never create a Gazebo joint or weld.
        obj = bottle_collision(self.cube_name, bottle)
        transform = np.linalg.inv(pose_matrix(wrist))
        obj.primitive_poses = [matrix_pose(transform@pose_matrix(p)) for p in obj.primitive_poses]
        obj.header.frame_id = 'link6'
        attached = AttachedCollisionObject()
        attached.link_name = 'link6'
        attached.touch_links = ['link7', 'link8']
        attached.object = obj
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True; req.scene.robot_state.is_diff = True
        req.scene.robot_state.attached_collision_objects = [attached]
        # MoveIt automatically removes the matching world object on attach.
        # Removing it a second time in the same diff makes the service fail.
        self.attached = True
        if not self.service(self.apply_scene, req).success:
            raise RuntimeError('Planning attachment failed.')
        self.attached = True

    def run(self):
        # The parent already has a `cartesian` service client. Keep it under a
        # different name so this class's Cartesian planning method is callable.
        self.current_state()
        wrist, bottle, tcp = self.snapshot()
        if self.phase == 'restore':
            self.restore_scene(wrist, bottle)
            return
        if self.phase == 'preloaded_lift':
            previous = json.loads(self.result_path.read_text())
            self.report = previous
            if previous.get('model_dimensions_m') != self.model_dimensions:
                raise RuntimeError('Bottle dimensions changed; previous grasp cannot be resumed.')
            stages = [stage['stage'] for stage in previous.get('stages', [])]
            if previous.get('success') or 'preload_execute' not in stages:
                raise RuntimeError('No failed preloaded hold available to resume.')
            preload_index = max(i for i, name in enumerate(stages) if name == 'preload_execute')
            if 'lift_execute' in stages[preload_index+1:]:
                raise RuntimeError('A lift was already executed after preload; do not repeat blindly.')
            if not self.execute_requested:
                self.record('inspection_only', message='Preloaded lift requires execute:=true.')
                return
            self.start_contact_monitor(); self.settle(.3)
            if not self.bilateral():
                raise RuntimeError('Bilateral contacts missing; no lift requested.')
            req = GetPlanningScene.Request()
            req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            self.original_acm = self.service(self.get_scene, req).scene.allowed_collision_matrix
            self.finger_contact(True)
            self.record('resume_preloaded_lift', message='No further closure or descent.')
            self.two_step_lift(bottle)
            return
        if self.phase == 'preload':
            self.preload_trial(wrist, bottle)
            return
        if self.phase == 'lift':
            if not self.execute_requested:
                self.record('inspection_only', message='Lift resume requires execute:=true; no motion requested.')
                return
            previous = json.loads(self.result_path.read_text())
            if previous.get('model_dimensions_m') != self.model_dimensions:
                raise RuntimeError('Bottle dimensions changed; previous grasp cannot be resumed.')
            stages = [stage['stage'] for stage in previous.get('stages', [])]
            if previous.get('success') or 'bilateral_contact_verified' not in stages or 'lift_execute' in stages:
                raise RuntimeError('Lift resume requires a prior contact-confirmed run with no previous lift execution.')
            self.report = previous
            self.start_contact_monitor()
            self.settle(.3)
            if not self.bilateral():
                raise RuntimeError('Fresh bilateral contacts missing; lift resume refused.')
            expected = contact_target(wrist, bottle, self.grasp_height*2)
            if np.linalg.norm(pose_matrix(expected)[:3, 3]-pose_matrix(wrist)[:3, 3]) > .01:
                raise RuntimeError('Robot no longer at the contact pose; lift resume refused.')
            req = GetPlanningScene.Request()
            req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            self.original_acm = self.service(self.get_scene, req).scene.allowed_collision_matrix
            self.finger_contact(True)
            self.record('resume_lift', message='No descent, reopening or additional close requested.')
            self.lift_from_grasp(bottle)
            return
        path = Path(get_package_share_directory('piper_gazebo'))/'worlds/bottle.sdf'
        root = ET.parse(path).getroot()
        geometry = root.find("model/link/collision[@name='body']/geometry/cylinder")
        radius = float(geometry.findtext('radius'))
        height = float(geometry.findtext('length'))
        if not 0 < radius*2 < .065 or height <= 0:
            raise RuntimeError('Bottle body must fit the gripper with clearance.')
        if np.dot(pose_matrix(bottle)[:3, 2], [0, 0, 1]) < .99:
            raise RuntimeError('Bottle is not upright.')
        xy = np.linalg.norm(tcp[:2]-[bottle.position.x, bottle.position.y])
        clearance = tcp[2]-bottle.position.z-self.model_dimensions['height']
        self.record('hover_preflight', tcp_world=tcp.tolist(), xy_error_m=float(xy), clearance_above_cap_m=float(clearance))
        if xy > .06 or not .015 <= clearance <= .15:
            raise RuntimeError('Robot is not at the verified hover. Run the fresh pregrasp planner with execute:=true first.')
        level = level_opening_direction(wrist)
        rotation = pose_matrix(wrist)[:3, :3]
        adjustment = Rotation.from_matrix(rotation.T@pose_matrix(level)[:3, :3]).magnitude()
        self.record('jaw_level_target', jaw_axis_vertical_component=float(rotation[2, 1]),
                    adjustment_deg=math.degrees(adjustment), tcp_position_preserved=True)
        if self.execute_requested and adjustment > math.radians(.1):
            tcp_before_level = tcp.copy()
            distance = float(np.linalg.norm(pose_matrix(level)[:3, 3]-pose_matrix(wrist)[:3, 3]))
            path = self.cartesian([level], max(.005, distance))
            self.record('jaw_level_execute')
            self.execute_arm(path, level)
            wrist, bottle, tcp = self.snapshot()
            axis_error = math.degrees(math.asin(min(1., abs(pose_matrix(wrist)[2, 1]))))
            tcp_drift = float(np.linalg.norm(tcp-tcp_before_level))
            if axis_error > 1. or tcp_drift > .003:
                raise RuntimeError('Jaw opening direction did not reach table-parallel alignment.')
            self.report['jaw_leveling_success'] = True
            self.record('jaw_level_verified', jaw_axis_vertical_component=float(pose_matrix(wrist)[2, 1]),
                        axis_error_deg=axis_error, tcp_translation_drift_m=tcp_drift)
        elif self.execute_requested:
            self.report['jaw_leveling_success'] = True
            self.record('jaw_level_verified', adjustment_skipped=True,
                        axis_error_deg=math.degrees(math.asin(min(1., abs(rotation[2, 1])))))
        elif not self.execute_requested:
            wrist = level  # Inspect the subsequent geometry using the proposed orientation.
        if not math.isfinite(self.grasp_height) or not .04 <= self.grasp_height <= self.model_dimensions['height']-.02:
            raise RuntimeError('grasp_height must be >=.04m and at least .02m below the bottle top.')
        change = ApplyPlanningScene.Request()
        change.scene.is_diff = True; change.scene.robot_state.is_diff = True
        change.scene.world.collision_objects = [bottle_collision(self.cube_name, bottle)]
        if not self.service(self.apply_scene, change).success:
            raise RuntimeError('Exact bottle collision geometry update failed.')
        target = contact_target(wrist, bottle, self.grasp_height*2)
        align = copy.deepcopy(wrist)
        align.position.x = target.position.x; align.position.y = target.position.y
        descent = wrist.position.z-target.position.z
        if not .02 < descent < .22:
            raise RuntimeError(f'Unexpected descent distance {descent:.3f}m.')
        self.record('contact_target', descent_m=descent,
                    flange_world=[target.position.x, target.position.y, target.position.z], bottle_diameter_m=radius*2,
                    contact_height_above_bottle_bottom_m=self.grasp_height)
        if not self.execute_requested:
            self.record('inspection_only', message='Use execute:=true to open, descend, close and test-lift. No motion requested.')
            return
        self.start_contact_monitor()
        self.record('open_gripper')
        self.gripper(.035)
        self.settle(.3)
        if self.current_state().joint_state.position[6] < .033:
            raise RuntimeError('Gripper did not open sufficiently.')
        distance = math.hypot(align.position.x-wrist.position.x, align.position.y-wrist.position.y)+descent
        path = self.cartesian([align, target], distance)
        self.record('descend_execute')
        self.execute_arm(path, target)
        self.record('descent_verified')
        after = self.entity_pose(f'{self.cube_name}::bottle_link')
        if np.linalg.norm(pose_matrix(after)[:3, 3]-pose_matrix(bottle)[:3, 3]) > .005:
            raise RuntimeError('Bottle moved during approach; stop before closing.')
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        self.original_acm = self.service(self.get_scene, req).scene.allowed_collision_matrix
        self.finger_contact(True)
        width = radius*2+.002
        confirmed = False
        while width >= .024:
            self.record('close_probe', commanded_total_opening_m=width)
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ['joint7']
            goal.trajectory.points = [JointTrajectoryPoint(positions=[width/2], time_from_start=Duration(sec=1))]
            result = self.action(self.grip, goal, timeout=10.)
            if result.error_code != 0:
                raise RuntimeError(f'Closing controller failed: {result.error_string}')
            deadline = time.monotonic()+.4
            since = None
            while time.monotonic() < deadline:
                self.settle(.025)
                since = (since or time.monotonic()) if self.bilateral() else None
                if since and time.monotonic()-since >= .15:
                    confirmed = True
                    break
            check_bottle = self.entity_pose(f'{self.cube_name}::bottle_link')
            if np.linalg.norm(pose_matrix(check_bottle)[:3, 3]-pose_matrix(bottle)[:3, 3]) > .01:
                raise RuntimeError('Bottle displaced more than 10mm during closing; no lift requested.')
            if confirmed:
                break
            width -= .001
        if not confirmed:
            raise RuntimeError('No stable bilateral bottle/finger contacts; no lift requested.')
        self.record('bilateral_contact_verified', contacts=self.contact_state[1], commanded_total_opening_m=width)
        if self.contact_preload_mm:
            self.small_contact_preload(bottle, width)
            self.two_step_lift(self.snapshot()[1])
        else:
            self.lift_from_grasp(bottle)

    def restore_scene(self, wrist, bottle):
        # Use only the saved hover POSE as a reference, never replay a saved path.
        saved = json.loads((Path.home()/'GraspGen/calibration/sim_pregrasp_plan.json').read_text())
        data = saved['target_link6']['pose']
        hover = Pose()
        for axis in ['x', 'y', 'z']:
            setattr(hover.position, axis, float(data['position'][axis]))
        for axis in ['x', 'y', 'z', 'w']:
            setattr(hover.orientation, axis, float(data['orientation'][axis]))
        hover = matrix_pose(pose_matrix(self.entity_pose('piper::base_link'))@pose_matrix(hover))
        hover = level_opening_direction(hover)
        tcp = (pose_matrix(hover)@[0., 0., .1358, 1.])[:3]
        if np.linalg.norm(tcp[:2]-pose_matrix(bottle)[:2, 3]) > .07 or hover.position.z < wrist.position.z:
            raise RuntimeError('Saved hover is incompatible with current scene; no recovery requested.')
        self.record('restore_inspection', hover_tcp_world=tcp.tolist())
        if not self.execute_requested:
            return
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        self.original_acm = self.service(self.get_scene, req).scene.allowed_collision_matrix
        self.finger_contact(True)
        self.gripper(.035)
        self.settle(1.)
        wrist, bottle, _ = self.snapshot()
        change = ApplyPlanningScene.Request()
        change.scene.is_diff = True; change.scene.robot_state.is_diff = True
        change.scene.world.collision_objects = [bottle_collision(self.cube_name, bottle)]
        if not self.service(self.apply_scene, change).success:
            raise RuntimeError('Could not synchronize released bottle before retraction.')
        up = copy.deepcopy(wrist); up.position.z = hover.position.z
        distance = up.position.z-wrist.position.z+float(np.linalg.norm(pose_matrix(hover)[:2, 3]-pose_matrix(wrist)[:2, 3]))
        trajectory = self.cartesian([up, hover], distance)
        self.execute_arm(trajectory, hover)
        # Restore the upright test fixture only after the robot has retracted.
        bottle = self.entity_pose(f'{self.cube_name}::bottle_link')
        before = pose_matrix(bottle)
        table = pose_matrix(self.entity_pose('pick_table::link'))
        reset = SetEntityState.Request()
        reset.state.name = self.cube_name; reset.state.reference_frame = 'world'
        reset.state.pose = copy.deepcopy(bottle)
        reset.state.pose.position.z = float(table[2, 3]+1.017)
        yaw = Rotation.from_matrix(before[:3, :3]).as_euler('xyz')[2]
        q = reset.state.pose.orientation
        q.x, q.y, q.z, q.w = map(float, Rotation.from_euler('z', yaw).as_quat())
        client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        try:
            if not self.service(client, reset).success:
                raise RuntimeError('Upright fixture reset failed.')
        finally:
            self.destroy_client(client)
        self.settle(1.)
        wrist, bottle, tcp = self.snapshot()
        if pose_matrix(bottle)[2, 2] < .999:
            raise RuntimeError('Bottle did not settle upright.')
        change.scene.world.collision_objects = [bottle_collision(self.cube_name, bottle)]
        if not self.service(self.apply_scene, change).success:
            raise RuntimeError('Restored bottle collision update failed.')
        self.report['success'] = True
        self.record('SCENE_RESTORED', tcp_world=tcp.tolist(), bottle_before_reset=before.tolist(),
                    bottle_after_reset=pose_matrix(bottle).tolist(), jaws_open=True)

    def small_contact_preload(self, bottle, commanded_width):
        state = self.current_state().joint_state
        width = 2*dict(zip(state.name, state.position))['joint7']
        # Contact prevents actual joints reaching the commanded closure. Use
        # the last command so a preload cannot accidentally relax the grip.
        target_width = min(width, commanded_width)-self.contact_preload_mm*.001
        self.record('small_preload_execute', initial_opening_m=width, target_opening_m=target_width,
                    extra_total_closure_mm=self.contact_preload_mm)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint7']
        goal.trajectory.points = [JointTrajectoryPoint(positions=[target_width/2], time_from_start=Duration(sec=2))]
        result = self.action(self.grip, goal, timeout=10.)
        if result.error_code != 0:
            raise RuntimeError('Small preload controller failed.')
        self.settle(.4)
        actual = self.entity_pose(f'{self.cube_name}::bottle_link')
        displacement = float(np.linalg.norm(pose_matrix(actual)[:3, 3]-pose_matrix(bottle)[:3, 3]))
        angle = Rotation.from_matrix(pose_matrix(bottle)[:3, :3].T@pose_matrix(actual)[:3, :3]).magnitude()
        self.record('small_preload_feedback', bottle_displacement_m=displacement,
                    bottle_rotation_deg=math.degrees(angle), bilateral=self.bilateral())
        if displacement > .003 or angle > math.radians(5) or not self.bilateral():
            raise RuntimeError('Small preload moved the bottle or lost bilateral contact; no lift requested.')

    def start_contact_monitor(self):
        # Keep a telemetry-only Gazebo contact subscriber alive throughout the test.
        monitor_path = Path(get_package_prefix('piper_gazebo'))/'lib/piper_gazebo/bottle_contact_monitor'
        self.monitor_log = open('/tmp/graspgen_bottle_contacts.log', 'w')
        self.monitor = subprocess.Popen([str(monitor_path), '--ros-args', '-p', f'bottle_name:={self.cube_name}'],
            stdout=self.monitor_log, stderr=subprocess.STDOUT)
        discovery_deadline = time.monotonic()+10.
        while time.monotonic() < discovery_deadline:
            self.settle(.1)
            if self.monitor.poll() is not None:
                raise RuntimeError('Contact monitor exited; inspect /tmp/graspgen_bottle_contacts.log.')
            if self.contact_state and self.contact_state[1].get('fresh'):
                break
        if not self.contact_state or not self.contact_state[1].get('fresh'):
            raise RuntimeError('Gazebo contact monitor unavailable; no contact motion requested.')

    def preload_trial(self, wrist, bottle):
        if not math.isfinite(self.preload_mm) or not 0 < self.preload_mm <= 1.:
            raise RuntimeError('preload_mm must be within (0, 1] mm of total additional jaw closure.')
        previous_text = self.result_path.read_text()
        previous = json.loads(previous_text)
        self.report = previous
        if previous.get('model_dimensions_m') != self.model_dimensions:
            raise RuntimeError('Bottle dimensions changed; previous grasp cannot be resumed.')
        stages = [stage['stage'] for stage in previous.get('stages', [])]
        if previous.get('success') or 'bilateral_contact_verified' not in stages:
            raise RuntimeError('Preload requires an existing contact-confirmed failed grasp, not a new target.')
        if 'preload_execute' in stages:
            raise RuntimeError('Preload already attempted; inspect its report before any further experiment.')
        joint_state = self.current_state().joint_state
        width = 2*dict(zip(joint_state.name, joint_state.position))['joint7']
        diameter = self.model_dimensions['radius']*2
        if not diameter*.5 <= width <= diameter+.005:
            raise RuntimeError(f'Unexpected current jaw opening {width:.4f}m; no preload allowed.')
        expected = contact_target(wrist, bottle, self.grasp_height*2)
        if np.linalg.norm(pose_matrix(expected)[:3, 3]-pose_matrix(wrist)[:3, 3]) > .016:
            raise RuntimeError('Robot moved away from the prior bottle-contact region.')
        if not self.execute_requested:
            self.record('preload_inspection_only', opening_m=width, extra_close_mm=self.preload_mm)
            return
        archive = self.result_path.with_name(self.result_path.stem+f'_before_preload_{time.time_ns()}.json')
        archive.write_text(previous_text)
        self.report = previous
        self.start_contact_monitor()
        self.settle(.3)
        if not self.bilateral():
            raise RuntimeError('Fresh bilateral contacts missing; no preload requested.')
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        self.original_acm = self.service(self.get_scene, req).scene.allowed_collision_matrix
        self.finger_contact(True)
        self.record('preload_execute', initial_opening_m=width, extra_close_mm=self.preload_mm, prior_report=str(archive))
        # Two slow half-steps avoid a sudden full preload at the shoulder.
        for fraction in [.5, 1.]:
            target = width-self.preload_mm*.001*fraction
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ['joint7']
            goal.trajectory.points = [JointTrajectoryPoint(positions=[target/2], time_from_start=Duration(sec=1))]
            result = self.action(self.grip, goal, timeout=10.)
            if result.error_code != 0:
                raise RuntimeError(f'Preload closing failed: {result.error_string}')
            self.settle(.3)
            current = self.entity_pose(f'{self.cube_name}::bottle_link')
            displacement = np.linalg.norm(pose_matrix(current)[:3, 3]-pose_matrix(bottle)[:3, 3])
            angle = Rotation.from_matrix(pose_matrix(bottle)[:3, :3].T@pose_matrix(current)[:3, :3]).magnitude()
            self.record('preload_feedback', opening_target_m=target, bottle_displacement_m=float(displacement), bottle_rotation_deg=math.degrees(angle), bilateral=self.bilateral())
            if displacement > .003 or angle > math.radians(5) or not self.bilateral():
                raise RuntimeError('Preload displaced/tilted the bottle or lost bilateral contact; no lift requested.')
        self.two_step_lift(bottle)

    def two_step_lift(self, bottle):
        original_relative = np.linalg.inv(pose_matrix(self.snapshot()[0]))@pose_matrix(bottle)
        self.lift_from_grasp(bottle, distance=.002, final=False)
        _, baseline, _ = self.snapshot()
        self.lift_from_grasp(baseline, distance=.008, final=False)
        final_wrist, final_bottle, _ = self.snapshot()
        total = final_bottle.position.z-bottle.position.z
        final_relative = np.linalg.inv(pose_matrix(final_wrist))@pose_matrix(final_bottle)
        drift = float(np.linalg.norm(final_relative[:3, 3]-original_relative[:3, 3]))
        if not lift_verified(total, drift, self.bilateral()):
            raise RuntimeError('Two-step lift total or cumulative slip did not pass retention checks.')
        self.report['success'] = True
        self.record('SUCCESS', total_bottle_rise_m=total, cumulative_relative_drift_m=drift,
                    message='Preload plus 2mm/8mm physical lift confirmed; jaws remain closed; no Gazebo weld.')

    def lift_from_grasp(self, bottle, distance=.01, final=True):
        wrist, held, _ = self.snapshot()
        relative = np.linalg.inv(pose_matrix(wrist))@pose_matrix(held)
        self.attach_planning_geometry(wrist, held)
        self.change_table_contact(True)  # Supported object may initially touch table; robot/table stays checked.
        raised = copy.deepcopy(wrist); raised.position.z += distance
        lift = self.cartesian([raised], distance)
        self.record('lift_execute', requested_rise_m=distance)
        self.execute_arm(lift, raised)
        self.settle(1.)
        final_wrist, final_bottle, _ = self.snapshot()
        rise = final_bottle.position.z-bottle.position.z
        drift = np.linalg.norm((np.linalg.inv(pose_matrix(final_wrist))@pose_matrix(final_bottle))[:3, 3]-relative[:3, 3])
        self.record('lift_measurement', actual_rise_m=rise, relative_translation_drift_m=float(drift), bilateral=self.bilateral())
        if not lift_verified(rise, float(drift), self.bilateral(), distance):
            raise RuntimeError('Physical lift/retention not verified; stop and inspect. No automatic retry or release.')
        self.change_table_contact(False)
        if final:
            self.report['success'] = True
            self.record('SUCCESS', message='Both contacts and 10mm physical lift confirmed; gripper remains closed. No Gazebo weld used.')

    def cleanup(self):
        if self.attached and not self.report['success']:
            # A failed physical hold must not leave a phantom held object in MoveIt.
            try:
                req = ApplyPlanningScene.Request()
                req.scene.is_diff = True; req.scene.robot_state.is_diff = True
                removal = AttachedCollisionObject()
                removal.link_name = 'link6'
                removal.object.id = self.cube_name
                removal.object.operation = CollisionObject.REMOVE
                req.scene.robot_state.attached_collision_objects = [removal]
                actual = self.entity_pose(f'{self.cube_name}::bottle_link')
                req.scene.world.collision_objects = [bottle_collision(self.cube_name, actual)]
                self.service(self.apply_scene, req)
            except RuntimeError:
                pass
        if self.original_acm is not None:
            req = ApplyPlanningScene.Request()
            req.scene.is_diff = True; req.scene.robot_state.is_diff = True
            req.scene.allowed_collision_matrix = self.original_acm
            try:
                self.service(self.apply_scene, req)
            except RuntimeError:
                pass
        if self.monitor:
            self.monitor.terminate()
            try:
                self.monitor.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                self.monitor.kill()
        if self.monitor_log:
            self.monitor_log.close()


def main():
    rclpy.init()
    node = VerifyBottle()
    # PickCube creates an instance attribute named cartesian; save its client
    # and remove the attribute so the subclass method is available.
    node.cartesian_client = node.__dict__.pop('cartesian')
    code = 0
    try:
        node.run()
    except (RuntimeError, ValueError, KeyboardInterrupt) as exc:
        node.record('STOPPED', reason=str(exc))
        code = 1
    finally:
        node.cleanup(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
