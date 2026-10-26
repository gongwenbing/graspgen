#!/usr/bin/env python3
"""Use RGB-D cube centres for picking, then place into a configured Gazebo basket."""
import copy
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener
from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from builtin_interfaces.msg import Duration
from stack_cubes import StackCubes, inverse_pose
from sync_pick_scene import compose_pose

NAMES = {'cube_red': 'pick_cube', 'cube_blue': 'pick_cube_blue', 'cube_green': 'pick_cube_green'}
BASKET_SLOTS = ((-.06, 0.), (0., 0.), (-.06, .06))


def placement_targets(basket, cube, relative, index, clearance):
    """Keep the carried body's orientation while moving to separate bin slots."""
    target = copy.deepcopy(cube)
    slot = Pose();slot.position.x, slot.position.y = BASKET_SLOTS[index]
    slot.position.z = .006+.02+.006
    target.position = compose_pose(basket, slot).position
    above = copy.deepcopy(target);above.position.z += clearance
    return above, compose_pose(above, inverse_pose(relative)), compose_pose(target, inverse_pose(relative))


class VisionCubeToBasket(StackCubes):
    def __init__(self):
        super().__init__()
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.execute_requested = self.declare_parameter('execute', False).value
        self.target_class = self.declare_parameter('target_class', '').value
        self.phase = self.declare_parameter('phase', 'pick_place').value
        if self.phase not in ('pick_place', 'place'):
            raise ValueError('phase must be pick_place or place')
        self.transport_height = float(self.declare_parameter('transport_height', .09).value)
        if not .06 <= self.transport_height <= .15:
            raise ValueError('transport_height must be 0.06..0.15m above release')
        self.max_objects = self.declare_parameter('max_objects', 1).value
        self.basket_slot = self.declare_parameter('basket_slot', 0).value
        if not 0 <= self.basket_slot <= 2 or self.basket_slot+self.max_objects > 3:
            raise ValueError('basket_slot plus max_objects must fit the three basket slots')
        if self.target_class and self.target_class not in NAMES:
            raise ValueError('target_class must be cube_red, cube_blue, cube_green or empty')
        if not 1 <= self.max_objects <= 3:
            raise ValueError('max_objects must be 1..3')
        # Physical friction is the default. Contact-triggered constraints must
        # be explicitly requested and are recorded as an assisted simulation.
        self.assisted = self.declare_parameter('assisted_grasp', False).value
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
        self.status = None
        self.status_sub = self.create_subscription(String, '/graspgen/status', self.status_callback, 10)
        self.output = Path(self.declare_parameter('report_file',
            str(Path.home()/'GraspGen/calibration/sim_vision_cube_to_basket.json')).value)
        self.report = dict(success=False, target_source='RGB-D color segmentation and known 0.04m cube geometry',
                           physics_attachment_used=bool(self.assisted), stages=[])

    def status_callback(self, message):
        self.status = (time.monotonic(), json.loads(message.data))

    def record(self, stage, **details):
        self.report['stages'].append(dict(stage=stage, **details))
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(self.report, indent=2))
        self.get_logger().info(f'{stage}: {json.dumps(details)}')

    def visual_target(self, excluded):
        deadline = time.monotonic()+15.
        last_reason = 'No fresh cube detection'
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            if not self.status or time.monotonic()-self.status[0] > 1.5:
                continue
            status = self.status[1]
            if not status.get('valid'):
                last_reason = status.get('reason', last_reason); continue
            stamp = float(status.get('stamp_sec', 0))
            if not 0 <= self.get_clock().now().nanoseconds*1e-9-stamp <= 1.5:
                continue
            for obj in status.get('objects', []):
                label = obj.get('class')
                if (label not in NAMES or label in excluded or not obj.get('valid')
                    or self.target_class and label != self.target_class):
                    continue
                if obj.get('position_semantics') != 'known_size_cube_center' or abs(obj.get('cube_size_m', 0)-.04) > 1e-6:
                    raise RuntimeError('Require color_cube backend with cube_size:=0.04')
                try:
                    tf = self.buffer.lookup_transform('world', status['frame'], Time(seconds=stamp))
                except Exception as exc:
                    last_reason = str(exc); continue
                frame = Pose();frame.position.x = tf.transform.translation.x
                frame.position.y = tf.transform.translation.y;frame.position.z = tf.transform.translation.z
                frame.orientation = tf.transform.rotation
                local = Pose();local.position.x, local.position.y, local.position.z = obj['center_m']
                local.orientation.x, local.orientation.y, local.orientation.z, local.orientation.w = obj['quaternion_xyzw']
                return label, compose_pose(frame, local)
        raise RuntimeError(last_reason)

    def check_ik(self, pose):
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm';request.ik_request.ik_link_name = 'link6'
        request.ik_request.pose_stamped.header.frame_id = 'world'
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.robot_state = self.current_state()
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(sec=3)
        response = self.service(self.ik, request)
        if response.error_code.val != 1:
            raise RuntimeError(f'Collision-aware IK failed: {response.error_code.val}')
        return dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))

    def place_cube(self, index, label, basket, after, relative):
        # Square cubes allow quarter-turn yaw changes without changing the
        # upright placement. Choose a feasible wrist branch at the same height.
        for yaw in (0., math.pi/2, -math.pi/2, math.pi):
            rotation = Pose();rotation.orientation = Quaternion(z=math.sin(yaw/2), w=math.cos(yaw/2))
            oriented = copy.deepcopy(after)
            oriented.orientation = compose_pose(rotation, after).orientation
            above_cube, transfer, release = placement_targets(basket, oriented, relative, index, self.transport_height)
            try:
                values = self.check_ik(transfer)
                # An endpoint alone may sit on a wrist limit. Check the entire
                # lowering path from that IK branch before committing to it.
                req = GetCartesianPath.Request()
                req.header.frame_id = 'world';req.group_name = 'arm';req.link_name = 'link6'
                req.start_state = self.current_state()
                req.start_state.joint_state.position = [values.get(name, position) for name, position in zip(
                    req.start_state.joint_state.name, req.start_state.joint_state.position)]
                req.waypoints = [release];req.max_step = .003;req.avoid_collisions = True
                req.revolute_jump_threshold = .2;req.prismatic_jump_threshold = .01
                req.max_velocity_scaling_factor = self.parameter('velocity_scale')
                req.max_acceleration_scaling_factor = self.parameter('velocity_scale')
                descent = self.service(self.cartesian, req)
                if descent.error_code.val != 1 or descent.fraction < .995:
                    continue
                break
            except RuntimeError:
                continue
        else:
            raise RuntimeError('No collision-free basket transfer orientation at requested height')
        self.record('transfer_target', cube_center_world_m=[getattr(above_cube.position, a) for a in 'xyz'], clearance_above_release_m=self.transport_height)
        self.move_arm(values)
        actual_transfer = self.cube_pose()
        error = math.dist([getattr(actual_transfer.position, a) for a in 'xyz'],
                          [getattr(above_cube.position, a) for a in 'xyz'])
        self.record('transfer_measurement', cube_center_world_m=[getattr(actual_transfer.position, a) for a in 'xyz'],
                    target_error_m=error)
        if error > .008:
            raise RuntimeError('Carried cube did not reach the transfer height/position')
        # Verify retention after transfer before lowering into the basket.
        carried = compose_pose(inverse_pose(self.entity_pose('piper::link6')), self.cube_pose())
        if math.dist([getattr(relative.position, a) for a in 'xyz'],
                 [getattr(carried.position, a) for a in 'xyz']) > .008:
            raise RuntimeError('Cube slipped during transfer')
        self.straight(release)
        # A full 70mm opening can hit a neighbouring cube at 60mm slot spacing.
        # Release the optional constraint explicitly; 54mm clears a 40mm cube.
        self.service(self.stabilizer, SetBool.Request(data=False))
        self.gripper(.027);self.settle(1.)
        self.detach_planning_cube()
        self.straight(transfer);self.finger_contact(False)
        self.gripper(.035)
        final = compose_pose(inverse_pose(basket), self.cube_pose()).position
        if not (abs(final.x) < .085 and abs(final.y) < .085 and .02 <= final.z <= .035):
            raise RuntimeError('Released cube did not settle inside basket')
        self.record('placed', label=label, basket_local_m=[final.x, final.y, final.z])

    def resume_place(self, basket):
        if not self.execute_requested or not self.assisted:
            raise RuntimeError('Resume requires execute:=true and assisted_grasp:=true')
        previous = json.loads(self.output.read_text())
        stages = previous.get('stages', [])
        if previous.get('success') or not previous.get('physics_attachment_used') or any(s['stage']=='placed' for s in stages):
            raise RuntimeError('No assisted held cube available to resume')
        if not any(s['stage']=='lift_verified' for s in stages):
            raise RuntimeError('Previous lift was not verified')
        label = next(s['label'] for s in reversed(stages) if s['stage']=='visual_target')
        self.select_cube(NAMES[label])
        status = self.service(self.grasp_status, Trigger.Request())
        if not status.success or status.message != self.cube_name:
            raise RuntimeError('The expected cube is no longer held')
        self.report = previous
        self.record('resume_placement')
        after = self.cube_pose()
        relative = compose_pose(inverse_pose(self.entity_pose('piper::link6')), after)
        self.pause_sync(True)
        self.place_cube(self.basket_slot, label, basket, after, relative)
        if 'initial_arm_positions' in self.report:
            self.move_arm(self.report['initial_arm_positions'])
        self.report['success'] = True
        self.record('SUCCESS', placed=1)

    def run_visual(self):
        initial = self.current_state().joint_state
        initial_positions = dict(zip(initial.name, initial.position))
        initial_positions['joint2'] = max(0., initial_positions['joint2'])
        initial_positions['joint3'] = min(0., initial_positions['joint3'])
        basket = self.entity_pose('pick_basket')
        if self.phase == 'place':
            self.resume_place(basket)
            return
        self.report['initial_arm_positions'] = initial_positions
        completed = set()
        for index in range(self.max_objects):
            label, visual = self.visual_target(completed)
            in_basket = compose_pose(inverse_pose(basket), visual).position
            if abs(in_basket.x) < .11 and abs(in_basket.y) < .11:
                raise RuntimeError('Detected cube is already inside the basket; select another colour')
            self.select_cube(NAMES[label])
            actual = self.cube_pose()  # Verification only; never used to construct the pick target.
            vision_error = math.dist([getattr(visual.position, a) for a in 'xyz'],
                                     [getattr(actual.position, a) for a in 'xyz'])
            if vision_error > .012:
                raise RuntimeError(f'Vision/Gazebo validation differs by {vision_error:.3f}m')
            grasp = copy.deepcopy(visual)
            grasp.position.z += self.parameter('grasp_offset')
            above = copy.deepcopy(grasp);above.position.z += self.parameter('approach_height')
            self.record('visual_target', label=label, center_world_m=[getattr(visual.position, a) for a in 'xyz'],
                        validation_error_m=vision_error, assisted_grasp=self.assisted)
            self.check_ik(above)
            if not self.execute_requested:
                self.record('inspection_complete', message='No motion requested; execute:=true to pick and place')
                return
            self.service(self.stabilizer, SetBool.Request(data=False))
            self.gripper(.035)
            self.approach(above)
            self.finger_contact(True)
            self.straight(grasp)
            self.contacts = None
            if self.assisted:
                self.service(self.stabilizer, SetBool.Request(data=True))
            self.gripper(.0193)
            self.settle(.3)
            contacts = ' '.join(s.collision1_name+' '+s.collision2_name for s in self.contacts.states) if self.contacts else ''
            if not all(f'::link{i}::' in contacts for i in (7, 8)):
                raise RuntimeError('Both finger contacts not detected; no lift')
            if self.assisted:
                status = self.service(self.grasp_status, Trigger.Request())
                if not status.success or status.message != self.cube_name:
                    raise RuntimeError('Contact stabilization not confirmed for selected cube')
            self.record('bilateral_contact', physics_attachment_used=bool(self.assisted))
            before = self.cube_pose()
            relative = compose_pose(inverse_pose(self.entity_pose('piper::link6')), before)
            raised = copy.deepcopy(grasp);raised.position.z += .05
            self.straight(raised);self.settle(.5)
            after = self.cube_pose()
            current_relative = compose_pose(inverse_pose(self.entity_pose('piper::link6')), after)
            drift = math.dist([getattr(relative.position, a) for a in 'xyz'],
                              [getattr(current_relative.position, a) for a in 'xyz'])
            rise = after.position.z-before.position.z
            self.record('lift_measurement', actual_rise_m=rise, relative_drift_m=drift)
            if not .03 <= rise <= .07 or drift > .008:
                raise RuntimeError('Cube retention failed; stop before transfer')
            self.record('lift_verified')
            relative = self.attach_planning_cube()
            self.place_cube(self.basket_slot+index, label, basket, after, relative)
            completed.add(label)
            self.move_arm(initial_positions)
            self.settle(.5)
        self.report['success'] = True
        self.record('SUCCESS', placed=len(completed))


def main(args=None):
    rclpy.init(args=args)
    node = VisionCubeToBasket();code = 0
    try:
        node.run_visual()
    except (RuntimeError, KeyboardInterrupt) as exc:
        node.record('STOPPED', reason=str(exc));code = 1
    finally:
        # Preserve a held payload and paused scene on failure; do not release it.
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
