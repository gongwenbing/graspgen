#!/usr/bin/env python3
"""Pick red, blue, green; stack them inside the basket and return to the start."""
import copy
import math
import time

import rclpy
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from pick_cube import PickCube
from shape_msgs.msg import SolidPrimitive
from stack_scene import CUBES
from std_srvs.srv import SetBool, Trigger
from sync_pick_scene import compose_pose


def inverse_pose(pose: Pose) -> Pose:
    rotation = Pose()
    q = pose.orientation
    rotation.orientation = Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)
    translation = Pose()
    translation.position.x = -pose.position.x
    translation.position.y = -pose.position.y
    translation.position.z = -pose.position.z
    return compose_pose(rotation, translation)


class StackCubes(PickCube):
    def __init__(self):
        super().__init__()
        self.pause_client = self.create_client(SetBool, '/pick_scene/pause')
        self.placed_names = []
        self.unexpected_contacts = set()
        self.placed_subscriptions = [self.create_subscription(
            ContactsState, f'/{name}/contacts',
            lambda msg, cube=name: self.observe_placed_contact(cube, msg), 10) for name in CUBES]

    def observe_placed_contact(self, name, message):
        if name not in self.placed_names:
            return
        for state in message.states:
            for collision in (state.collision1_name, state.collision2_name):
                if collision.startswith('piper::') and (name, collision) not in self.unexpected_contacts:
                    self.unexpected_contacts.add((name, collision))
                    self.get_logger().warn(f'Placed cube contact: {name} / {collision}')

    def pause_sync(self, pause: bool):
        deadline = time.monotonic() + 10.0
        while not self.service(self.pause_client, SetBool.Request(data=pause)).success:
            if time.monotonic() > deadline:
                raise RuntimeError('Scene synchronization did not pause cleanly.')
            self.settle(0.1)

    def select_cube(self, name: str):
        self.destroy_subscription(self.contact_subscription)
        self.cube_name = name
        self.contacts = None
        self.contact_subscription = self.create_subscription(
            ContactsState, f'/{name}/contacts', self.contact_callback, 10)

    def collision_cube(self, name: str, pose: Pose, frame: str) -> CollisionObject:
        obj = CollisionObject()
        obj.id = name
        obj.header.frame_id = frame
        obj.operation = CollisionObject.ADD
        obj.pose = pose
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.04, 0.04, 0.04])
        obj.primitives = [box]
        obj.primitive_poses = [Pose()]
        return obj

    def attach_planning_cube(self) -> Pose:
        self.pause_sync(True)
        wrist = self.entity_pose('piper::link6')
        relative = compose_pose(inverse_pose(wrist), self.cube_pose())
        self.get_logger().debug(f'Captured payload transform: {relative}')
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        attached = AttachedCollisionObject()
        attached.link_name = 'link6'
        attached.touch_links = ['link6', 'gripper_base', 'link7', 'link8']
        attached.object = self.collision_cube(self.cube_name, relative, 'link6')
        request.scene.robot_state.attached_collision_objects = [attached]
        if not self.service(self.apply_scene, request).success:
            raise RuntimeError('MoveIt could not attach the carried cube.')
        return relative

    def detach_planning_cube(self):
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        attached = AttachedCollisionObject()
        attached.link_name = 'link6'
        attached.object.id = self.cube_name
        attached.object.operation = CollisionObject.REMOVE
        request.scene.robot_state.attached_collision_objects = [attached]
        request.scene.world.collision_objects = [self.collision_cube(
            self.cube_name, self.cube_pose(), 'world')]
        if not self.service(self.apply_scene, request).success:
            raise RuntimeError('MoveIt could not detach the released cube.')
        self.pause_sync(False)

    def verify_stack(self, names, basket: Pose):
        previous = None
        poses = [(name, self.entity_pose(name)) for name in names]
        for name, pose in poses:
            self.get_logger().info(
                f'Stack check {name}: ({pose.position.x:.4f}, '
                f'{pose.position.y:.4f}, {pose.position.z:.4f})')
        for name, pose in poses:
            expected_z = basket.position.z + 0.006 + 0.02 + 0.04 * list(CUBES).index(name)
            if math.hypot(pose.position.x - basket.position.x,
                          pose.position.y - basket.position.y) > 0.008:
                raise RuntimeError(f'{name} is not centered in the basket: {pose.position}.')
            if abs(pose.position.z - expected_z) > 0.005:
                raise RuntimeError(f'{name} has wrong stack height: {pose.position.z:.4f}.')
            q = pose.orientation
            upright = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            if upright < math.cos(math.radians(5)):
                raise RuntimeError(f'{name} tilted excessively.')
            if previous is not None and math.hypot(
                pose.position.x - previous.position.x,
                pose.position.y - previous.position.y) > 0.006:
                raise RuntimeError('Cube centers do not overlap sufficiently.')
            previous = pose

    def run_stack(self):
        if not self.bool_parameter('stabilize_grasp'):
            raise RuntimeError('The stacking demo requires contact-triggered grasp stabilization.')
        initial = self.current_state().joint_state
        initial_positions = dict(zip(initial.name, initial.position))
        # Remove tiny physics drift outside the two one-sided joint limits.
        initial_positions['joint2'] = max(0.0, initial_positions['joint2'])
        initial_positions['joint3'] = min(0.0, initial_positions['joint3'])
        basket = self.entity_pose('pick_basket')
        placed = []
        self.placed_names = placed
        above: Pose | None = None
        for index, name in enumerate(CUBES):
            self.get_logger().info(f'STACK {index + 1}/3: {name}')
            self.select_cube(name)
            super().run()
            status = self.service(self.grasp_status, Trigger.Request())
            if not status.success or status.message != name:
                raise RuntimeError(f'Wrong object constrained: {status.message}')
            relative = self.attach_planning_cube()
            above_cube = Pose()
            above_cube.position = copy.deepcopy(basket.position)
            # 13 mm above the final green cube; stays within the wrist limits.
            above_cube.position.z += 0.006 + 0.10 + 0.013
            above = compose_pose(above_cube, inverse_pose(relative))
            self.get_logger().info('Transfer carried cube above basket')
            self.straight(above)
            target_cube = Pose()
            target_cube.position = copy.deepcopy(basket.position)
            if placed:
                support = self.entity_pose(placed[-1])
                target_cube.position = copy.deepcopy(support.position)
                target_cube.position.z += 0.046
            else:
                target_cube.position.z += 0.006 + 0.02 + 0.006
            target = compose_pose(target_cube, inverse_pose(relative))
            self.get_logger().info('Lower to stack and release')
            self.straight(target)
            self.settle(0.5)
            actual_cube = self.cube_pose()
            self.get_logger().debug(
                f'Release {name}: actual={actual_cube}, target={target_cube.position}')
            self.gripper(0.035)  # Opening releases the Gazebo constraint.
            self.settle(1.0)
            if self.service(self.grasp_status, Trigger.Request()).success:
                raise RuntimeError('Grasp constraint did not release.')
            self.service(self.stabilizer, SetBool.Request(data=False))
            self.detach_planning_cube()
            self.settle(0.7)
            self.straight(above)
            self.finger_contact(False)
            placed.append(name)
            self.settle(1.0)
            self.verify_stack(placed, basket)
            self.get_logger().info(f'PLACED: {name}')
        self.get_logger().info('Withdraw sideways clear of the top cube')
        if above is None:
            raise RuntimeError('No stack targets configured.')
        escape = copy.deepcopy(above)
        escape.position.x -= 0.08
        self.straight(escape)
        self.get_logger().info('Return arm to recorded initial joint positions')
        self.move_arm(initial_positions)
        self.settle(2.0)
        self.verify_stack(placed, basket)
        final_state = self.current_state().joint_state
        actual = dict(zip(final_state.name, final_state.position))
        if any(abs(actual[name] - initial_positions[name]) > 0.02 for name in self.arm_names):
            raise RuntimeError('Arm did not return to its initial position.')
        self.get_logger().info('SUCCESS: red/blue/green stacked in basket; arm returned to start.')


def main(args=None):
    rclpy.init(args=args)
    node = StackCubes()
    code = 0
    try:
        node.run_stack()
    except (RuntimeError, KeyboardInterrupt) as exc:
        node.get_logger().error(f'Stacking stopped: {exc}')
        code = 1
    finally:
        # A held object remains attached after an error. Opening releases it;
        # keeping the bridge paused prevents duplicate world/attached objects.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
