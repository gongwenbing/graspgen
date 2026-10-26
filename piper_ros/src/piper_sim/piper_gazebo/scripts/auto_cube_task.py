#!/usr/bin/env python3
"""Wait for the simulation, then observe/pick/place each colour in order."""
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from pick_cube import PickCube
from stack_cubes import inverse_pose
from sync_pick_scene import compose_pose
from vision_cube_to_basket import BASKET_SLOTS

TARGETS = [('cube_red', 'pick_cube'), ('cube_blue', 'pick_cube_blue'), ('cube_green', 'pick_cube_green')]


class AutoCubeTask(PickCube):
    def __init__(self):
        super().__init__()
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.count = self.declare_parameter('count', 3).value
        self.assisted = self.declare_parameter('assisted_grasp', True).value
        self.height = float(self.declare_parameter('transport_height', .09).value)
        self.timeout = float(self.declare_parameter('startup_timeout', 180.).value)
        if not 1 <= self.count <= 3 or not .06 <= self.height <= .15 or not 10 <= self.timeout <= 600:
            raise ValueError('Invalid count/transport_height/startup_timeout')
        self.result_dir = Path(self.declare_parameter('result_dir',
            str(Path.home()/'GraspGen/calibration/auto_cube'/time.strftime('%Y%m%d_%H%M%S'))).value)
        self.result_dir.mkdir(parents=True, exist_ok=True)
        self.report = dict(success=False, physics_attachment_used=bool(self.assisted), completed=[], stages=[])
        self.pub = self.create_publisher(String, '/auto_cube/status', 10)
        self.image_received = 0.
        self.image_stamp = 0.
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image, qos_profile_sensor_data)
        self.child = None
        self.scripts = Path(get_package_prefix('piper_gazebo'))/'lib/piper_gazebo'

    def image(self, msg):
        self.image_received = time.monotonic()
        self.image_stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def record(self, stage, **details):
        message = dict(stage=stage, **details)
        self.report['stages'].append(message)
        (self.result_dir/'summary.json').write_text(json.dumps(self.report, indent=2))
        self.pub.publish(String(data=json.dumps(message)))
        self.get_logger().info(json.dumps(message))

    def wait_ready(self):
        self.record('waiting_for_simulation', timeout_sec=self.timeout,
                    physics_attachment_used=bool(self.assisted), reports=str(self.result_dir))
        end = time.monotonic()+self.timeout
        expected = {'pick_table', 'pick_basket', *[name for _, name in TARGETS[:self.count]]}
        while time.monotonic() < end:
            self.settle(.25)
            now = self.get_clock().now().nanoseconds*1e-9
            if (not self.joints or time.monotonic()-self.joint_wall_time > 1.
                    or time.monotonic()-self.image_received > 2.
                    or not 0 <= now-self.image_stamp < 2.):
                continue
            if not all(c.service_is_ready() for c in (self.entity, self.get_scene, self.ik, self.stabilizer)):
                continue
            if not all(c.server_is_ready() for c in (self.grip, self.execute, self.move)):
                continue
            req = GetPlanningScene.Request()
            req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            scene = self.service(self.get_scene, req).scene
            if expected <= {obj.id for obj in scene.world.collision_objects}:
                self.current_state()
                held = self.service(self.grasp_status, Trigger.Request())
                if held.success:
                    raise RuntimeError('A payload is already held; start with a fresh scene')
                self.record('simulation_ready')
                return
        raise RuntimeError('Startup timeout: check camera, controllers, MoveIt and scene synchronization')

    def stop_child(self):
        if self.child and self.child.poll() is None:
            os.killpg(self.child.pid, signal.SIGINT)
            try:
                self.child.wait(timeout=10.)
            except subprocess.TimeoutExpired:
                os.killpg(self.child.pid, signal.SIGTERM)
                self.child.wait(timeout=5.)

    def run_script(self, script, parameters):
        command = [sys.executable, str(self.scripts/script), '--ros-args']
        for key, value in parameters.items():
            value = str(value).lower() if isinstance(value, bool) else str(value)
            command += ['-p', f'{key}:={value}']
        self.child = subprocess.Popen(command, start_new_session=True)
        try:
            while self.child.poll() is None:
                self.settle(.1)
            if self.child.returncode:
                raise RuntimeError(f'{script} failed ({self.child.returncode}); no automatic retry/release')
        finally:
            self.stop_child()

    def run_auto(self):
        self.wait_ready()
        self.gripper(.035)
        for slot, (label, model) in enumerate(TARGETS[:self.count]):
            self.record('observe', target=label, slot=slot)
            self.run_script('observe_bottle.py', dict(target_geometry='cube', bottle_name=model, execute=True))
            self.record('pick_and_place', target=label, slot=slot)
            result = self.result_dir/f'{label}.json'
            self.run_script('vision_cube_to_basket.py', dict(
                target_class=label, max_objects=1, basket_slot=slot, execute=True,
                assisted_grasp=self.assisted, transport_height=self.height, report_file=result))
            report = json.loads(result.read_text())
            if not report.get('success'):
                raise RuntimeError(f'{label} did not pass placement verification')
            self.report['completed'].append(label)
            self.record('placed', target=label)
        basket = self.entity_pose('pick_basket')
        positions = {}
        for slot, (label, model) in enumerate(TARGETS[:self.count]):
            local = compose_pose(inverse_pose(basket), self.entity_pose(model)).position
            expected_x, expected_y = BASKET_SLOTS[slot]
            if (abs(local.x-expected_x) > .015 or abs(local.y-expected_y) > .015
                    or not .02 <= local.z <= .035):
                raise RuntimeError(f'Final basket verification failed: {label}')
            positions[label] = [local.x, local.y, local.z]
        self.record('final_basket_verified', positions=positions)
        self.report['success'] = True
        self.record('SUCCESS', count=len(self.report['completed']))


def main(args=None):
    rclpy.init(args=args)
    node = AutoCubeTask();code = 0
    try:
        node.run_auto()
    except (Exception, KeyboardInterrupt) as exc:
        if rclpy.ok():node.record('STOPPED', reason=str(exc))
        code = 1
    finally:
        node.stop_child();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
