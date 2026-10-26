#!/usr/bin/env python3
"""Run after sourcing the build. Uses an isolated ROS domain; no robot driver."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import os
import signal
import subprocess
import tempfile
import time
import yaml

# Set before importing/initializing ROS, including in child processes.
os.environ['ROS_DOMAIN_ID'] = '173'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ.setdefault('ROS_LOG_DIR', '/tmp/graspgen-ik-test-logs')

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.srv import GetInteractiveMarkers


def main():
    rclpy.init()
    node = rclpy.create_node('ik_smoke_test')
    states, statuses, poses, transforms = [], [], [], []
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    node.create_subscription(JointState, '/ik/joint_states', states.append, 10)
    node.create_subscription(Bool, '/ik_status', statuses.append, 10)
    node.create_subscription(PoseStamped, '/target_pose', poses.append, 10)
    node.create_subscription(TFMessage, '/tf', transforms.append, 10)
    feedback_pub = node.create_publisher(InteractiveMarkerFeedback, '/interactive_marker/feedback', 10)
    pose_pub = node.create_publisher(PoseStamped, '/target_pose', 10)
    gripper_pub = node.create_publisher(Float64, '/gripper_cmd_topic', 10)
    hardware_commands = []
    node.create_subscription(JointState, '/ik/joint_commands', hardware_commands.append, 10)
    processes = []

    def wait_for(predicate, timeout=12):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError('Timed out waiting for ROS communication')

    with tempfile.TemporaryFile(mode='w+') as logs, tempfile.NamedTemporaryFile(mode='w+', suffix='.yaml') as params:
        try:
            model = (Path(get_package_share_directory('piper_description')) /
                     'urdf/piper_description.urdf').read_text()
            yaml.safe_dump({'/**': {'ros__parameters': {'robot_description': model}}}, params)
            params.flush()
            processes.append(subprocess.Popen(
                ['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                 '--ros-args', '--params-file', params.name, '-r', 'joint_states:=/ik/joint_states'],
                stdout=logs, stderr=subprocess.STDOUT, start_new_session=True))
            processes.append(subprocess.Popen(
                ['ros2', 'launch', 'piper_kinematics', 'piper_ik.launch.py'],
                stdout=logs, stderr=subprocess.STDOUT, start_new_session=True))
            wait_for(lambda: bool(states) and feedback_pub.get_subscription_count() > 0)
            assert all(math.isfinite(v) for v in states[-1].position)
            assert len(states[-1].position) == 7
            assert states[-1].position[6] == 0.0
            wait_for(lambda: all(tf_buffer.can_transform('base_link', link, Time())
                                 for link in ('link7', 'link8')))
            closed = tf_buffer.lookup_transform('gripper_base', 'link8', Time())
            client = node.create_client(GetInteractiveMarkers, '/interactive_marker/get_interactive_markers')
            assert client.wait_for_service(timeout_sec=10)
            future = client.call_async(GetInteractiveMarkers.Request())
            wait_for(future.done)
            marker = future.result().markers[0]
            assert marker.header.frame_id == 'base_link'
            assert len(marker.controls) == 6

            feedback = InteractiveMarkerFeedback()
            feedback.header.frame_id = 'base_link'
            feedback.client_id = 'smoke_test'
            feedback.marker_name = marker.name
            feedback.event_type = InteractiveMarkerFeedback.POSE_UPDATE
            # Independently compose the displayed URDF at a nonzero pose.
            root = ET.fromstring(model)
            matrix = np.eye(4)
            for i, angle in enumerate([0.1, 0.1, -0.1, 0.1, 0.1, -0.1], 1):
                joint = root.find(f"joint[@name='joint{i}']")
                origin = joint.find('origin')
                transform = np.eye(4)
                transform[:3, :3] = Rotation.from_euler('xyz', [float(v) for v in origin.get('rpy', '0 0 0').split()]).as_matrix()
                transform[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]
                motion = np.eye(4)
                motion[:3, :3] = Rotation.from_rotvec(angle * np.array([float(v) for v in joint.find('axis').get('xyz').split()])).as_matrix()
                matrix = matrix @ transform @ motion
            feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z = matrix[:3, 3]
            rotation = Rotation.from_matrix(matrix[:3, :3]).as_quat()
            (feedback.pose.orientation.x, feedback.pose.orientation.y,
             feedback.pose.orientation.z, feedback.pose.orientation.w) = rotation
            # The marker server rejects a new client during its first second.
            ready_at = time.monotonic() + 1.2
            wait_for(lambda: time.monotonic() >= ready_at)
            feedback_pub.publish(feedback)
            wait_for(lambda: bool(poses) and any(s.data for s in statuses))
            assert abs(poses[-1].pose.position.x - feedback.pose.position.x) < 1e-9
            def solution_matches_model():
                if not tf_buffer.can_transform('base_link', 'ik_solution', Time()):
                    return False
                actual = tf_buffer.lookup_transform('base_link', 'ik_solution', Time()).transform
                p = actual.translation
                r = actual.rotation
                position_error = np.linalg.norm(np.array([p.x, p.y, p.z]) - matrix[:3, 3])
                orientation_error = (Rotation.from_quat([r.x, r.y, r.z, r.w]).inv() * Rotation.from_matrix(matrix[:3, :3])).magnitude()
                return position_error <= 0.001 and orientation_error <= 0.01
            wait_for(solution_matches_model)
            wait_for(lambda: tf_buffer.can_transform('base_link', 'ik_current', Time()))
            # Once settled, the solver's current TCP must coincide with displayed link6.
            ready_at = time.monotonic() + 0.3
            wait_for(lambda: time.monotonic() >= ready_at)
            flange = tf_buffer.lookup_transform('base_link', 'link6', Time()).transform
            current = tf_buffer.lookup_transform('base_link', 'ik_current', Time()).transform
            assert np.linalg.norm(np.array([flange.translation.x-current.translation.x,
                flange.translation.y-current.translation.y, flange.translation.z-current.translation.z])) < 1e-7
            assert (Rotation.from_quat([flange.rotation.x, flange.rotation.y, flange.rotation.z, flange.rotation.w]).inv() *
                    Rotation.from_quat([current.rotation.x, current.rotation.y, current.rotation.z, current.rotation.w])).magnitude() < 1e-7

            wait_for(lambda: {'ik_target', 'ik_solution', marker.name}.issubset(
                {t.child_frame_id for msg in transforms for t in msg.transforms}))

            # Exercise all six marker controls away from the zero-pose limits.
            def urdf_fk(q):
                t = np.eye(4)
                for i, angle in enumerate(q, 1):
                    joint = root.find(f"joint[@name='joint{i}']")
                    origin = joint.find('origin')
                    frame = np.eye(4)
                    frame[:3, :3] = Rotation.from_euler('xyz', [float(v) for v in origin.get('rpy', '0 0 0').split()]).as_matrix()
                    frame[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]
                    motion = np.eye(4)
                    axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
                    motion[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
                    t = t @ frame @ motion
                return t

            def send_marker(t):
                feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z = t[:3, 3]
                q = Rotation.from_matrix(t[:3, :3]).as_quat()
                (feedback.pose.orientation.x, feedback.pose.orientation.y,
                 feedback.pose.orientation.z, feedback.pose.orientation.w) = q
                feedback.header.stamp = node.get_clock().now().to_msg()
                statuses.clear()
                poses.clear()
                feedback_pub.publish(feedback)
                wait_for(lambda: bool(statuses) and bool(poses))
                assert statuses[-1].data, 'Reachable marker target rejected'

            interior = np.array([0.2, 0.8, -0.7, 0.3, 0.4, -0.2])
            center = urdf_fk(interior)
            matrix = center.copy()
            send_marker(matrix)
            wait_for(solution_matches_model)
            worst_pos, worst_rot = 0.0, 0.0
            for axis in range(6):
                for direction in (-1, 1):
                    matrix = center.copy()
                    if axis < 3:
                        matrix[axis, 3] += direction * 0.005
                    else:
                        matrix[:3, :3] = Rotation.from_rotvec(np.eye(3)[axis-3] * direction * np.deg2rad(2)).as_matrix() @ center[:3, :3]
                    send_marker(matrix)
                    wait_for(solution_matches_model)
                    actual = tf_buffer.lookup_transform('base_link', 'ik_solution', Time()).transform
                    pos = np.array([actual.translation.x, actual.translation.y, actual.translation.z])
                    rot = actual.rotation
                    worst_pos = max(worst_pos, np.linalg.norm(pos-matrix[:3, 3]))
                    worst_rot = max(worst_rot, (Rotation.from_quat([rot.x,rot.y,rot.z,rot.w]).inv()*Rotation.from_matrix(matrix[:3, :3])).magnitude())
            print(f'PASS: 12 marker axis drags; maximum observed error {worst_pos*1000:.3f} mm, {np.rad2deg(worst_rot):.3f} deg', flush=True)
            for step in range(20):
                q = interior + 0.02 * np.sin(step * 0.1 + np.arange(6)*0.2)
                matrix = urdf_fk(q)
                send_marker(matrix)
            wait_for(solution_matches_model)
            print('PASS: 20 continuous marker targets', flush=True)

            gripper_pub.publish(Float64(data=0.02))
            wait_for(lambda: abs(states[-1].position[6] - 0.01) < 1e-9)
            def finger_moved():
                opened = tf_buffer.lookup_transform('gripper_base', 'link8', Time())
                a = closed.transform.translation
                b = opened.transform.translation
                return abs(math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2) - 0.01) < 1e-6
            wait_for(finger_moved)
            statuses.clear()
            invalid = PoseStamped()
            invalid.header.frame_id = 'base_link'
            invalid.pose.orientation.w = 1.0
            invalid.pose.position.x = 100.0
            pose_pub.publish(invalid)
            wait_for(lambda: any(not s.data for s in statuses))
            statuses.clear()
            invalid.pose.orientation.w = 0.0
            pose_pub.publish(invalid)
            wait_for(lambda: any(not s.data for s in statuses))
            assert not hardware_commands, 'Visualization must never send hardware commands'
            print('PASS: launch, marker service, feedback -> target -> IK, TF, both gripper links and mimic motion, unreachable/invalid target')
        except BaseException:
            logs.seek(0)
            print(logs.read())
            raise
        finally:
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
