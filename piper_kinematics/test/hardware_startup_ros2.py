#!/usr/bin/env python3
"""Fake feedback only. No CAN driver, no hardware enable service is invoked."""
import math
import os
import signal
import subprocess
import tempfile
import time
os.environ['ROS_DOMAIN_ID'] = '174'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ.setdefault('ROS_LOG_DIR', '/tmp/graspgen-ik-test-logs')
import rclpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from std_srvs.srv import SetBool


def main():
    rclpy.init()
    node = rclpy.create_node('fake_hardware_test')
    commands, visuals, poses = [], [], []
    node.create_subscription(JointState, '/ik/joint_commands', commands.append, 10)
    node.create_subscription(JointState, '/ik/joint_states', visuals.append, 10)
    node.create_subscription(PoseStamped, '/ik/current_pose', poses.append, 10)
    feedback = node.create_publisher(JointState, '/joint_states_single', 10)
    target = node.create_publisher(PoseStamped, '/target_pose', 10)
    gripper = node.create_publisher(Float64, '/gripper_cmd_topic', 10)
    enable = node.create_client(SetBool, '/piper_ik_node/enable_commands')
    sample = JointState()
    # Shuffled feedback verifies name-based mapping, rather than positional indexing.
    sample.name = ['gripper', 'joint4', 'joint1', 'joint6', 'joint2', 'joint5', 'joint3']
    sample.position = [0.02, 0.2, 0.1, -0.2, 0.7, 0.2, -0.6]
    sending, stale_stamp = False, False

    def send():
        if sending:
            sample.header.stamp = node.get_clock().now().to_msg()
            if stale_stamp:
                sample.header.stamp.sec -= 5
            feedback.publish(sample)
    timer = node.create_timer(0.02, send)

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)

    def wait(predicate, seconds=8):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError('Timed out')

    def arm(value=True):
        future = enable.call_async(SetBool.Request(data=value))
        wait(future.done)
        return future.result()

    with tempfile.TemporaryFile(mode='w+') as logs:
        process = subprocess.Popen(['ros2', 'launch', 'piper_kinematics', 'piper_ik.launch.py',
                                    'hardware_mode:=true', 'feedback_timeout:=0.3'],
                                   stdout=logs, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            assert enable.wait_for_service(timeout_sec=10)
            spin(0.4)
            assert not commands and not visuals
            assert not arm().success
            sending = True
            wait(lambda: bool(visuals) and bool(poses))
            assert list(visuals[-1].position[:6]) == [0.1, 0.7, -0.6, 0.2, 0.2, -0.2]
            assert visuals[-1].position[6] == 0.01
            target.publish(poses[-1])
            gripper.publish(Float64(data=0.03))
            spin(0.2)
            assert not commands, 'Feedback alone or targets before arm must not move the robot'
            assert arm().success
            spin(0.15)
            assert not commands, 'Arming alone must not output commands'
            # A rejected target, even after arming, must not become an active command.
            invalid = PoseStamped()
            invalid.header.frame_id = 'base_link'
            invalid.pose.orientation.w = 1.0
            invalid.pose.position.x = 100.0
            target.publish(invalid)
            spin(0.2)
            assert not commands
            goal = poses[-1]
            goal.pose.position.x += 0.002
            target.publish(goal)
            wait(lambda: len(commands) >= 5)
            assert commands[0].name[-1] == 'gripper'
            assert commands[0].position[6] == 0.02
            seed = [0.1, 0.7, -0.6, 0.2, 0.2, -0.2]
            assert max(abs(a-b) for a,b in zip(commands[0].position[:6], seed)) <= 0.021
            for a,b in zip(commands, commands[1:]):
                dt = (b.header.stamp.sec - a.header.stamp.sec) + (b.header.stamp.nanosec-a.header.stamp.nanosec)*1e-9
                assert max(abs(x-y) for x,y in zip(a.position[:6],b.position[:6])) <= 0.2 * dt + 0.002
            # Bad gripper commands cannot change the active opening.
            gripper.publish(Float64(data=float('nan')))
            gripper.publish(Float64(data=0.5))
            spin(0.15)
            assert commands[-1].position[6] == 0.02
            # Receive-time-only freshness is insufficient: reject old timestamps too.
            stale_stamp = True
            spin(0.15)
            count = len(commands)
            spin(0.15)
            assert len(commands) == count
            stale_stamp = False
            spin(0.15)
            count = len(commands)
            target.publish(goal)
            spin(0.15)
            assert len(commands) == count, 'Feedback recovery must not auto-rearm'
            assert arm().success
            gripper.publish(Float64(data=0.025))
            wait(lambda: len(commands) > count)
            assert commands[-1].position[6] >= 0.02
            # Timeout, then recovered feedback, also requires explicit rearm and a new target.
            sending = False
            spin(0.55)
            count = len(commands)
            spin(0.15)
            assert len(commands) == count
            sending = True
            spin(0.15)
            assert len(commands) == count
            assert arm().success
            spin(0.15)
            assert len(commands) == count
            target.publish(poses[-1])
            wait(lambda: len(commands) > count)
            assert arm(False).success
            spin(0.1)
            count = len(commands)
            target.publish(poses[-1])
            spin(0.15)
            assert len(commands) == count, 'Explicit disable must stop and discard targets'
            assert arm().success
            gripper.publish(Float64(data=0.025))
            wait(lambda: len(commands) > count)
            # Invalid/partial feedback revokes output immediately.
            sample.name = ['joint1']
            sample.position = [0.1]
            spin(0.15)
            count = len(commands)
            spin(0.15)
            assert len(commands) == count
            assert not arm().success
            assert arm(False).success
            assert all(math.isfinite(v) for msg in commands for v in msg.position)
            print('PASS: no startup output, named feedback, explicit arming, fresh target, bounded steps, gripper units, stale/invalid feedback stop, no automatic restart')
        except BaseException:
            logs.seek(0)
            print(logs.read())
            raise
        finally:
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
