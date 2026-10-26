#!/usr/bin/env python3
"""Action execution tests with fake topic hardware only. No CAN gateway."""
import os, signal, subprocess, tempfile, time
os.environ['ROS_DOMAIN_ID']='188';os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_LOG_DIR']='/tmp/graspgen-moveit-logs'
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance


def main():
    rclpy.init();node=rclpy.create_node('fake_bridge_hardware')
    arm=[f'joint{i}' for i in range(1,7)];q=[0.,.8,-.7,0.,.4,0.,.02];commands=[];visuals=[]
    sending=True;gate=True;follow=True
    feedback=node.create_publisher(JointState,'/joint_states_single',10)
    enabled=node.create_publisher(Bool,'/piper_hardware/outputs_enabled',10)
    def receive(msg):
        commands.append(msg)
        if follow:q[:]=msg.position
    node.create_subscription(JointState,'/moveit/joint_commands',receive,10)
    node.create_subscription(JointState,'/joint_states',visuals.append,10)
    def tick():
        enabled.publish(Bool(data=gate))
        if sending:
            msg=JointState();msg.header.stamp=node.get_clock().now().to_msg()
            msg.name=arm+['gripper'];msg.position=list(q);feedback.publish(msg)
    timer=node.create_timer(.01,tick)
    enable=node.create_client(SetBool,'/piper_trajectory_bridge/enable_execution')
    client=ActionClient(node,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory')
    grip=ActionClient(node,FollowJointTrajectory,'/gripper_controller/follow_joint_trajectory')
    def spin(t):
        end=time.monotonic()+t
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    def wait(fn,t=10):
        end=time.monotonic()+t
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if fn():return
        raise AssertionError('Timed out')
    def arm_gate(expected=True):
        f=enable.call_async(SetBool.Request(data=True));wait(f.done)
        assert f.result().success==expected,f.result().message
    def goal(delta=.02,duration=1.,gripper=False):
        g=FollowJointTrajectory.Goal();names=['joint7'] if gripper else arm
        values=[q[-1]/2] if gripper else list(q[:6]);end=values.copy();end[0]+=delta
        g.trajectory.joint_names=list(names)
        g.trajectory.points=[JointTrajectoryPoint(positions=values),JointTrajectoryPoint(positions=end)]
        g.trajectory.points[-1].time_from_start.sec=int(duration)
        g.trajectory.points[-1].time_from_start.nanosec=round((duration-int(duration))*1e9)
        g.goal_tolerance=[JointTolerance(name=n,position=.0002 if gripper else .001) for n in names]
        g.goal_time_tolerance.nanosec=300000000
        return g
    def send(g,a=client):
        f=a.send_goal_async(g);wait(f.done);return f.result()
    def result(h,t=10):
        f=h.get_result_async();wait(f.done,t);return f.result()
    with tempfile.TemporaryFile(mode='w+') as log:
        p=subprocess.Popen(['ros2','run','piper_moveit_bridge','trajectory_bridge'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            assert enable.wait_for_service(timeout_sec=15);assert client.wait_for_server(timeout_sec=10);assert grip.wait_for_server(timeout_sec=10)
            wait(lambda:bool(visuals));spin(.1)
            assert not commands
            assert not send(goal()).accepted
            gate=False;spin(.1);arm_gate(False);gate=True;spin(.1)
            arm_gate();h=send(goal());assert h.accepted
            assert result(h).result.error_code==0
            assert abs(q[0]-.02)<.002
            assert all(m.name[-1]=='gripper' for m in commands)
            for a,b in zip(commands,commands[1:]):
                dt=(b.header.stamp.sec-a.header.stamp.sec)+(b.header.stamp.nanosec-a.header.stamp.nanosec)*1e-9
                assert max(abs(x-y) for x,y in zip(a.position[:6],b.position[:6])) <= .2*dt+.002
            # Action cancellation and global serialization across arm/gripper.
            h=send(goal(.04,2));assert h.accepted;spin(.1)
            assert not send(goal(.001,1,True),grip).accepted
            f=h.cancel_goal_async();wait(f.done);assert result(h).status==5
            spin(.1);count=len(commands);spin(.1);assert len(commands)==count
            # Invalid names, timing, speed, NaN and out-of-limit points.
            arm_gate()
            for bad in ('names','speed','nan','limits','timing'):
                g=goal()
                if bad=='names':g.trajectory.joint_names[0]='joint2'
                if bad=='speed':g.trajectory.points[-1].time_from_start.nanosec=1000000;g.trajectory.points[-1].time_from_start.sec=0
                if bad=='nan':g.trajectory.points[-1].positions[0]=float('nan')
                if bad=='limits':g.trajectory.points[-1].positions[1]=-1.
                if bad=='timing':g.trajectory.points[-1].time_from_start.sec=0
                assert not send(g).accepted,bad
            # Width conversion: MoveIt joint7 is a half-opening, hardware is full opening.
            h=send(goal(.002,1,True),grip);assert h.accepted;assert result(h).result.error_code==0
            assert abs(q[-1]-.024)<.0005
            # Feedback timeout aborts and never automatically re-enables.
            h=send(goal(.04,2));assert h.accepted;spin(.1);sending=False
            assert result(h).status==6;spin(.1);count=len(commands);spin(.2);assert len(commands)==count
            sending=True;spin(.1);assert not send(goal()).accepted
            # Actual arrival is required: static fake hardware cannot succeed on elapsed time.
            arm_gate();follow=False;h=send(goal(.02,1));assert h.accepted;assert result(h).status==6
            follow=True;spin(.1)
            # Arm feedback is never clipped; an out-of-range measurement revokes readiness.
            q[1]=-.03;spin(.1);arm_gate(False)
            assert visuals[-1].position[1]==-.03
            print('PASS: FollowJointTrajectory success, cancel, serialization, invalid goals, width units, stale feedback, no auto-rearm, actual arrival, out-of-range feedback',flush=True)
        except BaseException:
            log.seek(0);print(log.read());raise
        finally:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:p.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
