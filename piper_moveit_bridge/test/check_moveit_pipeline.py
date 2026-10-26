#!/usr/bin/env python3
"""MoveIt end-to-end check with mock control or a fake-topic bridge; never opens CAN."""
import os,signal,subprocess,tempfile,time
import argparse
parser=argparse.ArgumentParser();parser.add_argument('--bridge',action='store_true');args=parser.parse_args()
os.environ['ROS_DOMAIN_ID']='191' if args.bridge else '185';os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_LOG_DIR']='/tmp/graspgen-moveit-logs'
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints,JointConstraint
rclpy.init();n=rclpy.create_node('check_mock_moveit');states=[]
n.create_subscription(JointState,'/joint_states',states.append,10)
def wait(fn,t=25):
 end=time.monotonic()+t
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.05)
  if fn():return
 raise AssertionError('Timed out')
if args.bridge:
 raw=[0.]*6+[.02]
 feedback=n.create_publisher(JointState,'/joint_states_single',10)
 gate=n.create_publisher(Bool,'/piper_hardware/outputs_enabled',10)
 def receive(msg):
  raw[:]=msg.position
 n.create_subscription(JointState,'/moveit/joint_commands',receive,10)
 def tick():
  msg=JointState();msg.header.stamp=n.get_clock().now().to_msg();msg.name=[f'joint{i}' for i in range(1,7)]+['gripper'];msg.position=list(raw)
  feedback.publish(msg);gate.publish(Bool(data=True))
 timer=n.create_timer(.01,tick)
log=open('/tmp/graspgen-moveit-bridge-pipeline.log' if args.bridge else '/tmp/graspgen-moveit-mock.log','w+')
command=['ros2','launch','piper_moveit_bridge','moveit_hardware.launch.py','start_can_gateway:=false','rviz:=false'] if args.bridge else ['ros2','launch','piper_with_gripper_moveit','demo.launch.py','use_rviz:=false']
p=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
 wait(lambda: bool(states));print('JOINT_FEEDBACK_OK',flush=True)
 if args.bridge:
  service=n.create_client(SetBool,'/piper_trajectory_bridge/enable_execution');assert service.wait_for_service(timeout_sec=10)
  ready=time.monotonic()+0.5;wait(lambda: time.monotonic()>=ready)
  f=service.call_async(SetBool.Request(data=True));wait(f.done);assert f.result().success,f.result().message

 c=n.create_client(GetMotionPlan,'/plan_kinematic_path');assert c.wait_for_service(timeout_sec=25)
 if args.bridge:
  with tempfile.TemporaryFile(mode='w+') as helper_log:
   helper=subprocess.Popen(['ros2','run','piper_moveit_bridge','small_step','--joint','joint1','--delta','0.01'],stdout=helper_log,stderr=subprocess.STDOUT)
   wait(lambda:helper.poll() is not None,25)
   helper_log.seek(0);text=helper_log.read();print(text,flush=True)
   assert helper.returncode==0 and 'Plan only' in text
 request=GetMotionPlan.Request();m=request.motion_plan_request;m.group_name='arm';m.allowed_planning_time=5.;m.num_planning_attempts=3;m.max_velocity_scaling_factor=.05;m.max_acceleration_scaling_factor=.05;m.start_state.joint_state=states[-1]
 measured=dict(zip(states[-1].name,states[-1].position));goal=Constraints()
 for i in range(1,7):
  name=f'joint{i}';target=measured[name]+(.03 if i==1 else 0.)
  goal.joint_constraints.append(JointConstraint(joint_name=name,position=target,tolerance_above=.001,tolerance_below=.001,weight=1.))
 m.goal_constraints=[goal];f=c.call_async(request);wait(f.done,15);result=f.result().motion_plan_response
 print('PLAN_ERROR',result.error_code.val,'POINTS',len(result.trajectory.joint_trajectory.points),flush=True);assert result.error_code.val==1
 a=ActionClient(n,ExecuteTrajectory,'/execute_trajectory');assert a.wait_for_server(timeout_sec=10)
 if args.bridge:
  ready=time.monotonic()+0.2;wait(lambda: time.monotonic()>=ready)
  f=service.call_async(SetBool.Request(data=True));wait(f.done);assert f.result().success,f.result().message
 g=ExecuteTrajectory.Goal();g.trajectory=result.trajectory;f=a.send_goal_async(g);wait(f.done);handle=f.result();assert handle.accepted
 f=handle.get_result_async();wait(f.done,30);print('EXECUTE_ERROR',f.result().result.error_code.val,flush=True);assert f.result().result.error_code.val==1
 final=dict(zip(states[-1].name,states[-1].position));print('JOINT1_AFTER',final['joint1'],flush=True)
 assert abs(final['joint1']-(measured['joint1']+.03))<.003
 print('PASS: MoveIt plan -> execute -> controller/bridge -> joint feedback',flush=True)
finally:
 if p.poll() is None:
  p.send_signal(signal.SIGINT)
  try:p.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 n.destroy_node();rclpy.shutdown();log.close()
