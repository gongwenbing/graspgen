#!/usr/bin/env python3
"""Synthetic feedback and fake planning service in domain199; never opens CAN."""
import os
os.environ['ROS_DOMAIN_ID']='199';os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_LOG_DIR']='/tmp/graspgen-pregrasp-test'
import time,json,subprocess,tempfile,threading
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.srv import GetMotionPlan
from piper_moveit_bridge.model import ARM
rclpy.init();node=Node('mock_pregrasp_planner');calls=[];displays=[];commands=[]
state=node.create_publisher(JointState,'/joint_states',10)
target=node.create_publisher(PoseStamped,'/grasp_preview/pregrasp_link6',10)
hardware=node.create_publisher(String,'/piper_hardware/status',10)
ready=node.create_publisher(String,'/piper_bridge/readiness',10)
preview=node.create_publisher(String,'/grasp_preview/status',10)
sub=node.create_subscription(DisplayTrajectory,'/display_planned_path',displays.append,10)
cmd=node.create_subscription(JointState,'/moveit/joint_commands',commands.append,10)
q=[0.,.8,-.7,0.,.4,0.,.01];candidate_valid=True
def publish():
 stamp=node.get_clock().now().to_msg()
 s=JointState();s.header.stamp=stamp;s.name=list(ARM)+['joint7'];s.position=q;state.publish(s)
 p=PoseStamped();p.header.stamp=stamp;p.header.frame_id='base_link';p.pose.position.x=.2;p.pose.position.z=.3;p.pose.orientation.x=1.;p.pose.orientation.w=0.
 if candidate_valid:target.publish(p)
 hardware.publish(String(data=json.dumps({'outputs_enabled':False,'reason':''})))
 ready.publish(String(data=json.dumps({'execution_enabled':False,'active':False})))
 preview.publish(String(data=json.dumps({'display_only':True,'target_valid':candidate_valid,'reason':'' if candidate_valid else 'No eligible grasp candidate'})))
def plan(req,res):
 c=req.motion_plan_request.goal_constraints[0];assert c.position_constraints[0].link_name=='link6'
 assert c.orientation_constraints[0].orientation.x==1.
 assert c.position_constraints[0].header.frame_id=='base_link'
 assert req.motion_plan_request.max_velocity_scaling_factor==.05
 calls.append(req)
 m=res.motion_plan_response;m.error_code.val=1;m.trajectory_start=req.motion_plan_request.start_state
 m.trajectory.joint_trajectory.joint_names=list(ARM)
 point=JointTrajectoryPoint();point.positions=q[:6];point.time_from_start.sec=1
 m.trajectory.joint_trajectory.points=[point]
 return res
service=node.create_service(GetMotionPlan,'/plan_kinematic_path',plan);timer=node.create_timer(.05,publish)
executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node);thread=threading.Thread(target=executor.spin);thread.start()
try:
 with tempfile.TemporaryDirectory() as tmp:
  out=Path(tmp)/'plan.json'
  result=subprocess.run(['ros2','run','piper_moveit_bridge','plan_pregrasp','--output',str(out)],capture_output=True,text=True,timeout=25)
  assert result.returncode==0,result.stdout+result.stderr
  assert calls and displays and not commands
  data=json.loads(out.read_text());assert data['display_only'] and data['target_link6']['header']['stamp']['sec']>0
  candidate_valid=False;old_calls=len(calls)
  result2=subprocess.run(['ros2','run','piper_moveit_bridge','plan_pregrasp','--wait-target','1','--output',str(out)],capture_output=True,text=True,timeout=10)
  assert result2.returncode!=0 and 'Pregrasp unavailable' in result2.stderr,result2.stdout+result2.stderr
  assert len(calls)==old_calls and not commands
  def recover():
   global candidate_valid
   candidate_valid=True
  recovery=threading.Timer(6.,recover);recovery.start()
  result3=subprocess.run(['ros2','run','piper_moveit_bridge','plan_pregrasp','--wait-target','6','--output',str(out)],capture_output=True,text=True,timeout=20)
  recovery.join()
  assert result3.returncode==0,result3.stdout+result3.stderr
  assert 'Waiting up to' in result3.stdout and len(calls)==old_calls+1 and not commands
  print(result.stdout.strip());print('PASS: pose constraints, preview file and DisplayTrajectory; zero joint commands, no execution client')
finally:
 executor.shutdown();thread.join(timeout=3);node.destroy_node();rclpy.shutdown()
