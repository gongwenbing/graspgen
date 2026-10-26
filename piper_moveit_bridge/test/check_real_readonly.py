#!/usr/bin/env python3
"""Opt-in real CAN read-only test. Requires connected/up can0, never enables outputs."""
import os,signal,subprocess,time,json
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='190';os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_LOG_DIR']='/tmp/graspgen-moveit-logs'
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetMotionPlan
rclpy.init();n=rclpy.create_node('check_real_readonly');states=[];status=[];cmd=[]
n.create_subscription(JointState,'/joint_states_single',states.append,10)
n.create_subscription(String,'/piper_hardware/status',status.append,10)
n.create_subscription(JointState,'/moveit/joint_commands',cmd.append,10)
before=int(Path('/sys/class/net/can0/statistics/tx_packets').read_text())
log=open('/tmp/graspgen-real-readonly.log','w')
p=subprocess.Popen(['ros2','launch','piper_moveit_bridge','moveit_hardware.launch.py','read_only:=true','rviz:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
 start=time.monotonic()
 while time.monotonic()-start<8:rclpy.spin_once(n,timeout_sec=.05)
 assert len(states)>30 and status,'No live feedback'
 planner=n.create_client(GetMotionPlan,'/plan_kinematic_path')
 assert planner.wait_for_service(timeout_sec=10),'MoveIt planning service did not start'
 st=json.loads(status[-1].data);assert st['read_only'] and not st['outputs_enabled'] and not cmd
 after=int(Path('/sys/class/net/can0/statistics/tx_packets').read_text());assert after==before,(before,after)
 print('LIVE_STATUS',json.dumps(st,ensure_ascii=False),flush=True)
 print('LIVE_FEEDBACK_MESSAGES',len(states),'CAN_TX_DELTA',after-before,flush=True)
 print('PASS: live CAN -> ROS feedback -> MoveIt display, output gates disabled, no CAN transmissions',flush=True)
finally:
 if p.poll() is None:
  p.send_signal(signal.SIGINT)
  try:p.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 n.destroy_node();rclpy.shutdown();log.close()
