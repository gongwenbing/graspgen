"""Isolated domain197 MoveIt scene roundtrip. No hardware/controllers/CAN."""
import os,subprocess,signal,json
from pathlib import Path
env=dict(os.environ,ROS_DOMAIN_ID='197',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR='/tmp/graspgen-table-roundtrip-logs')
log=open('/tmp/graspgen-table-roundtrip-movegroup.log','w')
p=subprocess.Popen(['ros2','launch','piper_with_gripper_moveit','move_group.launch.py'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
 result=subprocess.run(['ros2','run','piper_moveit_bridge','add_table'],env=env,capture_output=True,text=True,timeout=30)
 print('RETURN',result.returncode);print(result.stdout);print(result.stderr)
 Path('/home/zlq/GraspGen/doc/reports/table_moveit_roundtrip_20261105.txt').write_text('RETURN '+str(result.returncode)+'\n'+result.stdout+result.stderr)
 assert result.returncode==0, 'MoveIt table scene roundtrip failed'
 print('PASS: real MoveIt apply/readback for measured table and base-height guard; no hardware')
finally:
 if p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 log.close()
