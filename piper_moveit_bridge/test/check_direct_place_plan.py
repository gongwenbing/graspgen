#!/usr/bin/env python3
"""Actual MoveIt planning in private domain197, synthetic feedback/scene only.

No CAN gateway, motor controllers, execution goals or physical devices.
"""
import copy,json,os,re,signal,subprocess,threading,time
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='197';os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_LOG_DIR']='/tmp/graspgen-direct-place-test-logs'
import numpy as np,rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState,PointCloud2
from std_msgs.msg import String,Header
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene,AttachedCollisionObject,CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from piper_moveit_bridge.grasp_runtime import digest

ROOT=Path(__file__).resolve().parents[2]
seed=json.loads((ROOT/'calibration/ik_baseline_readonly_20261105.json').read_text())['seed']
config=json.loads((ROOT/'calibration/bottle_geometry.json').read_text())
rclpy.init();node=rclpy.create_node('synthetic_pick_place_feedback')
joints=node.create_publisher(JointState,'/joint_states',5)
hardware=node.create_publisher(String,'/piper_hardware/status',5);ready=node.create_publisher(String,'/piper_bridge/readiness',5)
vision=node.create_publisher(String,'/graspgen/status',2);cloud_pub=node.create_publisher(PointCloud2,'/graspgen/scene_cloud',2)
x,y=np.meshgrid(np.arange(.08,.48,.003),np.arange(-.58,-.18,.003))
points=np.column_stack([x.ravel(),y.ravel(),np.full(x.size,config['table_z_m'])])
cloud_template=point_cloud2.create_cloud_xyz32(Header(frame_id='base_link'),points.tolist())
center=[.13464630215496184,-.47047486188252224,config['table_z_m']+config['grasp_height_m']]
obj=dict(config,id=0,**{'class':'bottle'},valid=True,position_semantics='bottle_grasp_center',fit_rms_m=.001,arc_deg=120.,center_m=center,handeye_sha256=digest(ROOT/'calibration/handeye_result.yaml'))

def publish():
 stamp=node.get_clock().now().to_msg();js=JointState();js.header.stamp=stamp;js.name=list(seed);js.position=list(seed.values());joints.publish(js)
 h=dict(read_only=True,outputs_enabled=False,ctrl_mode=1,teach_status=0,err_code=0,motors_enabled=[True]*6,reason='',gripper_opening_m=seed['joint7']*2)
 hardware.publish(String(data=json.dumps(h)));ready.publish(String(data=json.dumps(dict(active=False,execution_enabled=False,reason='test_only'))))
 vision.publish(String(data=json.dumps(dict(valid=True,frame='base_link',generator='bottle_cylinder',stamp_sec=stamp.sec+stamp.nanosec*1e-9,selected_id=0,objects=[obj]))))
 cloud=copy.deepcopy(cloud_template);cloud.header.stamp=stamp;cloud_pub.publish(cloud)

timer=node.create_timer(.1,publish);executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node);thread=threading.Thread(target=executor.spin);thread.start()
log=open('/tmp/graspgen-direct-place-test-movegroup.log','w')
process=subprocess.Popen(['ros2','launch','piper_with_gripper_moveit','move_group.launch.py'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
 r=subprocess.run(['ros2','run','piper_moveit_bridge','add_table'],capture_output=True,text=True,timeout=25);print(r.stdout,r.stderr,flush=True);assert r.returncode==0
 client=node.create_client(ApplyPlanningScene,'/apply_planning_scene');assert client.wait_for_service(timeout_sec=5)
 camera=json.loads((ROOT/'calibration/current_collision_readonly_20261105.json').read_text())['attached'][0]
 attached=AttachedCollisionObject();attached.link_name='link6';attached.touch_links=['gripper_base','link6']
 o=attached.object;o.id='graspgen_camera_and_mount';o.header.frame_id='link6';o.operation=CollisionObject.ADD
 values=[float(v) for v in re.findall(r'[xyzw]=([-+0-9.eE]+)',camera['object']['object_pose'])]
 assert len(values)==7
 o.pose.position.x,o.pose.position.y,o.pose.position.z=values[:3]
 o.pose.orientation.x,o.pose.orientation.y,o.pose.orientation.z,o.pose.orientation.w=values[3:]
 from shape_msgs.msg import SolidPrimitive
 o.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.034,.129,.054])]
 local=Pose();local.orientation.w=1.;o.primitive_poses=[local]
 scene=PlanningScene();scene.is_diff=True;scene.robot_state.is_diff=True;scene.robot_state.attached_collision_objects=[attached]
 # Simulate a retained last-cycle bottle subsequently moved by the user.
 old=CollisionObject();old.id='graspgen_target_bottle';old.header.frame_id='base_link';old.pose.orientation.w=1.
 old.pose.position.x=.35;old.pose.position.y=-.35;old.pose.position.z=config['table_z_m']+config['total_height_m']/2
 old.primitives=[SolidPrimitive(type=SolidPrimitive.CYLINDER,dimensions=[config['total_height_m'],config['diameter_m']/2+.002])]
 local_old=Pose();local_old.orientation.w=1.;old.primitive_poses=[local_old];old.operation=CollisionObject.ADD
 scene.world.collision_objects=[old]
 f=client.call_async(ApplyPlanningScene.Request(scene=scene));end=time.monotonic()+5
 while not f.done() and time.monotonic()<end:time.sleep(.02)
 assert f.done() and f.result().success
 # This subprocess is PLAN ONLY. It cannot create actuator action clients.
 result=subprocess.run(['ros2','run','piper_moveit_bridge','bottle_grasp'],capture_output=True,text=True,timeout=150)
 print(result.stdout,result.stderr,flush=True)
 report=ROOT/'doc/reports/direct_random_place_plan_test_20261105.txt';report.write_text('RETURN '+str(result.returncode)+'\n'+result.stdout+result.stderr)
 assert result.returncode==0,'Full direct/random-place plan failed'
 assert 'PLAN ONLY' in result.stdout
 assert 'Previous bottle location verified empty' in result.stdout
 print('PASS: actual MoveIt direct grasp -> close sweep -> held lift/carry/place -> release/retreat preflight; no execution clients or CAN',flush=True)
finally:
 if process.poll() is None:
  os.killpg(process.pid,signal.SIGINT)
  try:process.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
 log.close();executor.shutdown();thread.join(timeout=2);node.destroy_node();rclpy.shutdown()
