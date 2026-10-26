"""Isolated fake MoveIt action/scene. Never opens CAN or runs real controllers."""
import os
os.environ['ROS_DOMAIN_ID']='196'
os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['ROS_LOG_DIR']='/tmp/graspgen-hover-execution-test'
import json,subprocess,tempfile,threading,time,math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer,CancelResponse
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import CollisionObject,AttachedCollisionObject
from moveit_msgs.srv import GetMotionPlan,GetPlanningScene
from moveit_msgs.action import ExecuteTrajectory
from piper_moveit_bridge.model import ARM

rclpy.init();node=Node('fake_hover_execution');goals=[];scenario='ok';shift=False;drop=False;yaw_shift=False;cancellations=[]
publishers={t:node.create_publisher(k,t,10) for k,t in [(JointState,'/joint_states'),
    (PoseStamped,'/grasp_preview/pregrasp_link6'),(String,'/piper_hardware/status'),
    (String,'/piper_bridge/readiness'),(String,'/grasp_preview/status')]}
q=[0.,.8,-.7,0.,.4,0.,.01]
def publish():
    stamp=node.get_clock().now().to_msg()
    js=JointState();js.header.stamp=stamp;js.name=list(ARM)+['joint7'];js.position=q
    publishers['/joint_states'].publish(js)
    pose=PoseStamped();pose.header.frame_id='base_link';pose.header.stamp=stamp
    pose.pose.position.x=.2+(.05 if shift else 0);pose.pose.position.z=.3;pose.pose.orientation.x=1.;pose.pose.orientation.w=0.
    if yaw_shift:
        pose.pose.orientation.x=math.cos(math.radians(22));pose.pose.orientation.y=-math.sin(math.radians(22))
    publishers['/grasp_preview/pregrasp_link6'].publish(pose)
    h={'read_only':scenario=='readonly','outputs_enabled':not drop,'reason':'','ctrl_mode':1,'teach_status':0,'err_code':0,'motors_enabled':[True]*6}
    b={'execution_enabled':True,'active':False}
    v={'display_only':True,'target_valid':True,'hover_height_m':.1,'surface_base_m':[.2,0.,-.02 if scenario=='below_table' else .0642],'reason':''}
    for topic,data in [('/piper_hardware/status',h),('/piper_bridge/readiness',b),('/grasp_preview/status',v)]:
        publishers[topic].publish(String(data=json.dumps(data)))
def plan(req,res):
    global shift,yaw_shift
    m=res.motion_plan_response;m.error_code.val=1;m.trajectory_start=req.motion_plan_request.start_state
    assert req.motion_plan_request.max_velocity_scaling_factor==.32
    m.trajectory.joint_trajectory.joint_names=list(ARM)
    p=JointTrajectoryPoint();p.positions=q[:6];p.velocities=[0.]*6;p.time_from_start.sec=1
    if scenario=='long':p.time_from_start.sec=164
    m.trajectory.joint_trajectory.points=[p]
    if scenario=='moved':shift=True
    if scenario=='yaw_noise':yaw_shift=True
    return res
def scene(req,res):
    if scenario=='missing_scene':return res
    table=CollisionObject();table.id='graspgen_table';table.header.frame_id='base_link'
    table.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[2.,2.,.05])]
    camera=AttachedCollisionObject();camera.link_name='link6';camera.touch_links=['link6','gripper_base']
    camera.object.id='graspgen_camera_and_mount';camera.object.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.036,.131,.056])]
    res.scene.world.collision_objects=[table];res.scene.robot_state.attached_collision_objects=[camera]
    return res
def execute(handle):
    global drop
    goals.append(handle.request)
    assert set(handle.request.trajectory.joint_trajectory.joint_names)==set(ARM)
    if scenario=='gate_drop':
        drop=True
        until=time.monotonic()+4
        while time.monotonic()<until:
            if handle.is_cancel_requested:
                handle.canceled();r=ExecuteTrajectory.Result();r.error_code.val=-4;return r
            time.sleep(.02)
        handle.abort();r=ExecuteTrajectory.Result();r.error_code.val=-4;return r
    if scenario=='controller_fail':
        handle.abort();r=ExecuteTrajectory.Result();r.error_code.val=-4;return r
    handle.succeed();r=ExecuteTrajectory.Result();r.error_code.val=1;return r
def cancel(handle):
    cancellations.append(True)
    return CancelResponse.ACCEPT
services=[node.create_service(GetMotionPlan,'/plan_kinematic_path',plan),node.create_service(GetPlanningScene,'/get_planning_scene',scene)]
from rclpy.callback_groups import ReentrantCallbackGroup
action=ActionServer(node,ExecuteTrajectory,'/execute_trajectory',execute_callback=execute,cancel_callback=cancel,callback_group=ReentrantCallbackGroup())
timer=node.create_timer(.03,publish)
executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node);thread=threading.Thread(target=executor.spin);thread.start()
try:
    with tempfile.TemporaryDirectory() as tmp:
        command=['ros2','run','piper_moveit_bridge','execute_pregrasp','--execute','--output',str(Path(tmp)/'preview.json')]
        for mode in ['readonly','below_table','missing_scene','moved','long','controller_fail','gate_drop','yaw_noise','tilt','ok']:
            scenario=mode;shift=False;drop=False;yaw_shift=False;before=len(goals)
            result=subprocess.run(command+(['--tilt-deg','10'] if mode=='tilt' else []),capture_output=True,text=True,timeout=25)
            if mode in ['ok','yaw_noise','tilt']:
                assert result.returncode==0,result.stdout+result.stderr
                assert len(goals)==before+1
                print(result.stdout.strip())
            elif mode in ['controller_fail','gate_drop']:
                assert result.returncode!=0,result.stdout+result.stderr
                assert len(goals)==before+1
                if mode=='gate_drop':assert cancellations,result.stdout+result.stderr
                print('STOPPED',mode)
            else:
                assert result.returncode!=0,result.stdout+result.stderr
                assert len(goals)==before,(mode,result.stdout,result.stderr)
                print('REFUSED',mode,result.stderr.strip().splitlines()[0])
    print('PASS: real execution entry refused readonly, missing collision scene and changed target; fake arm-only action succeeded')
finally:
    executor.shutdown();thread.join(timeout=3);action.destroy();node.destroy_node();rclpy.shutdown()
