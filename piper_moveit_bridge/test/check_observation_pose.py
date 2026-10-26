"""Synthetic scene and action in domain195. No CAN or real motion."""
import os
os.environ['ROS_DOMAIN_ID']='195';os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_LOG_DIR']='/tmp/graspgen-observation-test'
import copy,json,math,subprocess,tempfile,threading,time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import CollisionObject,AttachedCollisionObject,DisplayTrajectory
from moveit_msgs.srv import GetMotionPlan,GetPlanningScene
from moveit_msgs.action import ExecuteTrajectory
from piper_moveit_bridge.model import ARM
rclpy.init();node=Node('fake_observation_scene');scenario='preview';goals=[];displays=[];q=[0.,.8,-.7,0.,.4,0.,.01];desired=q[:6];desired[0]=.01
pubs={t:node.create_publisher(k,t,10) for k,t in [(JointState,'/joint_states'),(String,'/piper_hardware/status'),(String,'/piper_bridge/readiness')]}
sub=node.create_subscription(DisplayTrajectory,'/display_planned_path',displays.append,10)
def publish():
    stamp=node.get_clock().now().to_msg();js=JointState();js.header.stamp=stamp;js.name=list(ARM)+['joint7'];js.position=q;pubs['/joint_states'].publish(js)
    execute=scenario not in ['preview','missing_scene']
    h={'read_only':not execute,'outputs_enabled':execute and scenario!='closed','reason':'','ctrl_mode':0,'teach_status':2 if scenario=='teach' else 0,'err_code':0,'motors_enabled':[True]*6}
    b={'execution_enabled':execute and scenario!='closed','active':False}
    for topic,data in [('/piper_hardware/status',h),('/piper_bridge/readiness',b)]:pubs[topic].publish(String(data=json.dumps(data)))
def plan(req,res):
    assert req.motion_plan_request.group_name=='arm'
    assert req.motion_plan_request.max_velocity_scaling_factor==.32
    constraints=req.motion_plan_request.goal_constraints[0].joint_constraints
    assert all(abs(c.position-desired[ARM.index(c.joint_name)])<1e-8 for c in constraints)
    m=res.motion_plan_response;m.error_code.val=1;m.trajectory_start=req.motion_plan_request.start_state
    m.trajectory.joint_trajectory.joint_names=list(ARM)
    a=JointTrajectoryPoint();a.positions=q[:6]
    b=JointTrajectoryPoint();b.positions=desired;b.time_from_start.sec=1
    if scenario=='fast':b.velocities=[.09]*6
    m.trajectory.joint_trajectory.points=[a,b]
    if scenario=='drift':q[1]+=.02
    return res
def scene(req,res):
    if scenario=='missing_scene':return res
    t=CollisionObject();t.id='graspgen_table';t.header.frame_id='base_link';t.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[2.,2.,.05])]
    c=AttachedCollisionObject();c.link_name='link6';c.object.id='graspgen_camera_and_mount';c.touch_links=['link6','gripper_base'];c.object.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.036,.131,.056])]
    res.scene.world.collision_objects=[t];res.scene.robot_state.attached_collision_objects=[c];return res
def execute(handle):
    goals.append(handle.request)
    assert set(handle.request.trajectory.joint_trajectory.joint_names)==set(ARM)
    q[:6]=desired;publish();time.sleep(.05)
    handle.succeed();r=ExecuteTrajectory.Result();r.error_code.val=1;return r
services=[node.create_service(GetMotionPlan,'/plan_kinematic_path',plan),node.create_service(GetPlanningScene,'/get_planning_scene',scene)]
action=ActionServer(node,ExecuteTrajectory,'/execute_trajectory',execute_callback=execute)
timer=node.create_timer(.03,publish)
ex=MultiThreadedExecutor(num_threads=3);ex.add_node(node);thread=threading.Thread(target=ex.spin);thread.start()
try:
    with tempfile.TemporaryDirectory() as tmp:
        cfg=Path(tmp)/'pose.yaml';cfg.write_text(yaml.safe_dump({'joints_degrees':{n:math.degrees(v) for n,v in zip(ARM,desired)}}))
        command=['ros2','run','piper_moveit_bridge','observation_pose','--config',str(cfg),'--output',str(Path(tmp)/'plan.json')]
        for mode in ['preview','missing_scene','closed','teach','drift','fast','execute']:
            scenario=mode;q[:]=[0.,.8,-.7,0.,.4,0.,.01];before=len(goals)
            result=subprocess.run(command+([] if mode in ['preview','missing_scene'] else ['--execute']),capture_output=True,text=True,timeout=25)
            if mode=='preview':
                assert result.returncode==0,result.stdout+result.stderr
                assert displays and len(goals)==before
                print('PASS default preview: no action goal')
            elif mode=='execute':
                assert result.returncode==0,result.stdout+result.stderr
                assert len(goals)==before+1
                print(result.stdout.strip())
            else:
                assert result.returncode!=0,result.stdout+result.stderr
                assert len(goals)==before
                print('REFUSED',mode)
    print('PASS: observation pose needs no vision candidate; planning, gate/teach/start/velocity checks and arm-only execution')
finally:
    ex.shutdown();thread.join(timeout=3);action.destroy();node.destroy_node();rclpy.shutdown()
