"""Contact handoff test with a synthetic gripper in domain194; no CAN access."""
import os
os.environ['ROS_DOMAIN_ID']='194';os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_LOG_DIR']='/tmp/grasp-contact-test'
import json,threading,time
from collections import deque
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer,ActionClient
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
from piper_moveit_bridge.grasp_runtime import Runtime
from piper_moveit_bridge.execute_pregrasp import HoverExecution
from piper_moveit_bridge.model import ARM,Model
rclpy.init();server=Node('fake_contact_driver');client=Node('contact_client')
mode='good';busy=False;width=.0694;goals=[]
pubs={t:server.create_publisher(k,t,10) for k,t in [(JointState,'/joint_states'),(String,'/piper_hardware/status'),(String,'/piper_bridge/readiness')]}
def publish():
    js=JointState();js.header.stamp=server.get_clock().now().to_msg();js.name=list(ARM)+['joint7'];js.position=[0.,.8,-.7,0.,.4,0.,width/2]
    pubs['/joint_states'].publish(js)
    h={'read_only':False,'outputs_enabled':True,'ctrl_mode':1,'teach_status':0,'err_code':0,'motors_enabled':[True]*6,'reason':'',
       'gripper_fault':False,'gripper_enabled':True,'gripper_opening_m':width,'gripper_effort_nm':1. if mode=='wrong_sign' else -1.,'gripper_feedback_age_sec':1. if mode=='stale' else .01}
    if mode=='missing':h.pop('gripper_effort_nm')
    b={'execution_enabled':True,'active':busy,'gripper_contact_tolerance_supported':True}
    pubs['/piper_hardware/status'].publish(String(data=json.dumps(h)))
    pubs['/piper_bridge/readiness'].publish(String(data=json.dumps(b)))
def execute(handle):
    global busy,width
    goals.append(handle.request);busy=True;width=.0395
    tolerance=handle.request.goal_tolerance[0]
    assert tolerance.name=='joint7' and .00225<tolerance.position<=.003
    publish();time.sleep(.15);handle.succeed();busy=False
    result=FollowJointTrajectory.Result();result.error_code=0;return result
action=ActionServer(server,FollowJointTrajectory,'/gripper_controller/follow_joint_trajectory',execute_callback=execute)
timer=server.create_timer(.02,publish);executor=MultiThreadedExecutor(2);executor.add_node(server)
thread=threading.Thread(target=executor.spin);thread.start()
rt=object.__new__(Runtime);rt.node=client;rt.execute=True;rt.states=deque(maxlen=1);rt.hardware=deque(maxlen=1);rt.ready=deque(maxlen=1)
rt.model=Model(Path(get_package_share_directory('piper_description'))/'urdf/piper_description.urdf')
rt.runner=HoverExecution();rt.gripper_handle=None;rt.gripper_finished=False
rt.gripper=ActionClient(client,FollowJointTrajectory,'/gripper_controller/follow_joint_trajectory')
subs=[]
for kind,topic,dst in [(JointState,'/joint_states',rt.states),(String,'/piper_hardware/status',rt.hardware),(String,'/piper_bridge/readiness',rt.ready)]:
    subs.append(client.create_subscription(kind,topic,lambda msg,dst=dst:dst.append((time.monotonic(),msg)),10))
contact={'command_opening_m':.035,'observed_opening_m':.0395,'observed_effort_nm':-1.044}
try:
    for case in ['good','wrong_sign','stale','missing']:
        mode=case;width=.0694;busy=False;rt.gripper_finished=False;rt.gripper_handle=None
        until=time.monotonic()+.5;rt.wait(lambda:time.monotonic()>=until,2)
        before=len(goals)
        try:
            rt.close_gripper(.035,contact=contact)
            assert case=='good'
            print('PASS stable taught contact despite 4.5mm position gap')
        except (ValueError,RuntimeError) as exc:
            assert case!='good',str(exc)
            if case=='missing':assert len(goals)==before
            print('REFUSED',case,str(exc))
    print('PASS: no physical grip/slip verification claim, no lift or CAN used')
finally:
    rt.close();rt.gripper.destroy();client.destroy_node();executor.shutdown();thread.join(timeout=3);action.destroy();server.destroy_node();rclpy.shutdown()
