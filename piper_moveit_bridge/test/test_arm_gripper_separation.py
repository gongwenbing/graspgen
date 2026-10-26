import threading,time
from types import SimpleNamespace as NS
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from piper_moveit_bridge.can_gateway import CanGateway
from piper_moveit_bridge.trajectory_bridge import joint_command
from piper_moveit_bridge.model import ARM


def fake_gateway():
    measured=dict(zip(ARM,[0.,.8,-.7,0.,.4,0.]));calls=[]
    node=NS(lock=threading.RLock(),command_received_count=0,read_only=False,enabled=True,
        feedback_ready=lambda:'',snapshot=lambda:(measured.copy(),.0395,None,[True]*6),
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(time.time()*1e9))),
        model=NS(max_opening=.07,limits={n:(-4,4,.2) for n in ARM},check=lambda q:None),
        last_command=dict(measured,gripper=.035),sent_at=time.monotonic(),max_velocity=.2,
        max_opening_velocity=.01,command_sent_count=0,command_rejected_count=0,last_command_error='',
        sdk=NS(MotionCtrl_2=lambda *a:calls.append('mode'),JointCtrl=lambda *a:calls.append('arm'),GripperCtrl=lambda *a:calls.append('gripper')),
        get_logger=lambda:NS(error=lambda text:None))
    return node,measured,calls


def stamped(q,gripper=None):
    msg=JointState();msg.name=list(ARM);msg.position=[q[n] for n in ARM]
    if gripper is not None:msg.name.append('gripper');msg.position.append(gripper)
    now=time.time();msg.header.stamp.sec=int(now);msg.header.stamp.nanosec=int((now-int(now))*1e9)
    return msg


def test_contact_limited_fingers_do_not_change_target_when_arm_moves():
    node,q,calls=fake_gateway()
    CanGateway.command(node,stamped(q))
    assert calls==['mode','arm']
    assert node.enabled and node.command_rejected_count==0
    # Also works after idle baseline has cleared.
    node.last_command=None
    CanGateway.command(node,stamped(q))
    assert calls==['mode','arm','mode','arm']


def test_real_gripper_step_limit_is_still_enforced():
    node,q,calls=fake_gateway()
    CanGateway.command(node,stamped(q,.0395))
    assert not node.enabled and node.command_rejected_count==1
    assert 'gripper command step' in node.last_command_error
    assert not calls


def test_arm_message_omits_gripper_but_gripper_action_includes_it():
    whole=dict.fromkeys(ARM,0.);whole['joint7']=.01975
    arm=joint_command(whole,ARM,Time())
    grip=joint_command(whole,('joint7',),Time())
    assert arm.name==list(ARM) and len(arm.position)==6
    assert grip.name==list(ARM)+['gripper'] and grip.position[-1]==.0395
