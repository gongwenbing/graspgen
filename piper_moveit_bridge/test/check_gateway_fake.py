#!/usr/bin/env python3
"""Exercise CAN write gates with a fake SDK and fake CAN bus; never open hardware."""
import os,time,math
from types import SimpleNamespace as S
from unittest.mock import patch
os.environ['ROS_DOMAIN_ID']='192';os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_LOG_DIR']='/tmp/graspgen-moveit-logs'
import rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from piper_moveit_bridge.can_gateway import CanGateway
from piper_moveit_bridge.model import ARM

class SDK:
    def __init__(self,**kwargs):
        self.calls=[];self.joints=[0,45837,-40107,0,22918,0];self.width=20000
        self.status=S(ctrl_mode=1,teach_status=0,err_code=0,arm_status=0)
    def ConnectPort(self,**kwargs):assert kwargs=={'piper_init':False}
    def DisconnectPort(self):pass
    def GetArmJointMsgs(self):return S(joint_state=S(**{f'joint_{i}':v for i,v in enumerate(self.joints,1)}))
    def GetArmGripperMsgs(self):return S(gripper_state=S(grippers_angle=self.width))
    def GetArmStatus(self):return S(arm_status=self.status)
    def GetArmLowSpdInfoMsgs(self):return S(**{f'motor_{i}':S(foc_status=S(driver_enable_status=True)) for i in range(1,7)})
    def EnableArm(self,*a):self.calls.append(('enable',a))
    def DisableArm(self,*a):self.calls.append(('disable',a))
    def ResetPiper(self):self.calls.append(('reset',()))
    def MotionCtrl_2(self,*a):self.calls.append(('mode',a))
    def JointCtrl(self,*a):self.calls.append(('joints',a))
    def GripperCtrl(self,*a):self.calls.append(('gripper',a))
class Bus:
    def recv(self,timeout):time.sleep(.005);return None
    def shutdown(self):pass

rclpy.init()
with patch('piper_sdk.C_PiperInterface',SDK),patch('can.Bus',lambda **kwargs:Bus()):
    g=CanGateway()
    try:
        def refresh():
            now=time.monotonic();epoch=time.time()
            for i in [0x2a1,0x2a5,0x2a6,0x2a7,0x2a8]+list(range(0x261,0x267)):g.received[i]=(now,epoch)
        refresh();g.tick();assert not g.sdk.calls and not g.enabled
        for service in [g.enable_motors,g.enable_outputs,g.reset_after_support]:
            assert not service(SetBool.Request(data=True),SetBool.Response()).success
        assert not g.sdk.calls
        g.read_only=False;g.sdk.status.ctrl_mode=2
        assert not g.enable_motors(SetBool.Request(data=True),SetBool.Response()).success
        assert not g.enable_outputs(SetBool.Request(data=True),SetBool.Response()).success
        assert not g.sdk.calls
        assert not g.reset_after_support(SetBool.Request(data=True),SetBool.Response()).success
        g.sdk.status.teach_status=2
        assert not g.reset_after_support(SetBool.Request(data=False),SetBool.Response()).success
        g.enabled=True
        assert not g.reset_after_support(SetBool.Request(data=True),SetBool.Response()).success
        g.enabled=False;g.received[0x2a1]=(0,0)
        assert not g.reset_after_support(SetBool.Request(data=True),SetBool.Response()).success
        refresh()
        assert g.reset_after_support(SetBool.Request(data=True),SetBool.Response()).success
        assert g.sdk.calls==[('reset',())], 'Reset must not enable or send any joint/gripper target'
        assert not g.enabled
        g.sdk.calls.clear();g.sdk.status.teach_status=0
        g.sdk.status.ctrl_mode=1;g.sdk.joints[1]=-1000
        assert not g.enable_outputs(SetBool.Request(data=True),SetBool.Response()).success
        g.sdk.joints[1]=45837;refresh()
        assert g.enable_motors(SetBool.Request(data=True),SetBool.Response()).success
        assert g.sdk.calls==[('enable',(7,))], 'Motor enable must not issue a gripper-close or joint target'
        g.sdk.calls.clear()
        assert g.enable_outputs(SetBool.Request(data=True),SetBool.Response()).success
        assert not g.sdk.calls
        q,width,_,_=g.snapshot();msg=JointState();msg.header.stamp=g.get_clock().now().to_msg();msg.name=list(ARM)+['gripper'];msg.position=[q[n] for n in ARM]+[width]
        g.command(msg)
        assert [c[0] for c in g.sdk.calls]==['mode','joints'], 'Arm hold must not send a gripper command'
        g.sent_at=time.monotonic()-g.command_timeout-1
        refresh();idle_calls=list(g.sdk.calls);g.tick()
        assert g.enabled and g.last_command is None
        assert g.sdk.calls==idle_calls, 'Idle must retain gate without sending keepalive motion'
        msg.header.stamp=g.get_clock().now().to_msg();g.command(msg)
        assert g.enabled and [c[0] for c in g.sdk.calls[-2:]]==['mode','joints']
        old=len(g.sdk.calls);msg.header.stamp.sec-=5;g.command(msg)
        assert not g.enabled and len(g.sdk.calls)==old
        refresh();assert g.enable_outputs(SetBool.Request(data=True),SetBool.Response()).success
        msg.header.stamp=g.get_clock().now().to_msg();msg.position[0]+=1;g.command(msg)
        assert not g.enabled and len(g.sdk.calls)==old
        refresh();assert g.enable_outputs(SetBool.Request(data=True),SetBool.Response()).success
        for k in g.received:g.received[k]=(time.monotonic()-1,time.time()-1)
        g.tick();assert not g.enabled and len(g.sdk.calls)==old
        print('PASS: fake SDK proves passive startup, read-only gates, teaching/limits checks, motor enable without gripper close, stale/overspeed command rejection, feedback watchdog')
    finally:g.close();g.destroy_node();rclpy.shutdown()
