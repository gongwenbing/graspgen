from .speed_settings import GRIPPER_OPENING_CAP
"""PiPER 0.6.2 CAN gateway: passive startup; actuator writes require explicit gates."""
import json
import math
import threading
import time
import sys
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from .model import ARM, Model


class CanGateway(Node):
    def __init__(self):
        super().__init__('piper_can_gateway')
        sdk_path=self.declare_parameter('sdk_path','').value
        if sdk_path:
            root=Path(sdk_path).expanduser().resolve()
            if not (root/'piper_sdk/__init__.py').is_file():
                raise ValueError(f'Invalid SDK source directory: {root}')
            sys.path.insert(0,str(root))
        import can
        import piper_sdk
        if sdk_path and not Path(piper_sdk.__file__).resolve().is_relative_to(root):
            raise RuntimeError(f'SDK already imported from unexpected path: {piper_sdk.__file__}')
        from piper_sdk import C_PiperInterface
        self.get_logger().info(f'Using SDK: {piper_sdk.__file__}; interface=C_PiperInterface (V2)')
        self.can_name=self.declare_parameter('can_port','can0').value
        self.read_only=self.declare_parameter('read_only',True).value
        self.timeout=self.declare_parameter('feedback_timeout',0.3).value
        self.command_timeout=self.declare_parameter('command_timeout',0.5).value
        self.max_velocity=self.declare_parameter('max_arm_velocity',0.2).value
        self.max_opening_velocity=self.declare_parameter('max_opening_velocity',GRIPPER_OPENING_CAP).value
        for v in (self.timeout,self.command_timeout,self.max_velocity,self.max_opening_velocity):
            if not math.isfinite(v) or v<=0:raise ValueError('Timeouts and limits must be positive and finite')
        self.model=Model(self.declare_parameter('model_path',str(Path(get_package_share_directory('piper_description'))/'urdf/piper_description.urdf')).value)
        self.lock=threading.RLock();self.received={};self.enabled=False;self.sent_at=0.0;self.last_command=None
        self.command_received_count=0;self.command_sent_count=0;self.command_rejected_count=0
        self.last_command_error=''
        self.stop_event=threading.Event()
        self.sdk=C_PiperInterface(can_name=self.can_name)
        # Explicitly suppress PiperInit, which otherwise sends parameter queries.
        self.sdk.ConnectPort(piper_init=False)
        self.bus=can.Bus(channel=self.can_name,interface='socketcan',receive_own_messages=False)
        self.reader=threading.Thread(target=self.read_frames,daemon=True);self.reader.start()
        self.raw_pub=self.create_publisher(JointState,'/joint_states_single',10)
        self.state_pub=self.create_publisher(String,'/piper_hardware/status',10)
        self.gate_pub=self.create_publisher(Bool,'/piper_hardware/outputs_enabled',10)
        self.create_subscription(JointState,'/moveit/joint_commands',self.command,10)
        self.create_service(SetBool,'~/enable_outputs',self.enable_outputs)
        self.create_service(SetBool,'~/enable_motors',self.enable_motors)
        self.create_service(SetBool,'~/reset_after_support',self.reset_after_support)
        self.create_timer(0.02,self.tick)
        self.get_logger().info(f'CAN gateway started: read_only={self.read_only}; outputs disabled; no automatic enable or homing')

    def read_frames(self):
        while not self.stop_event.is_set():
            try:
                msg=self.bus.recv(timeout=0.1)
                if msg is not None:
                    with self.lock:self.received[msg.arbitration_id]=(time.monotonic(),msg.timestamp)
            except Exception:
                self.stop_event.set();break

    def snapshot(self):
        j=self.sdk.GetArmJointMsgs().joint_state
        q={name:getattr(j,f'joint_{i}')*math.pi/180000.0 for i,name in enumerate(ARM,1)}
        width=self.sdk.GetArmGripperMsgs().gripper_state.grippers_angle/1e6
        status=self.sdk.GetArmStatus().arm_status
        low=self.sdk.GetArmLowSpdInfoMsgs()
        flags=[bool(getattr(low,f'motor_{i}').foc_status.driver_enable_status) for i in range(1,7)]
        return q,width,status,flags

    def feedback_ready(self):
        required=[0x2a1,0x2a5,0x2a6,0x2a7,0x2a8]+list(range(0x261,0x267))
        if self.stop_event.is_set() or any(time.monotonic()-self.received.get(i,(0,0))[0]>self.timeout for i in required):
            return 'Incomplete or stale CAN feedback'
        q,width,status,flags=self.snapshot()
        if int(status.err_code) or int(status.arm_status)!=0:return f'Controller error/status {status.err_code}/{int(status.arm_status)}'
        if int(status.ctrl_mode) not in (0,1) or int(status.teach_status) not in (0,2,6):
            return 'Teaching/recording mode active; exit it manually before enabling control'
        try:
            msg=JointState();msg.name=list(ARM)+['gripper'];msg.position=[q[n] for n in ARM]+[width]
            self.model.feedback(msg)
        except (ValueError,KeyError) as e:return str(e)
        return ''

    def enable_outputs(self,request,response):
        with self.lock:
            if not request.data:
                self.enabled=False;self.last_command=None
                response.success=True;response.message='Outputs disabled; no motor disable or emergency-stop command sent';return response
            error='Gateway is read-only' if self.read_only else self.feedback_ready()
            if not error and not all(self.snapshot()[3]):error='Arm motors not all enabled'
            if error:
                response.success=False;response.message=error;return response
            self.enabled=True;self.last_command=None
            response.success=True;response.message='Output gate enabled; no motion command sent';return response

    def enable_motors(self,request,response):
        with self.lock:
            if self.read_only:
                response.success=False;response.message='Gateway is read-only';return response
            if request.data:
                error=self.feedback_ready()
                if error:
                    response.success=False;response.message=error;return response
                self.sdk.EnableArm(7)
                response.success=True;response.message='Arm enable request sent; verify motors_enabled in status; no joint/gripper target sent'
            else:
                self.enabled=False;self.last_command=None;self.sdk.DisableArm(7)
                response.success=True;response.message='Explicit arm disable sent'
            return response

    def reset_after_support(self,request,response):
        """Explicit supported-arm reset, never an automatic teach-mode transition.

        SDK ResetPiper may remove motor holding torque. data=True acknowledges
        physical support; it does not acknowledge that execution is ready.
        """
        with self.lock:
            if not request.data:
                response.success=False;response.message='No reset sent: data=true requires physical support against loss of holding torque';return response
            if self.read_only:
                response.success=False;response.message='Gateway is read-only';return response
            if self.enabled:
                response.success=False;response.message='Disable output gate before reset';return response
            if self.stop_event.is_set() or time.monotonic()-self.received.get(0x2a1,(0,0))[0]>self.timeout:
                response.success=False;response.message='No fresh controller status; no reset sent';return response
            status=self.snapshot()[2]
            if int(status.ctrl_mode)!=2 or int(status.teach_status)!=2 or int(status.err_code)!=0:
                response.success=False;response.message='Requires teaching mode, recording ended (teach_status=2), and err_code=0';return response
            self.enabled=False;self.last_command=None
            try:
                self.sdk.ResetPiper()
            except Exception as e:
                response.success=False;response.message=f'Reset transmission failed; inspect controller status: {e}';return response
            response.success=True
            response.message='Reset requested: holding torque may be lost. No enable, joint target or gripper target sent. Verify fresh status before proceeding.'
            return response

    def command(self,msg):
        with self.lock:
            self.command_received_count+=1
            if self.read_only or not self.enabled:return
            try:
                error=self.feedback_ready()
                if error:raise ValueError(error)
                if not all(self.snapshot()[3]):raise ValueError('Motor enable feedback lost')
                names=tuple(msg.name)
                has_gripper=set(names)==set(ARM+('gripper',)) and len(names)==7
                if not (has_gripper or (set(names)==set(ARM) and len(names)==6)) or len(msg.position)!=len(names):
                    raise ValueError('Hardware commands require six named arm joints, optionally full gripper opening')
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                age=self.get_clock().now().nanoseconds*1e-9-stamp
                if stamp<=0 or not -0.1<=age<=0.1:raise ValueError('Stale command')
                values=dict(zip(msg.name,msg.position))
                now=time.monotonic();measured,width,_,_=self.snapshot();measured['gripper']=max(0.,min(self.model.max_opening,width))
                normalized={n:values[n] for n in ARM};normalized['joint7']=values.get('gripper',measured['gripper'])/2
                self.model.check(normalized)
                previous=self.last_command if self.last_command is not None else measured
                dt=min(0.1,max(0.005,now-self.sent_at)) if self.last_command is not None else 0.02
                for name in names:
                    cap=self.max_opening_velocity if name=='gripper' else min(self.max_velocity,self.model.limits[name][2])
                    if abs(values[name]-previous.get(name,measured[name]))>cap*dt+0.0001:
                        raise ValueError(f'{name} command step exceeds gateway speed cap')
                # Joint mode only; no EndPoseCtrl, zero-point or limit configuration.
                self.sdk.MotionCtrl_2(0x01,0x01,10)
                self.sdk.JointCtrl(*[round(math.degrees(values[n])*1000) for n in ARM])
                if has_gripper and abs(values['gripper']-previous.get('gripper',measured['gripper']))>0.000001:
                    self.sdk.GripperCtrl(round(values['gripper']*1e6),1000,0x01,0)
                self.command_sent_count+=1;self.last_command_error=''
                self.last_command=values;self.sent_at=now
            except (ValueError,KeyError) as e:
                self.command_rejected_count+=1;self.last_command_error=str(e)
                self.enabled=False;self.last_command=None
                self.get_logger().error(f'Output gate revoked: {e}')

    def tick(self):
        with self.lock:
            q,width,status,flags=self.snapshot()
            groups=(0x2a5,0x2a6,0x2a7,0x2a8)
            if all(i in self.received for i in groups):
                # Use oldest required frame, never refresh an old value with wall time.
                stamp=min(self.received[i][1] for i in groups)
                msg=JointState();msg.header.stamp.sec=int(stamp);msg.header.stamp.nanosec=int((stamp-int(stamp))*1e9)
                msg.name=list(ARM)+['gripper'];msg.position=[q[n] for n in ARM]+[width];self.raw_pub.publish(msg)
            reason=self.feedback_ready()
            if reason or not all(flags):
                self.enabled=False;self.last_command=None
            elif self.last_command is not None and time.monotonic()-self.sent_at>self.command_timeout:
                # Completed/idle stream: retain operator authorization, but rebase
                # the next stream's step check on fresh measured positions.
                # No keepalive targets or automatic motion are sent while idle.
                self.last_command=None
            self.gate_pub.publish(Bool(data=self.enabled))
            grip=self.sdk.GetArmGripperMsgs()
            grip_state=grip.gripper_state;foc=grip_state.foc_status
            grip_fault=any(bool(getattr(foc,n)) for n in ['voltage_too_low','motor_overheating','driver_overcurrent','driver_overheating','sensor_status','driver_error_status'])
            self.state_pub.publish(String(data=json.dumps({'gripper_effort_nm':grip_state.grippers_effort/1000.,
                'gripper_enabled':bool(foc.driver_enable_status),'gripper_fault':grip_fault,
                'gripper_feedback_age_sec':time.time()-grip.time_stamp,
                'read_only':self.read_only,'outputs_enabled':self.enabled,
                'ctrl_mode':int(status.ctrl_mode),'teach_status':int(status.teach_status),'err_code':int(status.err_code),
                'motors_enabled':flags,'joint_degrees':[math.degrees(q[n]) for n in ARM],
                'command_received_count':self.command_received_count,
                'sdk_command_call_count':self.command_sent_count,
                'command_rejected_count':self.command_rejected_count,
                'last_command_error':self.last_command_error,
                'gripper_opening_m':width,'reason':reason or ('' if all(flags) else 'Motors not all enabled')})))

    def close(self):
        self.stop_event.set();self.reader.join(timeout=1);self.bus.shutdown();self.sdk.DisconnectPort()


def main(args=None):
    rclpy.init(args=args);node=CanGateway()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
