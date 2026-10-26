#!/usr/bin/env python3
"""Inspect, or explicitly send one CAN-position-mode command with local SDK."""
import argparse
import json
import sys
import time
from pathlib import Path


def check_state(hardware, bridge):
    if hardware.get('outputs_enabled', True) or bridge.get('execution_enabled', True) or bridge.get('active', True):
        raise ValueError('Both output gates must be closed and no trajectory may be active')
    if hardware.get('reason') or hardware.get('read_only', True):
        raise ValueError('Require writable gateway and fresh valid feedback: '+str(hardware.get('reason')))
    if hardware.get('ctrl_mode') not in (0, 1) or hardware.get('teach_status') != 0 or hardware.get('err_code') != 0:
        raise ValueError('Require standby/CAN mode, teach_status=0 and err_code=0; no reset will be sent')
    if hardware.get('motors_enabled') != [True] * 6:
        raise ValueError('All six motors must already be enabled')


def send_mode_only(arm):
    arm.MotionCtrl_2(0x01, 0x01, 10)


def can_feedback_matches(sdk_status, hardware):
    return all(int(sdk_status[k]) == hardware.get(k) == v
               for k, v in [('ctrl_mode', 1), ('teach_status', 0), ('err_code', 0)])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--switch',action='store_true',help='Actually send one CAN control-mode command')
    parser.add_argument('--stream-mode',action='store_true',help='With --switch, repeat only the mode command at 200Hz for two seconds; no joint targets')
    parser.add_argument('--sdk-path',default='/home/zlq/piper_sdk')
    parser.add_argument('--can-port',default='can0')
    args=parser.parse_args()
    import rclpy
    from std_msgs.msg import String
    rclpy.init();node=rclpy.create_node('explicit_can_mode_switch');latest={};subs=[];arm=None;bus=None
    for key,topic in [('hardware','/piper_hardware/status'),('bridge','/piper_bridge/readiness')]:
        subs.append(node.create_subscription(String,topic,lambda m,k=key:latest.update({k:(time.monotonic(),json.loads(m.data))}),10))
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def current():
        if any(k not in latest or time.monotonic()-latest[k][0]>.3 for k in ['hardware','bridge']):
            raise ValueError('No fresh hardware/bridge status')
        h=latest['hardware'][1];b=latest['bridge'][1];check_state(h,b);return h
    try:
        spin(2);h=current();print('BEFORE',json.dumps(h),flush=True)
        if h['ctrl_mode']==1:
            print('Already in CAN control; no command sent. Gates remain closed.');return
        if not args.switch:
            print('Inspect only. Add --switch to send one mode command; no command sent.');return
        root=Path(args.sdk_path).expanduser().resolve()
        if not (root/'piper_sdk/__init__.py').is_file():raise ValueError('SDK source path missing')
        sys.path.insert(0,str(root))
        import can
        import piper_sdk
        if not Path(piper_sdk.__file__).resolve().is_relative_to(root):raise ValueError('Unexpected SDK import path')
        bus=can.Bus(channel=args.can_port,interface='socketcan',receive_own_messages=False)
        arm=piper_sdk.C_PiperInterface_V2(can_name=args.can_port)
        arm.ConnectPort(piper_init=False)
        spin(.3);h=current()
        status=arm.GetArmStatus().arm_status
        if int(status.ctrl_mode)!=h['ctrl_mode'] or int(status.teach_status)!=0 or int(status.err_code)!=0:
            raise ValueError('SDK feedback and ROS state disagree; no command sent')
        # Clear prior captured frames; no target, enable, reset or teach command.
        for _ in range(20000):
            if bus.recv(timeout=0) is None:break
        send_mode_only(arm)
        end=time.monotonic()+2;echo=None;stable_since=None;observed_modes=set();raw_modes={};last_mode_send=time.monotonic()
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=0)
            m=bus.recv(timeout=.001)
            if m is not None and m.arbitration_id==0x151:echo=m.data.hex()
            if m is not None and m.arbitration_id==0x2a1 and len(m.data)==8:
                key=str(m.data[0]);raw_modes[key]=raw_modes.get(key,0)+1
            s=arm.GetArmStatus().arm_status
            sdk_status={k:int(getattr(s,k)) for k in ('ctrl_mode','teach_status','err_code')}
            observed_modes.add(sdk_status['ctrl_mode'])
            h=current()
            if args.stream_mode and time.monotonic()-last_mode_send>=.005:
                send_mode_only(arm);last_mode_send=time.monotonic()
            if can_feedback_matches(sdk_status,h):
                if stable_since is None:stable_since=time.monotonic()
            else:
                stable_since=None
        s=arm.GetArmStatus().arm_status
        sdk_status={k:int(getattr(s,k)) for k in ('ctrl_mode','teach_status','err_code')}
        h=current()
        confirmed=(can_feedback_matches(sdk_status,h) and stable_since is not None
                   and time.monotonic()-stable_since>=.5)
        print('AFTER',json.dumps({**sdk_status,'ros_ctrl_mode':h['ctrl_mode'],
              'observed_sdk_modes':sorted(observed_modes),'raw_status_mode_counts':raw_modes,'local_CAN_0x151_echo':echo,
              'confirmed':confirmed}),flush=True)
        if not confirmed:
            raise ValueError('CAN mode did not remain confirmed by both SDK and ROS for 0.5s. '
                             'Keep gates closed; do not execute. A missing local echo is normal '
                             'when SDK local_loopback=False.')
        print('CAN control confirmed during mode stream.' if args.stream_mode else 'CAN control confirmed after single command.')
        print('No joint/gripper target, reset or enable command sent; gates remain closed. This does not establish motion readiness after the stream stops.')
    except Exception as exc:
        print('FAILED:',exc,file=sys.stderr);raise SystemExit(1)
    finally:
        if arm is not None:arm.DisconnectPort()
        if bus is not None:bus.shutdown()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
