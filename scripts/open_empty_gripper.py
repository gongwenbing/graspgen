#!/usr/bin/env python3
"""Inspect, or explicitly recover an EMPTY closed gripper to 40mm at 5mm/s.

No joint targets, arm enable, mode switch, reset, zeroing or gate changes.
Normal ROS measurement tolerance remains unchanged. Execute only with both
software gates closed, stationary healthy arm and already enabled gripper.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

PREFIX='Gripper feedback exceeds configured measurement tolerance'


def check_state(h,b):
    if h.get('read_only') is not False or h.get('outputs_enabled') is not False:
        raise ValueError('Require writable gateway with output gate CLOSED')
    if b.get('execution_enabled') is not False or b.get('active') is not False:
        raise ValueError('Bridge gate must be CLOSED and no trajectory active')
    if h.get('ctrl_mode') not in (0,1) or h.get('teach_status')!=0 or h.get('err_code')!=0:
        raise ValueError('Require standby/CAN, no teaching and no controller errors')
    if h.get('motors_enabled')!=[True]*6 or h.get('gripper_enabled') is not True or h.get('gripper_fault') is not False:
        raise ValueError('Arm/gripper must already be enabled and healthy; this tool does not enable them')
    width=h.get('gripper_opening_m');age=h.get('gripper_feedback_age_sec')
    if any(not isinstance(v,(int,float)) or not math.isfinite(v) for v in (width,age)) or not 0<=age<=.3:
        raise ValueError(f'Fresh finite gripper telemetry required: width={width!r}m, feedback_age={age!r}s (must be0..0.3s)')
    if not -.002<=width<=.045:
        raise ValueError('Recovery supports raw opening -2..45mm only; inspect larger excursions')
    for reason in (h.get('reason',''),b.get('reason','')):
        if reason and not (reason.startswith(PREFIX) or reason in ('ready','Hardware output gate disabled or stale')):
            raise ValueError('Unrelated feedback fault: '+reason)
    q=h.get('joint_degrees',[])
    if len(q)!=6 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in q):
        raise ValueError('Six finite arm angles required')
    return width,q


def send_open_target(arm,width):
    if not math.isfinite(width) or not 0<=width<=.04:
        raise ValueError('Recovery target outside 0..40mm')
    # SDK 0x159 only: 0.001mm angle, 0.001Nm effort, already enabled, no zero.
    arm.GripperCtrl(round(width*1e6),500,0x01,0x00)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--confirm-empty',action='store_true',help='Physically verified gripper holds no object; actual opening releases anything held')
    parser.add_argument('--sdk-path',default='/home/zlq/piper_sdk')
    parser.add_argument('--can-port',default='can0')
    args=parser.parse_args()
    if args.execute and not args.confirm_empty:parser.error('--execute requires --confirm-empty')
    import rclpy
    from std_msgs.msg import String
    rclpy.init();node=rclpy.create_node('explicit_empty_gripper_open');latest={};arm=None
    for key,topic in [('hardware','/piper_hardware/status'),('bridge','/piper_bridge/readiness')]:
        node.create_subscription(String,topic,lambda m,key=key:latest.update({key:(time.monotonic(),json.loads(m.data))}),5)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    def current():
        if any(k not in latest or time.monotonic()-latest[k][0]>.3 for k in ('hardware','bridge')):
            raise ValueError('No fresh gateway/bridge telemetry')
        if any(node.count_publishers(t)!=1 for t in ('/piper_hardware/status','/piper_bridge/readiness')):
            raise ValueError('Exactly one gateway and bridge required')
        h=latest['hardware'][1];width,q=check_state(h,latest['bridge'][1]);return h,width,q
    try:
        spin(2)
        if 'hardware' in latest:print('BEFORE',json.dumps(latest['hardware'][1]),flush=True)
        h,width,q=current();original=list(q)
        if not args.execute:
            print('INSPECT ONLY: no SDK/CAN connection, no command sent. Explicit --execute --confirm-empty opens ONLY the gripper to 40mm.');return
        if width>=.0395:
            print('Already at least39.5mm open; no command sent.');return
        root=Path(args.sdk_path).expanduser().resolve()
        if not (root/'piper_sdk/__init__.py').is_file():raise ValueError('SDK source path missing')
        sys.path.insert(0,str(root))
        import piper_sdk
        if not Path(piper_sdk.__file__).resolve().is_relative_to(root):raise ValueError('Unexpected SDK path')
        arm=piper_sdk.C_PiperInterface(can_name=args.can_port)
        arm.ConnectPort(can_init=True,piper_init=False)
        spin(1);h,width,q=current()
        sdk_width=arm.GetArmGripperMsgs().gripper_state.grippers_angle/1e6
        if abs(sdk_width-width)>.0005:raise ValueError('SDK/ROS opening mismatch; no target sent')
        def verify_arm():
            _,measured,nowq=current()
            if max(abs(a-b) for a,b in zip(nowq,original))>math.degrees(.002):
                raise ValueError('Arm moved; stop sending recovery targets')
            return measured
        # Validate arm limits independently: gateway's gripper error can precede arm-bound checks.
        from ament_index_python.packages import get_package_share_directory
        from sensor_msgs.msg import JointState
        from piper_moveit_bridge.model import Model,ARM
        model=Model(Path(get_package_share_directory('piper_description'))/'urdf/piper_description.urdf')
        msg=JointState(name=list(ARM)+['gripper'],position=[math.radians(v) for v in q]+[max(0.,width)])
        model.feedback(msg)
        sdk_status=arm.GetArmStatus().arm_status
        if int(sdk_status.ctrl_mode) not in (0,1) or int(sdk_status.teach_status)!=0 or int(sdk_status.err_code)!=0:
            raise ValueError('SDK controller status not healthy; no target sent')
        verify_arm();start=max(0.,width);duration=(.04-start)/.005;began=time.monotonic()
        print('EXECUTE: empty gripper ONLY ->40mm, 5mm/s; no arm/mode/zero/reset commands; gates stay closed.',flush=True)
        best=width;last_progress=time.monotonic()
        while True:
            measured=verify_arm();elapsed=time.monotonic()-began
            if measured>best+.0002:best=measured;last_progress=time.monotonic()
            if time.monotonic()-last_progress>1.5:
                raise ValueError('Gripper opening not progressing; no further targets sent')
            target=min(.04,start+.005*elapsed)
            # Do not advance >2mm ahead of measured opening (initial -1.1mm is allowed).
            if target-measured>.002:raise ValueError('Gripper lag exceeds2mm; stop sending targets')
            send_open_target(arm,target)
            if elapsed>=duration:break
            spin(.02)
        end=time.monotonic()+2;stable_since=None
        while time.monotonic()<end:
            spin(.03);measured=verify_arm()
            if .0395<=measured<=.0415:
                if stable_since is None:stable_since=time.monotonic()
                if time.monotonic()-stable_since>=.3:break
            else:stable_since=None
        else:raise ValueError('40mm opening not confirmed')
        print('AFTER',json.dumps(current()[0]),flush=True)
        print('Opening confirmed; physical zero was not changed. Gates remain closed. Do not grasp until RGB-D and height checks pass.')
    except (RuntimeError,ValueError,KeyError,OSError,TypeError) as exc:
        print('FAILED:',exc,'Last sent opening remains; no automatic close/disable/reset.',file=sys.stderr);raise SystemExit(1)
    finally:
        if arm is not None:arm.DisconnectPort()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
