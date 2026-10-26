"""Resume only close/lift after a logged, successful descent; never replay approach."""
import argparse
import copy
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from .grasp_runtime import Runtime,ROOT,atomic_json,digest
from .grasp_workflow import PROFILE,validate_profile
from .model import ARM
from .grasp_calibration import pose_array


def check_resume_log(record,profile_hash,now):
    if not record.get('execute_requested') or record.get('profile_sha256')!=profile_hash:
        raise ValueError('Resume log must be executed with the unchanged profile')
    if [s.get('stage') for s in record.get('stages',[])]!=['hover','descent']:
        raise ValueError('Only logs with completed hover/descent and no close/lift may resume')
    if not 0<=now-record['stages'][-1]['confirmed_unix']<=1800:
        raise ValueError('Descent record expired (30 minutes); do not replay')
    if len(record.get('trajectories',[]))!=3:raise ValueError('Missing original planned trajectory')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log',required=True);parser.add_argument('--profile',default=str(PROFILE));parser.add_argument('--execute',action='store_true')
    args=parser.parse_args();source=Path(args.log).resolve();record=json.loads(source.read_text())
    if record.get('resume_attempt_started_unix'):
        parser.error('This log already had a resume attempt; inspect state instead of repeating')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('piper_resume_grasp');rt=None
    report={'source_log':str(source),'execute_requested':args.execute,'physical_grasp_verified':False,'stages':[]}
    output=ROOT/'calibration/grasp_runs'/f'resume_{time.time_ns()}.json'
    try:
        rt=Runtime(node,execute=args.execute);profile=json.loads(Path(args.profile).read_text());validate_profile(profile,rt.model_path)
        check_resume_log(record,digest(args.profile),time.time());rt.runner.check_scene();start,q=rt.inputs()
        trajectory=record['trajectories'][1]['joint_trajectory']
        endpoint=dict(zip(trajectory['joint_names'],trajectory['points'][-1]['positions']))
        if set(endpoint)!=set(ARM) or max(abs(q[n]-endpoint[n]) for n in ARM)>.003:
            raise ValueError('Arm no longer at logged descent endpoint; no close/lift sent')
        original_position=record['targets_link6_base_m']['grasp'];origin=rt.pose(start)
        if np.linalg.norm(pose_array(origin)[0]-original_position)>.003:
            raise ValueError('FK differs from recorded grasp position')
        closed=profile['closed_opening_m'];opening=q['joint7']*2
        if opening<closed+.008:raise ValueError('Gripper no longer open as expected; cannot safely resume')
        state=copy.deepcopy(start);index=state.name.index('joint7')
        for width in np.linspace(opening,closed,12):
            state.position[index]=float(width/2);rt.valid_state(state)
        goal=copy.deepcopy(origin);goal.position.z+=.01;lift=rt.cart_plan(state,goal)
        rt.show(state,[lift]);report['close_opening_m']=closed
        print('Remaining stages planned: close to recorded opening -> vertical lift 10mm. No approach/descent replay.',flush=True)
        if not args.execute:
            print('PLAN ONLY; no close/lift sent.');return
        _,actual=rt.inputs()
        if max(abs(actual[n]-q[n]) for n in ARM)>.001 or abs(actual['joint7']-q['joint7'])>.0003:
            raise ValueError('Arm/gripper changed during resume preflight')
        record['resume_attempt_started_unix']=time.time();atomic_json(source,record)
        if profile.get('taught_contact'):
            rt.close_gripper(closed,contact=profile['taught_contact'])
        else:
            rt.close_gripper(closed)
        report['stages'].append('close_evidence_checked');atomic_json(output,report)
        actual_state,_=rt.inputs();actual_goal=rt.pose(actual_state);actual_goal.position.z+=.01
        lift=rt.cart_plan(actual_state,actual_goal)
        rt.runner.check_scene();rt.arm(lift);report['stages'].append('test_lift_completed')
        print('Remaining controller sequence succeeded. Visually verify bottle held; no automatic release.',flush=True)
    except KeyboardInterrupt:report['error']='Interrupted';raise SystemExit(130)
    except (RuntimeError,ValueError,KeyError,TypeError,OSError) as exc:
        report['error']=str(exc);node.get_logger().error(str(exc));raise SystemExit(1)
    finally:
        if rt:rt.close()
        atomic_json(output,report);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
