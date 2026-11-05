"""Measured upright bottle: geometric centre -> MoveIt -> contact -> 10mm lift."""
import argparse
import copy
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.convert import message_to_ordereddict
from .grasp_runtime import Runtime, ROOT, atomic_json, digest
from .grasp_workflow import make_pose
from .model import ARM
from .tool_config import load_tcp
from .bottle_planning import contact_model,plan_pick_place,remaining_plans,lift_target
from .table_placement import free_table_candidates
from .bottle_scene import BottleScene


def candidate(rt, config):
    if not rt.vision or time.monotonic()-rt.vision[-1][0]>.8:
        raise ValueError('No fresh generator status; start vision with generator:=bottle_cylinder')
    data=json.loads(rt.vision[-1][1].data)
    if not data.get('valid'):raise ValueError('Vision: '+data.get('reason','Invalid bottle fit'))
    if data.get('frame')!='base_link' or data.get('generator')!='bottle_cylinder':
        raise ValueError('Use generator:=bottle_cylinder; PCA/empirical centres cannot drive this mode')
    if not -.1<=rt.node.get_clock().now().nanoseconds*1e-9-data.get('stamp_sec',0)<=.8:
        raise ValueError('Stale fitted bottle timestamp')
    objects=[o for o in data['objects'] if o.get('id')==data.get('selected_id') and o.get('valid')]
    if len(objects)!=1:raise ValueError('Missing unique selected bottle')
    obj=objects[0]
    if obj.get('class')!='bottle' or obj.get('position_semantics')!='bottle_grasp_center':
        raise ValueError('Require upright measured bottle centre')
    if obj.get('handeye_sha256')!=digest(ROOT/'calibration/handeye_result.yaml'):
        raise ValueError('Generator calibration differs from selected hand-eye file; restart full stack')
    for key in ('diameter_m','total_height_m','grasp_height_m','body_bottom_m','body_top_m','table_z_m'):
        if key not in obj:raise ValueError('Restart updated grasp generator: missing '+key)
        if not np.isfinite(obj[key]) or abs(obj[key]-config.get(key,0.))>1e-6:
            raise ValueError('Generator bottle dimensions differ from current configuration')
    if not 0<=obj['fit_rms_m']<=.002 or obj['arc_deg']<45:
        raise ValueError('Unreliable cylinder fit')
    center=np.asarray(obj['center_m'],dtype=float)
    if center.shape!=(3,) or not np.isfinite(center).all() or abs(center[2]-(config.get('table_z_m',0.)+config['grasp_height_m']))>1e-6:
        raise ValueError('Invalid fitted grip centre')
    return center,data,obj


def run(execute=False,reach_only=False,pick_only=False):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('piper_bottle_grasp');rt=None;scene=None;possible_hold=False;placed=False
    log=ROOT/'calibration/grasp_runs'/f'bottle_{time.time_ns()}.json'
    record={'method':'measured_upright_bottle','started_unix':time.time(),
            'execute_requested':execute,'reach_only':reach_only,'pick_only':pick_only,'stages':[],'physical_grasp_verified':False}
    try:
        config_path=ROOT/'calibration/bottle_geometry.json'
        config=json.loads(config_path.read_text())
        keys=('diameter_m','total_height_m','body_bottom_m','body_top_m','grasp_height_m')
        if any(not isinstance(config.get(k),(int,float)) or not np.isfinite(config[k]) for k in keys):
            raise ValueError('Fill measured bottle dimensions in calibration/bottle_geometry.json')
        tcp_config=load_tcp();tcp=np.array(tcp_config['translation_m'])
        if not np.allclose(tcp,[0,0,.1358],atol=1e-6):
            raise ValueError('This commissioning mode requires selected model fingertip-level TCP [0,0,0.1358]m')
        baseline={str(p):digest(p) for p in (config_path,ROOT/'calibration/tcp_config.json',ROOT/'calibration/handeye_result.yaml')}
        rt=Runtime(node,execute=execute);rt.runner.motion_name='Measured bottle grasp'
        rt.runner.check_scene();start,q=rt.inputs()
        contact=contact_model(config)
        expected_open=contact['observed_opening_m']+.008
        if q['joint7']*2<expected_open-.0005:
            raise ValueError(f'Open EMPTY gripper to at least {expected_open*1000:.1f}mm before planning; no motion sent')
        if execute:
            h=json.loads(rt.hardware[-1][1].data);b=json.loads(rt.ready[-1][1].data)
            if h.get('gripper_enabled') is not True or h.get('gripper_fault') is not False or not b.get('gripper_contact_tolerance_supported'):
                raise ValueError('Require healthy gripper and contact-aware bridge')
        samples=[];end=time.monotonic()+20;last_reason='Waiting for fit'
        while time.monotonic()<end:
            rt.wait(lambda:True,.1)
            try:
                center,data,obj=candidate(rt,config)
                if not samples or samples[-1][0]!=data['stamp_sec']:
                    samples.append((data['stamp_sec'],center,data,obj));samples=samples[-5:]
                if len(samples)>=5 and samples[-1][0]-samples[0][0]>=.3:
                    if max(np.linalg.norm(s[1]-center) for s in samples)>.003:
                        samples=[];raise ValueError('Fitted centre moved >3mm')
                    break
            except (ValueError,KeyError) as exc:
                last_reason=str(exc);samples=[]
        else:raise ValueError('No stable bottle within 20s: '+last_reason)
        _,center,data,obj=samples[-1]
        place_locations=[]
        if not reach_only and not pick_only:
            points=rt.scene_points_at(data['stamp_sec'])
            place_locations=free_table_candidates(points,center[:2],config.get('table_z_m',0.),config['diameter_m'])
            record['random_placement_candidates_xy']=place_locations
        scene=BottleScene(rt,center,config['diameter_m'],config['total_height_m'],table_z=config.get('table_z_m',0.),
                          source_scene_points=lambda:rt.scene_points_at(data['stamp_sec']))
        record.update(config=config,tcp=tcp_config,source_snapshot=data,configuration_sha256=baseline,
            target_tcp_base_m=center.tolist(),direct_grasp=True,
            contact_model=contact,
            collision_model='target cylinder and existing MoveIt scene; placement support from observed full cloud')
        plans=plan_pick_place(rt,scene,start,center,config,tcp,contact,place_locations,
                             reach_only=reach_only,pick_only=pick_only)
        record['selected_candidate']={'name':plans['choice']['name'],'tilt_deg':plans['choice']['tilt_deg']}
        record['planning_attempts']=plans['attempts']
        if 'place_xy' in plans:record['selected_random_place_xy']=plans['place_xy']
        phases=[plans[name] for name in ('direct','test_lift','carry_lift','transfer','place','retreat') if name in plans]
        rt.show(start,phases);record['trajectories']=[message_to_ordereddict(t) for t in phases]
        atomic_json(log,record)
        description='direct to grip only' if reach_only else 'direct -> close -> 10mm lift' if pick_only else 'direct -> close -> lift -> random same-table placement -> open -> retreat'
        print('Full plan:',description,'; candidate=',plans['choice']['name'],'; log=',log,flush=True)
        if 'place_xy' in plans:print('Observed clear random place XY(m):',plans['place_xy'],flush=True)
        if not execute:
            print('PLAN ONLY: no actuator action client created.');return
        _,current=rt.inputs()
        if max(abs(current[n]-q[n]) for n in ARM)>.002 or abs(current['joint7']-q['joint7'])>.0005:
            raise ValueError('Start changed during planning')
        fresh,_,_=candidate(rt,config)
        if np.linalg.norm(fresh-center)>.003:raise ValueError('Bottle moved during planning; no execution')
        if any(digest(p)!=value for p,value in baseline.items()):raise ValueError('Configuration changed during planning')
        rt.runner.check_scene();rt.arm(plans['direct']);record['stages'].append('direct_grasp_pose');atomic_json(log,record)
        if reach_only:
            print('Direct grip pose reached. No close/lift requested.');return
        # Target is fixed after the final pre-submission check. Never chase
        # eye-in-hand close-up noise or silently replay a failed movement.
        record['target_policy']='fixed after final pre-submission vision check'
        scene.contacts()
        actual,_=rt.inputs();actual_grasp=rt.pose(actual);scene.attach(actual_grasp);possible_hold=True
        rt.close_gripper(contact['command_opening_m'],contact=contact)
        record['stages'].append('contact_checked');atomic_json(log,record)
        actual,_=rt.inputs();lift=rt.cart_plan(actual,lift_target(rt,actual,.01))
        rt.runner.check_scene();rt.arm(lift);record['stages'].append('test_lift');atomic_json(log,record)
        if pick_only:
            record['controller_sequence_succeeded']=True
            print('10mm test lift complete. Verify hold visually; no transport/release.');return
        # Replan transport with actual contact opening and actual attachment.
        actual,_=rt.inputs()
        from .grasp_calibration import pose_array
        _,held_rotation=pose_array(actual_grasp)
        rest=remaining_plans(rt,scene,actual,plans['place_xy'],dict(config,tcp_translation_m=tcp.tolist()),held_rotation,contact['observed_opening_m']+.008)
        for phase in ('carry_lift','transfer','place'):
            # Earlier controller success allows a small terminal tracking error.
            # Never reuse the predicted start of the next preflight trajectory.
            actual,_=rt.inputs();target=rest['carry_targets'][phase]
            if phase=='transfer':
                segment=rt.pose_plan(actual,target,label='Carry from measured state',keep_orientation=True)
            else:
                segment=rt.cart_plan(actual,target)
            rt.runner.check_scene();rt.arm(segment);record['stages'].append(phase);atomic_json(log,record)
        # Release only at a fully planned, observed supported location.
        rt.open_gripper(rest['release_opening_m'])
        possible_hold=False;placed=True;scene.placed(plans['place_xy'])
        record['stages'].append('release_confirmed');atomic_json(log,record)
        actual,_=rt.inputs();retreat=rt.cart_plan(actual,lift_target(rt,actual,.05));rt.arm(retreat)
        record['stages'].append('retreat');record['controller_sequence_succeeded']=True
        print('Pick/place controller sequence complete. Verify bottle stands stably; placed collision object retained.')
    except KeyboardInterrupt:
        record['error']='Interrupted';raise SystemExit(130)
    except (ValueError,RuntimeError,KeyError,TypeError,OSError) as exc:
        record['error']=str(exc);node.get_logger().error(str(exc));raise SystemExit(1)
    finally:
        if rt:rt.close()
        if scene and not possible_hold and not placed:
            try:scene.cleanup()
            except Exception as exc:record['scene_cleanup_error']=str(exc)
        record['placed_object_retained']=placed;record['possible_held_object']=possible_hold;record['finished_unix']=time.time();atomic_json(log,record)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--execute',action='store_true')
    modes=parser.add_mutually_exclusive_group();modes.add_argument('--reach-only',action='store_true');modes.add_argument('--pick-only',action='store_true')
    args=parser.parse_args();run(args.execute,reach_only=args.reach_only,pick_only=args.pick_only)
