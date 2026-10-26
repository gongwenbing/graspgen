"""Record a successful grasp, then reproduce it near the taught bottle position."""
import argparse
import copy
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Pose
from rosidl_runtime_py.convert import message_to_ordereddict
from .tool_config import load_tcp
from .model import ARM
from .grasp_runtime import Runtime,ROOT,atomic_json,digest
from .grasp_calibration import empirical_profile,corrected_targets,pose_array

PROFILE=ROOT/'calibration/grasp_profile.json'
REFERENCE=ROOT/'calibration/pregrasp_plan_preview.json'
HAND_EYE=ROOT/'calibration/handeye_result.yaml'


def make_pose(position,rotation):
    p=Pose();p.position.x,p.position.y,p.position.z=map(float,position)
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,rotation.as_quat())
    return p


def record_main():
    parser=argparse.ArgumentParser(description='Read-only record of user-confirmed held bottle. No targets sent.')
    parser.add_argument('--confirmed-held',action='store_true',help='User has physically confirmed bottle is held stably')
    parser.add_argument('--lift-mm',type=float,default=10.)
    parser.add_argument('--close-command-mm',type=float,required=True,help='Actual successful close command, not the measured contact opening')
    parser.add_argument('--reference',default=str(REFERENCE));parser.add_argument('--output',default=str(PROFILE))
    args=parser.parse_args()
    if not np.isfinite(args.close_command_mm) or not 5<=args.close_command_mm<=60:
        parser.error('--close-command-mm must be within 5..60mm')
    if not args.confirmed_held:parser.error('Use --confirmed-held only after physically verifying the held bottle')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('record_successful_grasp');rt=None
    try:
        rt=Runtime(node);start,q=rt.inputs();rt.runner.check_scene();pose=rt.pose(start);position,rotation=pose_array(pose)
        reference=json.loads(Path(args.reference).read_text())
        if reference.get('hover_tilt_deg')!=10 or not reference['preview_status_at_plan'].get('target_valid'):
            raise ValueError('Require original valid 10-degree successful-session pregrasp reference')
        selected=reference['vision_status_at_plan'];objects=[o for o in selected['objects'] if o['id']==selected['selected_id'] and o.get('valid')]
        if len(objects)!=1 or objects[0]['class']!='bottle':raise ValueError('Reference must be selected valid bottle')
        tcp_config=load_tcp()
        profile=empirical_profile(reference['preview_status_at_plan']['surface_base_m'],position,rotation,args.lift_mm/1000,q['joint7']*2,tcp_config['translation_m'])
        observed=q['joint7']*2;command=args.close_command_mm/1000
        profile['closed_opening_m']=command;profile['observed_contact_opening_m']=observed
        if abs(observed-command)>.001:
            from .grasp_contact import validate_contact_config,contact_matches
            h=json.loads(rt.hardware[-1][1].data)
            contact={'command_opening_m':command,'observed_opening_m':observed,
                     'observed_effort_nm':h.get('gripper_effort_nm',float('nan'))}
            validate_contact_config(contact)
            if not contact_matches(h,contact):
                raise ValueError('Need fresh healthy contact telemetry from updated gateway to record command/contact difference')
            profile['taught_contact']=contact
        obs=reference['start_joints'];profile['reference_observation_joints_rad']=dict(zip(obs['name'],obs['position']))
        profile.update({'created_unix':time.time(),'user_confirmed_held':True,'recorded_lift_m':args.lift_mm/1000,
            'reference_path':str(Path(args.reference).resolve()),'reference_sha256':digest(args.reference),
            'handeye_sha256':digest(HAND_EYE),'urdf_sha256':digest(rt.model_path),
            'reference_object_extent_m':objects[0]['observed_extent_m'],
            'reference_observed_opening_m':objects[0]['opening_m'],
            'successful_held_joints':message_to_ordereddict(start),
            'successful_held_link6':message_to_ordereddict(pose),
            'manual_corrections_user_reported':{'descent_mm':80,'outward_mm':20,'inward_mm':10,'test_lift_mm':10},
            'tcp_note':'TCP uses shared translation config; empirical correction is not an independent TCP calibration.',
            'tcp_config_at_capture':tcp_config})
        _,after=rt.inputs()
        if max(abs(q[n]-after[n]) for n in ARM)>.001 or abs(q['joint7']-after['joint7'])>.0003:
            raise ValueError('Arm/gripper moved during capture')
        out=Path(args.output)
        if out.exists():raise ValueError(f'Profile already exists: {out}; use another --output to preserve prior evidence')
        if profile.get('taught_contact'):
            contact_evidence=out.parent/f'grasp_contact_{time.time_ns()}.json'
            atomic_json(contact_evidence,{'taught_contact':profile['taught_contact'],'user_confirmed_held':True,'captured_unix':time.time()})
            profile['contact_evidence_file']=str(contact_evidence.resolve());profile['contact_evidence_sha256']=digest(contact_evidence)
        evidence=out.parent/f'grasp_success_{time.time_ns()}.json';atomic_json(evidence,profile)
        profile['evidence_file']=str(evidence);profile['evidence_sha256']=digest(evidence);atomic_json(out,profile)
        print('Recorded successful grasp without movement:',out)
        print('Empirical radial/tangent/Z offset (mm):',np.round(np.array(profile['correction_radial_tangent_z_m'])*1000,3).tolist())
        print('Close command (mm):',round(profile['closed_opening_m']*1000,3),'actual contact opening (mm):',round(observed*1000,3),'TCP remains provisional.')
    except (ValueError,RuntimeError,KeyError,TypeError,OSError) as exc:node.get_logger().error(str(exc));raise SystemExit(1)
    finally:
        if rt:rt.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def validate_profile(profile,model_path):
    if profile.get('schema_version')!=1 or profile.get('method')!='single_success_empirical' or not profile.get('user_confirmed_held'):
        raise ValueError('Missing recorded successful grasp evidence')
    if digest(HAND_EYE)!=profile['handeye_sha256'] or digest(model_path)!=profile['urdf_sha256']:
        raise ValueError('Hand-eye/robot model changed; recapture correction')
    if digest(profile['evidence_file'])!=profile['evidence_sha256']:
        raise ValueError('Successful capture evidence changed')
    for key in ['tcp_translation_m','correction_radial_tangent_z_m','reference_surface_base_m']:
        if len(profile[key])!=3 or not np.all(np.isfinite(profile[key])):raise ValueError('Nonfinite grasp profile')
    if not np.allclose(profile['tcp_translation_m'],load_tcp()['translation_m'],atol=1e-9):
        raise ValueError('TCP changed; empirical correction must be recaptured')
    if not .005<=profile['closed_opening_m']<=.06:raise ValueError('Invalid close opening')
    contact=profile.get('taught_contact')
    if contact is not None:
        from .grasp_contact import validate_contact_config
        validate_contact_config(contact)
        if digest(profile['contact_evidence_file'])!=profile['contact_evidence_sha256']:
            raise ValueError('Taught contact evidence changed')
        if abs(contact['command_opening_m']-profile['closed_opening_m'])>1e-6:
            raise ValueError('Contact command does not match profile close target')


def auto_main():
    parser=argparse.ArgumentParser(description='Local taught-bottle grasp: fresh target, hover, descend, close, test lift. No transport/release.')
    parser.add_argument('--profile',default=str(PROFILE));parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('piper_auto_grasp');rt=None
    run={'started_unix':time.time(),'execute_requested':args.execute,'stages':[],'physical_grasp_verified':False}
    log=ROOT/'calibration/grasp_runs'/f'{time.time_ns()}.json'
    try:
        rt=Runtime(node,execute=args.execute);profile=json.loads(Path(args.profile).read_text());validate_profile(profile,rt.model_path)
        rt.runner.check_scene();start,q=rt.inputs()
        obs=profile['reference_observation_joints_rad']
        if max(abs(q[n]-obs[n]) for n in ARM)>.03:raise ValueError('Return to recorded observation pose first; automatic observation motion is not performed')
        opening=q['joint7']*2;closed=profile['closed_opening_m']
        if args.execute and profile.get('taught_contact'):
            h=json.loads(rt.hardware[-1][1].data)
            if not all(k in h for k in ['gripper_effort_nm','gripper_enabled','gripper_fault','gripper_feedback_age_sec']):
                raise ValueError('Restart updated gateway/bridge for contact telemetry before automatic execution')
            if not json.loads(rt.ready[-1][1].data).get('gripper_contact_tolerance_supported'):
                raise ValueError('Restart updated trajectory bridge before contact-aware execution')
        if opening<max(.04,closed+.008):raise ValueError('Require empty opened gripper before a new grasp; never start while holding bottle')
        samples=[];last_reason='Waiting for five fresh stable frames';end=time.monotonic()+15
        while time.monotonic()<end:
            rt.wait(lambda:True,.1)
            try:
                p,v,obj=rt.candidate()
                if obj['class']!=profile['object_class']:raise ValueError('Wrong object class')
                if abs(obj['opening_m']-profile['reference_observed_opening_m'])>.008:raise ValueError('Observed bottle width differs from taught bottle')
                stamp=v['stamp_sec']
                if not samples or stamp!=samples[-1][0]:samples.append((stamp,np.array(p['surface_base_m']),p,v,obj))
                samples=samples[-6:]
                if len(samples)>=5 and samples[-1][0]-samples[0][0]>=.3:
                    if max(np.linalg.norm(s[1]-samples[-1][1]) for s in samples)>.005:
                        last_reason='Target moved more than 5mm across stable-window frames';samples=[];continue
                    break
            except ValueError as exc:
                last_reason=str(exc);samples=[]
        else:raise ValueError('No stable fresh taught-bottle target within 15s: '+last_reason)
        surface=samples[-1][1];preview,vision,obj=samples[-1][2:]
        preview_tcp=preview.get('tcp_translation_m',[0,0,.1358])
        if not np.allclose(preview_tcp,profile['tcp_translation_m'],atol=1e-6):
            raise ValueError('Preview TCP configuration differs from recorded profile; restart preview after TCP changes')
        hover,grasp,lift,rotation=corrected_targets(profile,surface)
        run.update({'vision_snapshot':vision,'preview_snapshot':preview,'targets_link6_base_m':{'hover':hover.tolist(),'grasp':grasp.tolist(),'lift':lift.tolist()}})
        atomic_json(log,run)
        target=make_pose(hover,rotation)
        approach=rt.hover_plan(start,target);hover_state=rt.state_after(start,approach.joint_trajectory)
        # Keep the actual solved hover orientation for a strictly straight descent.
        solved=rt.pose(hover_state);solved_grasp=copy.deepcopy(solved);solved_grasp.position.z-=profile['approach_height_m']
        descent=rt.cart_plan(hover_state,solved_grasp);grasp_state=rt.state_after(hover_state,descent.joint_trajectory)
        # Validate the finger sweep with the object absent from the scene; this
        # checks robot/environment collisions, not desired bottle contact.
        close_state=copy.deepcopy(grasp_state);index=close_state.name.index('joint7')
        for width in np.linspace(opening,closed,12):
            close_state.position[index]=float(width/2);rt.valid_state(close_state)
        solved_lift=copy.deepcopy(solved_grasp);solved_lift.position.z+=profile['test_lift_m']
        lift_plan=rt.cart_plan(close_state,solved_lift)
        rt.show(start,[approach,descent,lift_plan])
        run.update({'profile_sha256':digest(args.profile),'vision_snapshot':vision,'preview_snapshot':preview,
            'targets_link6_base_m':{'hover':hover.tolist(),'grasp':grasp.tolist(),'lift':lift.tolist()},
            'tcp_provisional':True,'object_collision_geometry_complete':False,
            'trajectories':[message_to_ordereddict(t) for t in [approach,descent,lift_plan]],
            'close_opening_m':closed,'lift_m':profile['test_lift_m']})
        atomic_json(log,run)
        print('Full local grasp planned: approach -> descend -> close to recorded opening -> lift 10mm.',flush=True)
        print('Physical contact/slip not detected; bottle collision volume not modeled. Log:',log,flush=True)
        if not args.execute:
            print('PLAN ONLY: no arm or gripper execution client created.');return
        _,current=rt.inputs()
        if max(abs(q[n]-current[n]) for n in ARM)>.002 or abs(q['joint7']-current['joint7'])>.0005:
            raise ValueError('Arm/gripper changed during preflight')
        fresh,_,newobj=rt.candidate()
        if newobj['class']!=obj['class'] or np.linalg.norm(np.array(fresh['surface_base_m'])-surface)>.005:
            raise ValueError('Target moved during planning; no execution requested')
        for stage,trajectory in [('hover',approach),('descent',descent)]:
            rt.runner.check_scene();rt.arm(trajectory);run['stages'].append({'stage':stage,'confirmed_unix':time.time()});atomic_json(log,run)
        if profile.get('taught_contact'):
            rt.close_gripper(closed,contact=profile['taught_contact'])
        else:
            rt.close_gripper(closed)
        run['stages'].append({'stage':'close_evidence_checked','confirmed_unix':time.time()});atomic_json(log,run)
        # Contact stops the fingers wider than the command; plan using the
        # real measured finger geometry, not the hypothetical fully closed state.
        lift_start,_=rt.inputs();actual_lift=rt.pose(lift_start);actual_lift.position.z+=profile['test_lift_m']
        lift_plan=rt.cart_plan(lift_start,actual_lift)
        rt.runner.check_scene();rt.arm(lift_plan);run['stages'].append({'stage':'test_lift','confirmed_unix':time.time()});run['controller_sequence_succeeded']=True
        print('Controller sequence completed. Visually confirm bottle lifted and did not slip. Physical grasp is NOT automatically verified. No release/transport.',flush=True)
    except KeyboardInterrupt:
        run['error']='Interrupted';raise SystemExit(130)
    except (ValueError,RuntimeError,KeyError,TypeError,OSError) as exc:
        run['error']=str(exc);node.get_logger().error(str(exc));raise SystemExit(1)
    finally:
        if rt:rt.close()
        run['finished_unix']=time.time();atomic_json(log,run)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
