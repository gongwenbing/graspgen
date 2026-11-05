"""Plan all pick/place phases before any actuator execution."""
import copy
import numpy as np
from .grasp_workflow import make_pose
from .grasp_calibration import pose_array
from .bottle_candidates import direct_candidates



def contact_model(config):
    """Keep measured cylinder size separate from calibrated jaw feedback."""
    from .grasp_contact import validate_contact_config
    observed=config.get('gripper_contact')
    if observed is None:
        return {'command_opening_m':config['diameter_m']-.002,
                'observed_opening_m':config['diameter_m'],'observed_effort_nm':-.25}
    if observed.get('user_confirmed_body_grasp') is not True:
        raise ValueError('Contact calibration requires confirmed body grasp')
    if abs(observed.get('diameter_m',0)-config['diameter_m'])>1e-6:
        raise ValueError('Contact calibration belongs to a different bottle diameter')
    if observed.get('tilt_deg')!=20.:
        raise ValueError('Contact calibration supports only the confirmed +20 degree grasp')
    result={k:observed[k] for k in ('command_opening_m','observed_opening_m','observed_effort_nm')}
    validate_contact_config(result)
    if result['observed_effort_nm']>=0 or result['observed_opening_m']+.008>.07:
        raise ValueError('Invalid calibrated closing contact or release opening')
    return result


def after(rt,start,trajectory):
    return rt.state_after(start,trajectory.joint_trajectory)


def lift_target(rt,state,height):
    pose=rt.pose(state);pose.position.z+=height;return pose


def remaining_plans(rt,scene,start,place_xy,config,rotation,opening):
    """Held bottle stays upright; gripper orientation is kept during transport."""
    tcp=np.array(config['tcp_translation_m']);table=config.get('table_z_m',0.)
    place_tcp=np.r_[place_xy,table+config['grasp_height_m']]
    place=make_pose(place_tcp-rotation.apply(tcp),rotation)
    ok,code=rt.pose_ik(start,place)
    if not ok:raise ValueError(f'Placement IK/collision rejection {code}')
    raised=lift_target(rt,start,.08)
    lift=rt.cart_plan(start,raised);raised_state=after(rt,start,lift)
    transfer_goal=copy.deepcopy(place);transfer_goal.position.z+=.09
    transfer=rt.pose_plan(raised_state,transfer_goal,label='Carry to random placement',keep_orientation=True)
    above_state=after(rt,raised_state,transfer)
    lower=rt.cart_plan(above_state,place);place_state=after(rt,above_state,lower)
    # Check the actual object is placed before opening; arm stays fixed.
    index=place_state.name.index('joint7');release_state=copy.deepcopy(place_state)
    for width in np.linspace(place_state.position[index]*2,opening,12):
        release_state.position[index]=float(width/2);rt.valid_state(release_state)
    saved_attachment=scene.attachment()
    try:
        scene.placed(place_xy)
        retreat=rt.cart_plan(release_state,lift_target(rt,release_state,.05))
    finally:
        scene.restore_attachment(saved_attachment)
    return dict(carry_lift=lift,transfer=transfer,place=lower,retreat=retreat,
                place_tcp=place_tcp.tolist(),release_opening_m=opening,
                carry_targets=dict(carry_lift=copy.deepcopy(raised),transfer=copy.deepcopy(transfer_goal),place=copy.deepcopy(place)))


def plan_pick_place(rt,scene,start,center,config,tcp,contact,place_locations,*,reach_only=False,pick_only=False):
    opening=contact['observed_opening_m']+.008;attempts=[]
    for choice in direct_candidates(center,config['diameter_m'],config['total_height_m'],tcp,config.get('table_z_m',0.)):
        if config.get('gripper_contact') and choice['name']!='tilt_+20':continue
        scene.source();name=choice['name'];goal=make_pose(choice['grasp_flange'],choice['rotation'])
        report=dict(candidate=name,tilt_deg=choice['tilt_deg'],stage='direct_ik');attempts.append(report)
        print('Planning direct grasp candidate:',name,flush=True)
        try:
            ok,code=rt.pose_ik(start,goal)
            if not ok:raise ValueError(f'Open-gripper IK/collision rejection {code}')
            report['stage']='direct_path'
            direct=rt.pose_plan(start,goal,label='Direct grasp')
            grasp_state=after(rt,start,direct);pose=rt.pose(grasp_state);pos,rotation=pose_array(pose)
            if np.linalg.norm(pos+rotation.apply(tcp)-center)>.002:
                raise ValueError('Solved TCP differs from bottle centre')
            result=dict(direct=direct,choice=choice,actual_rotation=rotation,attempts=attempts)
            if reach_only:
                report['accepted']=True;return result
            # Finger/bottle collisions were enabled for the whole direct path.
            # Permit expected finger contact only for closing and holding.
            scene.contacts();closed=copy.deepcopy(grasp_state);index=closed.name.index('joint7')
            report['stage']='close_sweep'
            for width in np.linspace(grasp_state.position[index]*2,contact['command_opening_m'],15):
                closed.position[index]=float(width/2);rt.valid_state(closed)
            # Use widest permitted measured contact geometry for preflight.
            closed.position[index]=(contact['observed_opening_m']+.0015)/2
            scene.attach(pose);report['stage']='test_lift'
            test_lift=rt.cart_plan(closed,lift_target(rt,closed,.01));lift_state=after(rt,closed,test_lift)
            result['test_lift']=test_lift
            if pick_only:
                scene.source();report['accepted']=True;return result
            report['placements']=[]
            for xy in place_locations:
                report['stage']='carry_place_release_retreat';entry={'xy':xy};report['placements'].append(entry)
                try:
                    plans=remaining_plans(rt,scene,lift_state,xy,dict(config,tcp_translation_m=tcp.tolist()),rotation,opening)
                    result.update(plans);result['place_xy']=xy;entry['accepted']=True
                    scene.source();report['accepted']=True;return result
                except ValueError as exc:
                    entry['error']=str(exc)
                    scene.source();scene.contacts();scene.attach(pose)
            raise ValueError('No random placement has a complete collision-free carry/place/retreat plan')
        except ValueError as exc:
            report['error']=str(exc);print('Candidate rejected:',name,str(exc),flush=True)
            scene.source()
    raise ValueError('All direct grasp candidates failed; no motion requested. '+str(attempts))
