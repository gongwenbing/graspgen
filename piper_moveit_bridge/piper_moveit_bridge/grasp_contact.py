"""Narrow, taught-bottle contact evidence; not a general force/slip detector."""
import math


def validate_contact_config(config):
    values=[config[k] for k in ['command_opening_m','observed_opening_m','observed_effort_nm']]
    if any(not isinstance(v,(int,float)) or not math.isfinite(v) for v in values):
        raise ValueError('Invalid taught contact parameters')
    command,observed,effort=values
    if not .005<=command<=.06 or not .001<observed-command<=.005 or not .25<=abs(effort)<=2.:
        raise ValueError('Taught contact outside supported narrow gap/effort range')


def contact_matches(hardware,config):
    if hardware.get('gripper_fault') is not False or hardware.get('gripper_enabled') is not True:
        return False
    width=hardware.get('gripper_opening_m');effort=hardware.get('gripper_effort_nm')
    age=hardware.get('gripper_feedback_age_sec')
    if any(not isinstance(v,(int,float)) or not math.isfinite(v) for v in [width,effort,age]):return False
    if not 0<=age<=.3:return False
    reference=config['observed_effort_nm'];expected=config['observed_opening_m']
    return (abs(width-expected)<=.0015 and width>config['command_opening_m']+.001
            and effort*reference>0 and abs(effort)>=max(.25,abs(reference)*.3))
