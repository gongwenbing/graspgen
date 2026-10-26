from .speed_settings import CARTESIAN_JOINT_SPEED,CARTESIAN_LINEAR_SPEED,ARM_EXECUTION_CAP
"""Small Cartesian translation with fixed orientation; plan only unless --execute."""
import argparse
import copy
import json
import math
import time
from collections import deque
from pathlib import Path
from types import SimpleNamespace
import rclpy
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from rosidl_runtime_py.convert import message_to_ordereddict
from .model import ARM, Model, validate_trajectory
from .execute_pregrasp import HoverExecution


def retime(trajectory, measured, min_duration=0., joint_speed=CARTESIAN_JOINT_SPEED):
    """Conservative segment timing: bounded joint speed and requested minimum duration."""
    if set(trajectory.joint_names)!=set(ARM) or len(trajectory.joint_names)!=6:
        raise ValueError('Require exactly six arm joints')
    previous=[measured[n] for n in trajectory.joint_names];elapsed=0.
    for i,p in enumerate(trajectory.points):
        if len(p.positions)!=6 or not all(math.isfinite(v) for v in p.positions):
            raise ValueError('Invalid Cartesian joint point')
        delta=max(abs(a-b) for a,b in zip(p.positions,previous))
        if delta>.05:raise ValueError('Cartesian path has a joint jump above 0.05rad')
        if i==0 and delta<1e-6:elapsed=0.
        else:elapsed+=max(.05,min_duration/max(1,len(trajectory.points)-1),delta/joint_speed)
        p.time_from_start.sec=int(elapsed)
        p.time_from_start.nanosec=round((elapsed-int(elapsed))*1e9)
        p.velocities=[];p.accelerations=[];p.effort=[]
        previous=list(p.positions)
    return elapsed


def radial_displacement(pose, mm, tcp_xyz=None):
    # Use the configured TCP projection, rather than a tilted flange's XY.
    q=pose.orientation;p=pose.position
    from scipy.spatial.transform import Rotation
    offset=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply([0,0,.1358] if tcp_xyz is None else tcp_xyz)
    x=p.x+offset[0];y=p.y+offset[1]
    radius=math.hypot(x,y)
    if radius<.05:raise ValueError('TCP too close to base axis to define outward direction')
    return (mm/1000*x/radius,mm/1000*y/radius,0.)


def check_translation(start, pose, delta):
    a=start.position;b=pose.position
    movement=(b.x-a.x,b.y-a.y,b.z-a.z)
    length2=sum(v*v for v in delta)
    t=sum(x*y for x,y in zip(movement,delta))/length2
    residual=math.sqrt(sum((x-t*y)**2 for x,y in zip(movement,delta)))
    if residual>.001:raise ValueError('Cartesian solution leaves requested straight line by more than 1mm')
    if not -.001/math.sqrt(length2)<=t<=1+.001/math.sqrt(length2):
        raise ValueError('Cartesian solution leaves requested translation interval')
    q=start.orientation;r=pose.orientation
    dot=abs(q.x*r.x+q.y*r.y+q.z*r.z+q.w*r.w)
    if 2*math.acos(min(1.,dot))>math.radians(1):
        raise ValueError('Cartesian orientation changes by more than 1deg')


def check_line(start, pose, mm):
    check_translation(start,pose,(0.,0.,-mm/1000))


def check_gripper_motion(opening, holding, down_mm, outward_mm):
    if holding:
        if outward_mm is not None or down_mm is None or not -10 <= down_mm < 0:
            raise ValueError('Holding-object mode permits only vertical lifts up to 10mm')
        if not math.isfinite(opening) or not .001 < opening <= .07:
            raise ValueError('Require a finite nonzero measured gripper opening for held-object lift')
    elif opening < .04:
        raise ValueError('Open gripper to at least 40mm first; for a held object use --holding-object with a lift <=10mm')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    movement=parser.add_mutually_exclusive_group()
    movement.add_argument('--outward-mm',type=float,help='Horizontal radial translation away from base. Positive moves outward; negative moves toward base. Maximum absolute step 20mm.')
    movement.add_argument('--down-mm',type=float,help='Positive lowers; negative raises. Maximum absolute step 80mm.')
    parser.add_argument('--holding-object',action='store_true',help='Keep current opening; only allow a vertical lift of at most 10mm. Does not verify grip force or attach object geometry.')
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    if args.outward_mm is None and args.down_mm is None:args.down_mm=10.
    if args.outward_mm is not None:
        if not math.isfinite(args.outward_mm) or not 0<abs(args.outward_mm)<=20:
            parser.error('--outward-mm must be nonzero and within -20..20mm')
    elif not math.isfinite(args.down_mm) or not 0<abs(args.down_mm)<=80:
        parser.error('--down-mm must be nonzero and within -80..80mm')
    model=Model(Path(get_package_share_directory('piper_description'))/'urdf/piper_description.urdf')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('piper_cartesian_step')
    states=deque(maxlen=1);hardware=deque(maxlen=1);ready=deque(maxlen=1);subs=[]
    for kind,topic,dst in [(JointState,'/joint_states',states),(String,'/piper_hardware/status',hardware),(String,'/piper_bridge/readiness',ready)]:
        subs.append(node.create_subscription(kind,topic,lambda m,dst=dst:dst.append((time.monotonic(),m)),10))
    runner=HoverExecution();runner.motion_name='Small Cartesian translation'
    def wait(predicate,timeout=15,error='Cartesian operation timed out'):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():return
        raise RuntimeError(error)
    def inputs():
        if any(not x or time.monotonic()-x[-1][0]>.3 for x in [states,hardware,ready]):
            raise ValueError('Missing fresh arm/gateway/bridge feedback')
        js=states[-1][1];stamp=js.header.stamp.sec+js.header.stamp.nanosec*1e-9
        if not -.1<=node.get_clock().now().nanoseconds*1e-9-stamp<=.3:raise ValueError('Stale joint timestamp')
        h=json.loads(hardware[-1][1].data);b=json.loads(ready[-1][1].data)
        if h.get('reason') or h.get('ctrl_mode')!=1 or h.get('teach_status')!=0 or h.get('err_code')!=0:
            raise ValueError('Require fresh CAN control mode, no teaching or errors')
        if b.get('active'):raise ValueError('Another trajectory is active')
        if args.execute:runner.check_hardware(h,b)
        return js,model.feedback(js)
    try:
        fk=node.create_client(GetPositionFK,'/compute_fk')
        cart=node.create_client(GetCartesianPath,'/compute_cartesian_path')
        runner.prepare(node,wait,need_action=args.execute)
        if not fk.wait_for_service(timeout_sec=8) or not cart.wait_for_service(timeout_sec=8):
            raise RuntimeError('MoveIt FK/Cartesian service unavailable')
        until=time.monotonic()+3;wait(lambda:time.monotonic()>=until,5)
        wait(lambda:all([states,hardware,ready]));runner.check_scene()
        if node.count_publishers('/piper_hardware/status')!=1 or node.count_publishers('/piper_bridge/readiness')!=1:
            raise ValueError('Require exactly one gateway and one trajectory bridge')
        start,measured=inputs()
        check_gripper_motion(measured['joint7']*2,args.holding_object,args.down_mm,args.outward_mm)
        def pose_for(js):
            req=GetPositionFK.Request();req.header.frame_id='base_link';req.fk_link_names=['link6'];req.robot_state.joint_state=js
            f=fk.call_async(req);wait(f.done,8,error='FK response timed out')
            result=f.result()
            if result.error_code.val!=1 or len(result.pose_stamped)!=1:raise ValueError('FK failed')
            return result.pose_stamped[0].pose
        origin=pose_for(start);goal=copy.deepcopy(origin)
        from .tool_config import load_tcp
        delta=radial_displacement(origin,args.outward_mm,load_tcp()['translation_m']) if args.outward_mm is not None else (0.,0.,-args.down_mm/1000)
        for axis,value in zip(('x','y','z'),delta):
            setattr(goal.position,axis,getattr(goal.position,axis)+value)
        distance_mm=1000*math.sqrt(sum(v*v for v in delta))
        # Fixed orientation means translating link6 also translates TCP by exactly
        # the same base-Z displacement, regardless of the provisional TCP offset.
        req=GetCartesianPath.Request();req.header.frame_id='base_link';req.start_state.joint_state=start
        req.group_name='arm';req.link_name='link6';req.waypoints=[goal]
        req.max_step=.001;req.jump_threshold=2.;req.revolute_jump_threshold=.05
        req.avoid_collisions=True;req.max_velocity_scaling_factor=.05;req.max_acceleration_scaling_factor=.05
        f=cart.call_async(req);wait(f.done,20,error='Cartesian planning timed out; no execution requested')
        result=f.result()
        if result.error_code.val!=1 or result.fraction<.999999:
            raise ValueError(f'Incomplete Cartesian path: fraction={result.fraction:.3f}, code={result.error_code.val}; no execution requested')
        trajectory=result.solution.joint_trajectory
        if result.solution.multi_dof_joint_trajectory.points:raise ValueError('Unexpected multi-DOF trajectory')
        duration=retime(trajectory,measured,distance_mm/1000/CARTESIAN_LINEAR_SPEED)
        validate_trajectory(trajectory,ARM,measured,model,ARM_EXECUTION_CAP,.005,.005)
        for point in trajectory.points:
            js=copy.deepcopy(start);values=dict(zip(trajectory.joint_names,point.positions))
            js.position=[values.get(n,p) for n,p in zip(start.name,start.position)]
            check_translation(origin,pose_for(js),delta)
        endjs=copy.deepcopy(start);values=dict(zip(trajectory.joint_names,trajectory.points[-1].positions))
        endjs.position=[values.get(n,p) for n,p in zip(start.name,start.position)]
        endpose=pose_for(endjs)
        if math.sqrt(sum((getattr(endpose.position,a)-getattr(goal.position,a))**2 for a in ('x','y','z')))>.001:raise ValueError('Cartesian endpoint error above 1mm')
        plan=SimpleNamespace(trajectory=result.solution)
        path=Path.home()/'GraspGen/calibration/cartesian_step_preview.json'
        path.write_text(json.dumps({'display_only':True,'holding_object':args.holding_object,'down_mm':args.down_mm,'outward_mm':args.outward_mm,'delta_base_m':delta,'trajectory':message_to_ordereddict(result.solution)},indent=2)+'\n')
        publisher=node.create_publisher(DisplayTrajectory,'/display_planned_path',10)
        display=DisplayTrajectory();display.model_id='piper';display.trajectory_start=result.start_state;display.trajectory=[result.solution]
        for _ in range(3):
            publisher.publish(display);until=time.monotonic()+.2;wait(lambda:time.monotonic()>=until,2)
        print(f'Cartesian plan succeeded: base XYZ displacement={[round(v*1000,3) for v in delta]}mm, duration={duration:.1f}s; fixed orientation, no gripper command',flush=True)
        if not args.execute:
            print('PLAN ONLY. Gates need not be changed; no execution client created.');return
        runner.check_scene();_,actual=inputs()
        if max(abs(actual[n]-measured[n]) for n in ARM)>.002 or abs(actual['joint7']-measured['joint7'])>.0005:
            raise ValueError('Arm/gripper changed during planning; no execution requested')
        runner.execute_arm_plan(model,plan,hardware,ready,states)
    except KeyboardInterrupt:raise SystemExit(130)
    except (RuntimeError,ValueError,KeyError,TypeError) as exc:
        node.get_logger().error(str(exc));raise SystemExit(1)
    finally:
        runner.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
