from .speed_settings import APPROACH_VELOCITY_SCALE,APPROACH_ACCELERATION_SCALE,ARM_EXECUTION_CAP
"""Plan a recorded observation joint pose; movement requires explicit --execute."""
import argparse
import json
import math
import time
from collections import deque
from pathlib import Path
import yaml
import rclpy
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit_msgs.msg import Constraints,JointConstraint,DisplayTrajectory
from moveit_msgs.srv import GetMotionPlan
from rosidl_runtime_py.convert import message_to_ordereddict
from .model import ARM,Model,validate_trajectory
from .execute_pregrasp import HoverExecution


def load_goal(path,model):
    data=yaml.safe_load(Path(path).expanduser().read_text())
    angles=data['joints_degrees']
    if set(angles)!=set(ARM) or any(not isinstance(v,(int,float)) or not math.isfinite(v) for v in angles.values()):
        raise ValueError('Observation pose must specify exactly six finite joint angles in degrees')
    goal={name:math.radians(angles[name]) for name in ARM}
    model.check(dict(goal,joint7=0.))
    return data,goal


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    share=Path(get_package_share_directory('piper_moveit_bridge'))
    parser.add_argument('--config',default=str(share/'config/observation_pose.yaml'))
    parser.add_argument('--output',default=str(Path.home()/'GraspGen/calibration/observation_plan_preview.json'))
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    model=Model(Path(get_package_share_directory('piper_description'))/'urdf/piper_description.urdf')
    config,goal=load_goal(args.config,model)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('piper_observation_pose')
    states=deque(maxlen=1);hardware=deque(maxlen=1);ready=deque(maxlen=1);subs=[]
    for kind,topic,dst in [(JointState,'/joint_states',states),(String,'/piper_hardware/status',hardware),(String,'/piper_bridge/readiness',ready)]:
        subs.append(node.create_subscription(kind,topic,lambda m,dst=dst:dst.append((time.monotonic(),m)),10))
    runner=HoverExecution();runner.motion_name='Observation pose'
    def wait(predicate,timeout=15,error='Observation pose operation timed out'):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate():return
        raise RuntimeError(error)
    def inputs():
        if any(not x or time.monotonic()-x[-1][0]>.3 for x in [states,hardware,ready]):
            raise ValueError('No fresh arm/gateway/bridge feedback')
        js=states[-1][1];stamp=js.header.stamp.sec+js.header.stamp.nanosec*1e-9
        if not -.1<=node.get_clock().now().nanoseconds*1e-9-stamp<=.3:raise ValueError('Stale joint timestamp')
        h=json.loads(hardware[-1][1].data);b=json.loads(ready[-1][1].data)
        if args.execute:runner.check_hardware(h,b)
        else:
            if h.get('outputs_enabled') or b.get('execution_enabled') or b.get('active'):
                raise ValueError('Close both gates for plan-only observation preview')
            reason=h.get('reason','')
            if reason and not (reason.startswith('Teaching/recording mode active;') or reason=='Motors not all enabled'):
                raise ValueError(f'Invalid hardware feedback: {reason}')
        return js,model.feedback(js)
    try:
        client=node.create_client(GetMotionPlan,'/plan_kinematic_path')
        runner.prepare(node,wait,need_action=args.execute)
        if not client.wait_for_service(timeout_sec=10):raise RuntimeError('MoveIt planning service unavailable')
        until=time.monotonic()+3;wait(lambda:time.monotonic()>=until,5)
        wait(lambda:all([states,hardware,ready]))
        runner.check_scene()
        start,measured=inputs()
        request=GetMotionPlan.Request();m=request.motion_plan_request;m.group_name='arm'
        m.start_state.joint_state=start;m.allowed_planning_time=10.;m.num_planning_attempts=3
        m.max_velocity_scaling_factor=APPROACH_VELOCITY_SCALE;m.max_acceleration_scaling_factor=APPROACH_ACCELERATION_SCALE
        constraints=Constraints();constraints.name='recorded_observation_joint_goal'
        for name in ARM:
            constraints.joint_constraints.append(JointConstraint(joint_name=name,position=goal[name],tolerance_above=.001,tolerance_below=.001,weight=1.))
        m.goal_constraints=[constraints]
        print('Observation joint goal (degrees): '+json.dumps(config['joints_degrees']),flush=True)
        future=client.call_async(request);wait(future.done,20,error='MoveIt observation planning response timed out; no execution requested')
        plan=future.result().motion_plan_response
        if plan.error_code.val!=1:raise RuntimeError(f'Observation planning failed: {plan.error_code.val}')
        trajectory=plan.trajectory.joint_trajectory
        times,_=validate_trajectory(trajectory,ARM,measured,model,ARM_EXECUTION_CAP,.005,.005)
        if plan.trajectory.multi_dof_joint_trajectory.points:raise ValueError('Unexpected multi-DOF trajectory')
        end=dict(zip(trajectory.joint_names,trajectory.points[-1].positions))
        if max(abs(end[name]-goal[name]) for name in ARM)>.002:raise ValueError('Plan endpoint differs from observation goal')
        path=Path(args.output).expanduser();path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps({'display_only':True,'purpose':'observation_pose','goal_degrees':config['joints_degrees'],
            'start_joints':message_to_ordereddict(start),'trajectory':message_to_ordereddict(plan.trajectory)},indent=2)+'\n')
        publisher=node.create_publisher(DisplayTrajectory,'/display_planned_path',10)
        display=DisplayTrajectory();display.model_id='piper';display.trajectory_start=plan.trajectory_start;display.trajectory=[plan.trajectory]
        for _ in range(3):
            publisher.publish(display);until=time.monotonic()+.2;wait(lambda:time.monotonic()>=until,2)
        print(f'Observation plan succeeded: duration={times[-1]:.1f}s, points={len(trajectory.points)}; saved={path}',flush=True)
        if not args.execute:
            print('PLAN ONLY: no execution client or motion request; inspect MoveIt RViz trajectory.',flush=True)
            return
        runner.check_scene()
        _,actual=inputs()
        if max(abs(actual[name]-measured[name]) for name in ARM)>.005:
            raise ValueError('Arm moved during planning; no execution requested')
        runner.execute_arm_plan(model,plan,hardware,ready,states)
        print('Observation position reached. Verify fresh RGB-D/object status before any pregrasp; no automatic grasp.',flush=True)
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (RuntimeError,ValueError,KeyError,TypeError) as exc:
        node.get_logger().error(str(exc));raise SystemExit(1)
    finally:
        runner.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
