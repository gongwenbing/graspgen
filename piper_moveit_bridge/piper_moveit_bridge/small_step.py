from .speed_settings import APPROACH_VELOCITY_SCALE,APPROACH_ACCELERATION_SCALE,ARM_EXECUTION_CAP
"""Plan a <=0.02 rad single-joint step; execution requires --execute and open gates."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint
from .model import ARM, Model


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--joint',choices=ARM,default='joint1')
    parser.add_argument('--delta',type=float,default=0.01)
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    if not 0 < abs(args.delta) <= 0.02:parser.error('delta must be nonzero and at most 0.02 rad')
    rclpy.init();node=rclpy.create_node('piper_small_step');states=[];hardware=[];readiness=[]
    node.create_subscription(JointState,'/joint_states',states.append,10)
    node.create_subscription(String,'/piper_hardware/status',lambda m:hardware.append((time.monotonic(),json.loads(m.data))),10)
    node.create_subscription(String,'/piper_bridge/readiness',lambda m:readiness.append((time.monotonic(),json.loads(m.data))),10)
    def wait(fn,timeout=15):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if fn():return
        raise RuntimeError('Timed out waiting for feedback/planning/execution')
    def check_gates():
        if not hardware or time.monotonic()-hardware[-1][0]>.3:
            raise RuntimeError('No fresh hardware gateway status')
        h=hardware[-1][1]
        if h['read_only'] or not h['outputs_enabled'] or h['reason']:
            raise RuntimeError(f'Hardware output disabled: {h}')
        if not readiness or time.monotonic()-readiness[-1][0]>.3 or not readiness[-1][1]['execution_enabled']:
            raise RuntimeError('Bridge execution gate is disabled')
    try:
        wait(lambda: bool(states))
        model=Model(Path(get_package_share_directory('piper_description'))/'urdf/piper_description.urdf')
        measured=model.feedback(states[-1]);goal=dict(measured);goal[args.joint]+=args.delta;model.check(goal)
        client=node.create_client(GetMotionPlan,'/plan_kinematic_path')
        if not client.wait_for_service(timeout_sec=10):raise RuntimeError('MoveIt planning service unavailable')
        # Refresh subscriptions after blocking discovery, before choosing the start state.
        until=time.monotonic()+.2;wait(lambda:time.monotonic()>=until)
        stamp=states[-1].header.stamp.sec+states[-1].header.stamp.nanosec*1e-9
        if node.get_clock().now().nanoseconds*1e-9-stamp>.3:raise RuntimeError('Joint state is stale')
        measured=model.feedback(states[-1]);goal=dict(measured);goal[args.joint]+=args.delta;model.check(goal)
        req=GetMotionPlan.Request();m=req.motion_plan_request;m.group_name='arm';m.allowed_planning_time=5.;m.num_planning_attempts=3
        m.max_velocity_scaling_factor=APPROACH_VELOCITY_SCALE;m.max_acceleration_scaling_factor=APPROACH_ACCELERATION_SCALE;m.start_state.joint_state=states[-1]
        constraints=Constraints()
        for name in ARM:constraints.joint_constraints.append(JointConstraint(joint_name=name,position=goal[name],tolerance_above=.0002,tolerance_below=.0002,weight=1.))
        m.goal_constraints=[constraints]
        future=client.call_async(req);wait(future.done);plan=future.result().motion_plan_response
        if plan.error_code.val!=1:raise RuntimeError(f'MoveIt planning failed: {plan.error_code.val}')
        print(f'Plan succeeded: {args.joint} delta={args.delta:.6f} rad, points={len(plan.trajectory.joint_trajectory.points)}',flush=True)
        if not args.execute:
            print('Plan only. No trajectory execution requested.',flush=True);return
        check_gates()
        action=ActionClient(node,ExecuteTrajectory,'/execute_trajectory')
        if not action.wait_for_server(timeout_sec=5):raise RuntimeError('MoveIt execution action unavailable')
        check_gates()
        goal_msg=ExecuteTrajectory.Goal();goal_msg.trajectory=plan.trajectory
        future=action.send_goal_async(goal_msg);wait(future.done);handle=future.result()
        if not handle.accepted:raise RuntimeError('MoveIt rejected execution')
        future=handle.get_result_async();wait(future.done,130)
        code=future.result().result.error_code.val
        if code!=1:raise RuntimeError(f'MoveIt execution failed: {code}')
        print('Execution succeeded, confirmed by controller action feedback.',flush=True)
    except (RuntimeError,ValueError,KeyError) as e:
        node.get_logger().error(str(e));raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
