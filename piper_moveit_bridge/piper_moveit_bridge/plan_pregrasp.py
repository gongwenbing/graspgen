from .speed_settings import APPROACH_VELOCITY_SCALE,APPROACH_ACCELERATION_SCALE,ARM_EXECUTION_CAP
"""One-shot pregrasp planning only. Deliberately has no execution client."""
import argparse,json,math,time
from collections import deque
from pathlib import Path
import rclpy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped,Pose
from std_msgs.msg import String,Header
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import Constraints,PositionConstraint,OrientationConstraint,DisplayTrajectory
from moveit_msgs.srv import GetMotionPlan
from rosidl_runtime_py.convert import message_to_ordereddict
from .model import Model,ARM
from .reachability import wrist_reach_bound
from .hover_geometry import tilt_hover


class TargetUnavailable(ValueError):
    pass


def make_constraints(pose):
    c=Constraints();c.name='display_only_pregrasp_link6'
    position=PositionConstraint();position.header=Header(frame_id=pose.header.frame_id);position.link_name='link6';position.weight=1.
    sphere=SolidPrimitive();sphere.type=SolidPrimitive.SPHERE;sphere.dimensions=[.003]
    region=Pose();region.position=pose.pose.position;region.orientation.w=1.
    position.constraint_region.primitives=[sphere];position.constraint_region.primitive_poses=[region]
    orientation=OrientationConstraint();orientation.header=Header(frame_id=pose.header.frame_id);orientation.link_name='link6';orientation.orientation=pose.pose.orientation
    orientation.absolute_x_axis_tolerance=.03;orientation.absolute_y_axis_tolerance=.03;orientation.absolute_z_axis_tolerance=.03;orientation.weight=1.
    c.position_constraints=[position];c.orientation_constraints=[orientation]
    return c


def main(execution=None):
    p=argparse.ArgumentParser(description='Fresh-plan pregrasp hover execution; requires --execute and open gates. No descent or gripper command.' if execution is not None else __doc__)
    p.add_argument('--output',default='calibration/pregrasp_plan_preview.json')
    p.add_argument('--wait-target',type=float,default=10.,help='Seconds to wait for a fresh valid target (default: 10)')
    p.add_argument('--tilt-deg',type=float,default=0.,help='Explicit outward hover tilt 0..20deg about the same 135.8mm model TCP (default: 0)')
    if execution is not None:
        p.add_argument('--execute',action='store_true',required=True,help='Explicitly request a fresh pregrasp plan and execution')
    args=p.parse_args()
    if execution is not None:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    else:rclpy.init()
    node=rclpy.create_node('pregrasp_execution' if execution else 'pregrasp_plan_only')
    state=deque(maxlen=1);target=deque(maxlen=1);hardware=deque(maxlen=1);readiness=deque(maxlen=1);preview=deque(maxlen=1);vision=deque(maxlen=1)
    def track(dst,msg):dst.append((time.monotonic(),msg))
    subscriptions=[]
    for kind,topic,dst in [(JointState,'/joint_states',state),(PoseStamped,'/grasp_preview/pregrasp_link6',target),
        (String,'/piper_hardware/status',hardware),(String,'/piper_bridge/readiness',readiness),(String,'/grasp_preview/status',preview),(String,'/graspgen/status',vision)]:
        subscriptions.append(node.create_subscription(kind,topic,lambda m,dst=dst:track(dst,m),10))
    def wait(pred,timeout=15,error='Timed out: keep camera, vision, preview, hand-eye TF and MoveIt running'):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if pred():return
        raise RuntimeError(error)
    def validate_inputs():
        for label,container in [('hardware',hardware),('bridge',readiness),('preview',preview),('joints',state)]:
            if not container or time.monotonic()-container[-1][0]>.5:raise ValueError(f'No fresh {label} input')
        h=json.loads(hardware[-1][1].data);b=json.loads(readiness[-1][1].data);v=json.loads(preview[-1][1].data)
        if execution is None:
            if h['outputs_enabled'] or b['execution_enabled'] or b['active']:raise ValueError('Close both output gates before this plan-only check')
        else:
            execution.check_hardware(h,b)
        # A plan-only query does not require CAN motion mode or enabled motors.
        # Still reject transport/controller/limit faults; feedback is checked below.
        reason=h['reason']
        if reason and not (reason.startswith('Teaching/recording mode active;') or reason=='Motors not all enabled'):
            raise ValueError(f'Hardware feedback invalid: {reason}')
        if not v.get('display_only') or not v.get('target_valid'):
            detail=v.get('reason','No valid preview')
            if vision and time.monotonic()-vision[-1][0]<1.:
                raw=json.loads(vision[-1][1].data)
                rejected=[f'{o.get("class","object")}: {o["reason"]}' for o in raw.get('objects',[]) if o.get('reason')]
                if not raw.get('valid'):
                    detail+='; vision: '+('; '.join(rejected[:3]) or raw.get('reason','No valid candidate'))
            raise TargetUnavailable(f'Pregrasp unavailable (/grasp_preview/status): {detail}')
        if execution is not None:
            height=float(v.get('hover_height_m',float('nan')))
            if not math.isfinite(height) or height<.10:
                raise ValueError('Execution requires a finite hover height >= 0.10m')
            from .execute_pregrasp import check_surface_above_table
            check_surface_above_table(v)
        if not target or time.monotonic()-target[-1][0]>.5:raise TargetUnavailable('No fresh /grasp_preview/pregrasp_link6 target')
        pose=target[-1][1]
        stamp=pose.header.stamp.sec+pose.header.stamp.nanosec*1e-9
        age=node.get_clock().now().nanoseconds*1e-9-stamp
        if pose.header.frame_id!='base_link':raise ValueError('Non-base target')
        if not -.1<=age<=1.0:raise TargetUnavailable('Stale target')
        q=pose.pose.orientation;pos=pose.pose.position
        values=[pos.x,pos.y,pos.z,q.x,q.y,q.z,q.w]
        if not all(math.isfinite(x) for x in values) or abs(sum(x*x for x in values[3:])-1)>.001:raise ValueError('Invalid target pose')
        js=state[-1][1];t=js.header.stamp.sec+js.header.stamp.nanosec*1e-9
        if not -.1<=node.get_clock().now().nanoseconds*1e-9-t<=.3:raise ValueError('Stale joint feedback')
        tcp_xyz=v.get('tcp_translation_m',[0.,0.,.1358])
        if args.tilt_deg:
            # All available workcell previews currently use this TCP. Refuse
            # a mismatch rather than silently tilt about a different point.
            tcp=v.get('pregrasp_tcp_base_m');flange=v.get('pregrasp_link6_base_m')
            if tcp is not None and flange is not None and abs(math.dist(tcp,flange)-math.sqrt(sum(x*x for x in tcp_xyz)))>.001:
                raise ValueError('Tilt requires preview TCP offset 0.1358m')
        return tilt_hover(pose,args.tilt_deg,tcp_xyz=tcp_xyz),js
    try:
        if not math.isfinite(args.tilt_deg) or not 0<=args.tilt_deg<=20:raise ValueError('--tilt-deg must be within 0..20 degrees')
        client=node.create_client(GetMotionPlan,'/plan_kinematic_path')
        if execution is not None:execution.prepare(node,wait)
        if not client.wait_for_service(timeout_sec=10):raise RuntimeError('MoveIt /plan_kinematic_path unavailable')
        # Fast DDS request/reply endpoints discover independently. Service
        # availability alone does not prove the response endpoint is matched.
        # Keep spinning before sampling the target, rather than delaying a
        # previously captured pose or replaying a timed-out request.
        discovery_until=time.monotonic()+3.
        wait(lambda:time.monotonic()>=discovery_until,5)
        wait(lambda:all((state,hardware,readiness,preview)))
        refresh=time.monotonic()+.2;wait(lambda:time.monotonic()>=refresh)
        if not math.isfinite(args.wait_target) or not 0<=args.wait_target<=60:
            raise ValueError('--wait-target must be between 0 and 60 seconds')
        deadline=time.monotonic()+args.wait_target
        announced=False
        while True:
            try:
                pose,js=validate_inputs()
                break
            except TargetUnavailable:
                if time.monotonic()>=deadline:raise
                if not announced:
                    print(f'Waiting up to {args.wait_target:g}s for a fresh valid vision target; no execution requested',flush=True)
                    announced=True
                rclpy.spin_once(node,timeout_sec=.02)
        model_path=Path(get_package_share_directory('piper_description'))/'urdf/piper_description.urdf'
        model=Model(model_path)
        model.feedback(js)
        if execution is not None:execution.check_scene()
        bound=wrist_reach_bound(pose,model_path)
        if bound and bound['outside_geometric_bound']:
            raise ValueError(f"Target outside geometric reach for this orientation: wrist distance >= {bound['wrist_distance_min_m']:.3f} m, arm bound {bound['main_arm_length_sum_m']:.3f} m. Move the object closer to the base; no execution requested.")
        req=GetMotionPlan.Request();m=req.motion_plan_request;m.group_name='arm'
        m.allowed_planning_time=10.;m.num_planning_attempts=5;m.max_velocity_scaling_factor=APPROACH_VELOCITY_SCALE;m.max_acceleration_scaling_factor=APPROACH_ACCELERATION_SCALE
        m.start_state.joint_state=js;m.goal_constraints=[make_constraints(pose)]
        print(f'{"Fresh pregrasp plan" if execution else "Plan only"}: link6 goal base XYZ={pose.pose.position.x:.4f}, {pose.pose.position.y:.4f}, {pose.pose.position.z:.4f} m',flush=True)
        if args.tilt_deg:print(f'Explicit hover tilt={args.tilt_deg:g}deg; same TCP target, no descent or gripper command',flush=True)
        future=client.call_async(req)
        try:
            wait(future.done,20,error='MoveIt /plan_kinematic_path response timed out after 20s. Target was received; check move_group logs for service response/DDS errors. No execution requested; do not reuse an older saved plan.')
        except RuntimeError:
            client.remove_pending_request(future)
            future.cancel()
            raise
        plan=future.result().motion_plan_response
        if plan.error_code.val!=1:raise RuntimeError(f'MoveIt planning failed, error_code={plan.error_code.val}; no execution requested')
        trajectory=plan.trajectory.joint_trajectory
        if not trajectory.points:raise ValueError('MoveIt returned empty trajectory')
        current=model.feedback(js)
        for point in trajectory.points:
            whole=dict(current);whole.update(zip(trajectory.joint_names,point.positions));model.check(whole)
        out=Path(args.output).expanduser();out.parent.mkdir(parents=True,exist_ok=True)
        out.write_text(json.dumps({'display_only':True,'tcp_provisional':True,'environment_collision_model_complete':False,
            'vision_status_at_plan':json.loads(vision[-1][1].data) if vision else None,
            'preview_status_at_plan':json.loads(preview[-1][1].data),
            'start_joints':message_to_ordereddict(js),
            'hover_tilt_deg':args.tilt_deg,
            'target_link6':message_to_ordereddict(pose),'trajectory':message_to_ordereddict(plan.trajectory)},indent=2)+'\n')
        pub=node.create_publisher(DisplayTrajectory,'/display_planned_path',10)
        display=DisplayTrajectory();display.model_id='piper';display.trajectory_start=plan.trajectory_start;display.trajectory=[plan.trajectory]
        for _ in range(3):
            pub.publish(display);until=time.monotonic()+.2;wait(lambda:time.monotonic()>=until,2)
        print(f'Plan succeeded: points={len(trajectory.points)}; saved={out.resolve()}',flush=True)
        if execution is None:
            print('PLAN ONLY: no execution requested. Only existing planning-scene collisions checked; physical TCP and environment remain unverified.',flush=True)
        else:
            execution.run(model,pose,js,plan,validate_inputs,hardware,readiness,state)
    except (RuntimeError,ValueError,KeyError,TypeError) as e:
        node.get_logger().error(str(e));raise SystemExit(1)
    finally:
        if execution is not None:execution.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
