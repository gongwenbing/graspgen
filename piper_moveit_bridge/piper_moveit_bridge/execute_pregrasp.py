from .speed_settings import ARM_EXECUTION_CAP
"""Explicit fresh-plan hover execution. No saved-plan loading or gripper commands."""
import math
import json
import time
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from std_srvs.srv import SetBool
from .model import ARM,validate_trajectory


def pose_difference(first, second):
    def unpack(p):
        a=p.pose.position;q=p.pose.orientation
        return np.array([a.x,a.y,a.z]),Rotation.from_quat([q.x,q.y,q.z,q.w])
    a,ra=unpack(first);b,rb=unpack(second)
    # This command only hovers. Freeze the planned wrist yaw instead of
    # tracking the unstable PCA tangent of a nearly circular visible patch.
    # Still reject a changed approach direction and translated target.
    za=ra.as_matrix()[:,2];zb=rb.as_matrix()[:,2]
    approach_angle=math.acos(float(np.clip(np.dot(za,zb),-1.,1.)))
    return float(np.linalg.norm(a-b)),approach_angle


def check_surface_above_table(preview):
    """Current supported workcell has a tabletop at base_link Z=0."""
    surface=preview.get('surface_base_m')
    if not isinstance(surface,list) or len(surface)!=3 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in surface):
        raise ValueError('Missing finite visual object surface position')
    if surface[2] <= 0:
        raise ValueError(f'Visual object surface Z={surface[2]:.4f}m is at/below the configured tabletop Z=0; inspect RGB/depth alignment and object cloud. Do not compensate with TCP or execute this target.')


class HoverExecution:
    motion_name='Pregrasp hover'
    def __init__(self):
        self.handle=None;self.finished=False

    @staticmethod
    def check_hardware(h,b,active_allowed=False):
        if h.get('read_only',True) or not h.get('outputs_enabled') or h.get('reason'):
            raise ValueError(f'Hardware not ready for execution: {h.get("reason") or "read_only/output gate"}')
        # Gateway supports standby and sends CAN position mode together with
        # the first validated trajectory command. No standalone mode switch
        # or replay of an old target is needed here.
        if h.get('ctrl_mode') not in (0,1) or h.get('teach_status')!=0 or h.get('err_code')!=0 or h.get('motors_enabled')!=[True]*6:
            raise ValueError('Require standby/CAN mode=0/1, teach_status=0, no errors and all six motors enabled')
        if not b.get('execution_enabled') or (b.get('active') and not active_allowed):
            raise ValueError('Bridge execution disabled or another trajectory is active')

    def prepare(self,node,wait,need_action=True):
        self.node=node;self.wait=wait
        self.scene=node.create_client(GetPlanningScene,'/get_planning_scene')
        self.action=ActionClient(node,ExecuteTrajectory,'/execute_trajectory') if need_action else None
        if not self.scene.wait_for_service(timeout_sec=5) or (need_action and not self.action.wait_for_server(timeout_sec=5)):
            raise RuntimeError('MoveIt planning scene/execution action unavailable')

    def check_scene(self):
        req=GetPlanningScene.Request()
        req.components.components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY|PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        f=self.scene.call_async(req);self.wait(f.done,8,error='Scene verification timed out; no execution requested')
        scene=f.result().scene
        tables=[o for o in scene.world.collision_objects if o.id=='graspgen_table']
        cameras=[o for o in scene.robot_state.attached_collision_objects if o.object.id=='graspgen_camera_and_mount' and o.link_name=='link6']
        if len(tables)!=1 or len(cameras)!=1:
            raise ValueError('Execution requires graspgen_table and camera/mount attached to link6; run add_table/add_camera with gates closed first')
        if tables[0].header.frame_id!='base_link' or not tables[0].primitives or not cameras[0].object.primitives:
            raise ValueError('Missing/invalid table or camera collision geometry')
        camera=cameras[0]
        if any(name in camera.touch_links for name in ['link7','link8']):
            raise ValueError('Finger collision checks must remain enabled')
        dimensions=camera.object.primitives[0].dimensions
        if len(dimensions)!=3 or any(a<b for a,b in zip(dimensions,[.03,.125,.05])):
            raise ValueError('Camera collision envelope is smaller than measured dimensions')

    def run(self,model,pose,start,plan,validate,hardware,readiness,states):
        self.check_scene()
        fresh,current=validate()
        distance,angle=pose_difference(pose,fresh)
        if distance>.01 or angle>math.radians(5):
            raise ValueError('Vision target position/approach changed during planning (>10mm/5deg); replan, no execution requested')
        original=model.feedback(start);measured=model.feedback(current)
        if max(abs(original[n]-measured[n]) for n in ARM)>.005:
            raise ValueError('Arm moved during planning (>0.005rad); replan, no execution requested')
        trajectory=plan.trajectory.joint_trajectory
        if set(trajectory.joint_names)!=set(ARM) or len(trajectory.joint_names)!=6 or plan.trajectory.multi_dof_joint_trajectory.points:
            raise ValueError('Only six arm joints may execute; gripper/multi-DOF trajectory forbidden')
        # Use the controller's own validator before submitting an action:
        # same 120s duration bound, bounds, derivatives and measured start.
        validate_trajectory(trajectory,ARM,measured,model,ARM_EXECUTION_CAP,.005,.005)
        previous=-1.;positions=None
        for point in trajectory.points:
            t=point.time_from_start.sec+point.time_from_start.nanosec*1e-9
            if not math.isfinite(t) or t<=previous or len(point.positions)!=6:
                raise ValueError('Invalid trajectory time/positions')
            if not all(math.isfinite(x) for x in list(point.positions)+list(point.velocities)+list(point.accelerations)):
                raise ValueError('Nonfinite trajectory')
            if point.velocities and max(abs(x) for x in point.velocities)>ARM_EXECUTION_CAP:
                raise ValueError(f'Pregrasp execution limited to {ARM_EXECUTION_CAP}rad/s')
            if positions is not None and max(abs(a-b)/(t-previous) for a,b in zip(point.positions,positions))>ARM_EXECUTION_CAP:
                raise ValueError(f'Trajectory segment exceeds {ARM_EXECUTION_CAP}rad/s')
            positions=point.positions;previous=t
        if not 0<previous<=120:
            raise ValueError('Pregrasp duration must be within the bridge limit of 120 seconds')
        fresh,current=validate()
        distance,angle=pose_difference(pose,fresh)
        if distance>.01 or angle>math.radians(5) or max(abs(original[n]-model.feedback(current)[n]) for n in ARM)>.005:
            raise ValueError('Target/start changed before submission; no execution requested')
        self.execute_arm_plan(model,plan,hardware,readiness,states)

    def execute_arm_plan(self,model,plan,hardware,readiness,states):
        """Shared six-arm action execution; callers validate their own goal."""
        if self.action is None:raise RuntimeError('No execution client in plan-only mode')
        if any(not x or time.monotonic()-x[-1][0]>.3 for x in [hardware,readiness,states]):
            raise RuntimeError('No fresh execution feedback before submission')
        self.check_hardware(json.loads(hardware[-1][1].data),json.loads(readiness[-1][1].data))
        js=states[-1][1];stamp=js.header.stamp.sec+js.header.stamp.nanosec*1e-9
        if not -.1<=self.node.get_clock().now().nanoseconds*1e-9-stamp<=.3:
            raise RuntimeError('Stale joint feedback before submission')
        trajectory=plan.trajectory.joint_trajectory
        if set(trajectory.joint_names)!=set(ARM) or len(trajectory.joint_names)!=6 or plan.trajectory.multi_dof_joint_trajectory.points:
            raise ValueError('Only six arm joints may execute')
        times,_=validate_trajectory(trajectory,ARM,model.feedback(js),model,ARM_EXECUTION_CAP,.005,.005)
        previous=times[-1]
        goal=ExecuteTrajectory.Goal();goal.trajectory=plan.trajectory
        print(f'EXECUTE: {self.motion_name}, duration={previous:.1f}s; fixed target; no gripper command',flush=True)
        f=self.action.send_goal_async(goal);self.wait(f.done,10,error='Execution goal acknowledgement timed out; check controller status before retrying')
        self.handle=f.result()
        if not self.handle.accepted:raise RuntimeError('MoveIt rejected execution goal')
        result=self.handle.get_result_async();started=time.monotonic();deadline=started+previous*2+15
        while not result.done():
            self.wait(lambda:True,.2)
            if time.monotonic()>deadline:raise RuntimeError('Execution timed out; cancel requested')
            if any(not x or time.monotonic()-x[-1][0]>.5 for x in [hardware,readiness,states]):
                raise RuntimeError('Execution feedback lost; cancel requested')
            h=json.loads(hardware[-1][1].data);b=json.loads(readiness[-1][1].data)
            self.check_hardware(h,b,active_allowed=True)
            if time.monotonic()-started>1 and h.get('ctrl_mode')!=1:
                raise RuntimeError('CAN control mode not established during execution; cancel requested')
            js=states[-1][1];stamp=js.header.stamp.sec+js.header.stamp.nanosec*1e-9
            source_age=self.node.get_clock().now().nanoseconds*1e-9-stamp
            receipt_age=time.monotonic()-states[-1][0]
            if not -.1<=source_age<=.3:
                raise RuntimeError(f'Joint feedback stale during execution: source_age={source_age:.3f}s, receipt_age={receipt_age:.3f}s; cancel requested')
        response=result.result()
        self.finished=True
        if response.status!=GoalStatus.STATUS_SUCCEEDED or response.result.error_code.val!=1:
            raise RuntimeError(f'Execution failed: action={response.status}, MoveIt={response.result.error_code.val}')
        final=model.feedback(states[-1][1])
        if max(abs(final[n]-p) for n,p in zip(trajectory.joint_names,trajectory.points[-1].positions))>.02:
            raise RuntimeError('Controller reported success but latest joint feedback differs from final target')
        print(f'{self.motion_name} execution succeeded; controller result and joint feedback checked. No gripper command.',flush=True)

    def close(self):
        if self.handle is not None and not self.finished:
            try:
                f=self.handle.cancel_goal_async()
                self.wait(f.done,3,error='Cancellation acknowledgement timed out')
                print('Cancel requested; this is not a hardware emergency stop.',flush=True)
            except Exception as exc:
                print(f'Could not confirm cancellation: {exc}; check controller and use physical stop if necessary',flush=True)
                self.disable_software_gates()

    def disable_software_gates(self):
        """Best effort after unconfirmed cancellation; no motor disable/reset."""
        for service in ['/piper_can_gateway/enable_outputs','/piper_trajectory_bridge/enable_execution']:
            client=None
            try:
                client=self.node.create_client(SetBool,service)
                if not client.wait_for_service(timeout_sec=.5):raise RuntimeError('service unavailable')
                future=client.call_async(SetBool.Request(data=False))
                self.wait(future.done,2,error='software gate response timed out')
                response=future.result()
                if response is None or not response.success:raise RuntimeError('closure not confirmed')
                print(f'Software gate closed: {service}; motor enable and clamp target unchanged.',flush=True)
            except Exception as exc:
                print(f'Could not confirm software gate closure {service}: {exc}. Use physical stop if still moving.',flush=True)
            finally:
                if client is not None:self.node.destroy_client(client)


def main():
    from .plan_pregrasp import main as plan
    try:plan(execution=HoverExecution())
    except KeyboardInterrupt:raise SystemExit(130)
