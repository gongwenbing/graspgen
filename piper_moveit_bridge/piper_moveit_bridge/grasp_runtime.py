from .speed_settings import APPROACH_VELOCITY_SCALE,APPROACH_ACCELERATION_SCALE,ARM_EXECUTION_CAP,CARTESIAN_LINEAR_SPEED,GRIPPER_OPEN_SPEED,GRIPPER_CLOSE_SPEED,GRIPPER_MIN_DURATION
"""Shared ROS helpers. Construction and planning never send actuator targets."""
import copy
import hashlib
import json
import time
from collections import deque
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import rclpy
from sensor_msgs.msg import JointState,PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from moveit_msgs.msg import DisplayTrajectory,Constraints,PositionConstraint,OrientationConstraint,BoundingVolume,RobotState
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose,PoseStamped
from moveit_msgs.srv import GetPositionFK,GetMotionPlan,GetCartesianPath,GetStateValidity,GetPositionIK
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from .grasp_contact import contact_matches
from .gripper_timing import gripper_points
from trajectory_msgs.msg import JointTrajectoryPoint
from .model import ARM,Model,validate_trajectory
from .execute_pregrasp import HoverExecution
from .cartesian_step import retime,check_translation
from .grasp_calibration import pose_array
from ament_index_python.packages import get_package_share_directory

ROOT=Path.home()/'GraspGen'


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomic_json(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(path.suffix+'.tmp');temporary.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n');temporary.replace(path)


class Runtime:
    def __init__(self,node,execute=False):
        self.node=node;self.execute=execute;self.states=deque(maxlen=1);self.hardware=deque(maxlen=1);self.ready=deque(maxlen=1)
        self.subs=[]
        self.scene_clouds=deque(maxlen=10)
        self.tf=Buffer();self.tf_listener=TransformListener(self.tf,node)
        self.subs.append(node.create_subscription(PointCloud2,'/graspgen/scene_cloud',lambda m:self.scene_clouds.append((time.monotonic(),m)),2))
        self.preview=deque(maxlen=1);self.vision=deque(maxlen=1)
        for kind,topic,dst in [(JointState,'/joint_states',self.states),(String,'/piper_hardware/status',self.hardware),
             (String,'/piper_bridge/readiness',self.ready),(String,'/grasp_preview/status',self.preview),(String,'/graspgen/status',self.vision)]:
            self.subs.append(node.create_subscription(kind,topic,lambda m,dst=dst:dst.append((time.monotonic(),m)),1))
        self.model_path=Path(get_package_share_directory('piper_description'))/'urdf/piper_description.urdf';self.model=Model(self.model_path)
        self.runner=HoverExecution();self.runner.motion_name='Automatic local grasp'
        self.fk=node.create_client(GetPositionFK,'/compute_fk')
        self.cart=node.create_client(GetCartesianPath,'/compute_cartesian_path')
        self.ik=node.create_client(GetPositionIK,'/compute_ik')
        self.motion=node.create_client(GetMotionPlan,'/plan_kinematic_path')
        self.valid=node.create_client(GetStateValidity,'/check_state_validity')
        self.runner.prepare(node,self.wait,need_action=execute)
        for c in [self.fk,self.cart,self.motion,self.valid,self.ik]:
            if not c.wait_for_service(timeout_sec=8):raise RuntimeError('MoveIt planning service missing')
        until=time.monotonic()+3;self.wait(lambda:time.monotonic()>=until,5)
        self.wait(lambda:all([self.states,self.hardware,self.ready]))
        self.gripper_handle=None;self.gripper_finished=False
        self.gripper=ActionClient(node,FollowJointTrajectory,'/gripper_controller/follow_joint_trajectory') if execute else None
        self.display=node.create_publisher(DisplayTrajectory,'/display_planned_path',10)

    def wait(self,predicate,timeout=15,error='ROS operation timed out'):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(self.node,timeout_sec=.01)
            if predicate():return
        raise RuntimeError(error)

    def call(self,client,request):
        f=client.call_async(request);self.wait(f.done,25);response=f.result()
        if response is None:raise RuntimeError('Empty MoveIt service result')
        return response

    def inputs(self):
        if self.node.count_publishers('/piper_hardware/status')!=1 or self.node.count_publishers('/piper_bridge/readiness')!=1:
            raise ValueError('Require exactly one gateway and bridge')
        if any(not q or time.monotonic()-q[-1][0]>.3 for q in [self.states,self.hardware,self.ready]):raise ValueError('Stale feedback')
        js=self.states[-1][1];stamp=js.header.stamp.sec+js.header.stamp.nanosec*1e-9
        if not -.1<=self.node.get_clock().now().nanoseconds*1e-9-stamp<=.3:raise ValueError('Stale joint timestamp')
        h=json.loads(self.hardware[-1][1].data);b=json.loads(self.ready[-1][1].data)
        if h.get('reason') or h.get('ctrl_mode') not in (0,1) or h.get('teach_status')!=0 or h.get('err_code')!=0:
            raise ValueError('Hardware not healthy: '+json.dumps(h))
        if b.get('active'):raise ValueError('Trajectory bridge still reports an active action')
        if self.execute:self.runner.check_hardware(h,b)
        return copy.deepcopy(js),self.model.feedback(js)

    def pose(self,js):
        req=GetPositionFK.Request();req.header.frame_id='base_link';req.fk_link_names=['link6'];req.robot_state.joint_state=js
        r=self.call(self.fk,req)
        if r.error_code.val!=1 or len(r.pose_stamped)!=1:raise ValueError('FK failed')
        return r.pose_stamped[0].pose

    def candidate(self):
        if any(not q or time.monotonic()-q[-1][0]>.5 for q in [self.preview,self.vision]):raise ValueError('No fresh visual target')
        p=json.loads(self.preview[-1][1].data);v=json.loads(self.vision[-1][1].data)
        if not v.get('valid'):
            detail='; '.join(str(o.get('reason')) for o in v.get('objects',[]) if o.get('reason'))
            raise ValueError('Vision: '+(detail or v.get('reason','Invalid visual target')))
        if not p.get('target_valid'):raise ValueError('Preview: '+p.get('reason','Invalid target'))
        now=self.node.get_clock().now().nanoseconds*1e-9
        if not -.1<=now-v.get('stamp_sec',0)<=.5:raise ValueError('Stale image timestamp')
        objects=[o for o in v.get('objects',[]) if o.get('id')==v.get('selected_id') and o.get('valid')]
        if len(objects)!=1:raise ValueError('Missing selected object')
        return p,v,objects[0]

    def state_after(self,start,trajectory):
        result=copy.deepcopy(start);values=dict(zip(trajectory.joint_names,trajectory.points[-1].positions))
        result.position=[values.get(n,p) for n,p in zip(result.name,result.position)]
        return result

    def validate(self,trajectory,start):
        if trajectory.multi_dof_joint_trajectory.points:raise ValueError('Unexpected multi-DOF trajectory')
        return validate_trajectory(trajectory.joint_trajectory,ARM,self.model.feedback(start),self.model,ARM_EXECUTION_CAP,.005,.002)

    def cart_plan(self,start,target):
        origin=self.pose(start);delta=np.array([target.position.x-origin.position.x,target.position.y-origin.position.y,target.position.z-origin.position.z])
        req=GetCartesianPath.Request();req.header.frame_id='base_link';req.start_state.joint_state=start;req.start_state.is_diff=True
        req.group_name='arm';req.link_name='link6';req.waypoints=[target];req.max_step=.001;req.jump_threshold=2.;req.revolute_jump_threshold=.05
        req.avoid_collisions=True;req.max_velocity_scaling_factor=.05;req.max_acceleration_scaling_factor=.05
        r=self.call(self.cart,req)
        if r.error_code.val!=1 or r.fraction<.999999:raise ValueError(f'Incomplete Cartesian path: {r.fraction:.3f}/{r.error_code.val}')
        retime(r.solution.joint_trajectory,self.model.feedback(start),np.linalg.norm(delta)/CARTESIAN_LINEAR_SPEED)
        self.validate(r.solution,start)
        for pt in r.solution.joint_trajectory.points:
            js=copy.deepcopy(start);values=dict(zip(r.solution.joint_trajectory.joint_names,pt.positions))
            js.position=[values.get(n,p) for n,p in zip(js.name,js.position)]
            check_translation(origin,self.pose(js),delta)
        end=self.pose(self.state_after(start,r.solution.joint_trajectory))
        if np.linalg.norm(pose_array(end)[0]-pose_array(target)[0])>.001:raise ValueError('Cartesian endpoint mismatch')
        return r.solution

    def scene_points_at(self,stamp):
        matches=[m for received,m in self.scene_clouds if abs(m.header.stamp.sec+m.header.stamp.nanosec*1e-9-stamp)<1e-6 and time.monotonic()-received<1.5]
        if not matches:raise ValueError('No matching full scene cloud for random placement; restart updated vision')
        cloud=matches[-1]
        self.wait(lambda:self.tf.can_transform('base_link',cloud.header.frame_id,Time.from_msg(cloud.header.stamp)),1,error='Scene image-time TF unavailable')
        tf=self.tf.lookup_transform('base_link',cloud.header.frame_id,Time.from_msg(cloud.header.stamp))
        q=tf.transform.rotation;t=tf.transform.translation
        rows=list(point_cloud2.read_points(cloud,field_names=('x','y','z'),skip_nans=True))
        points=np.asarray([list(p) for p in rows],dtype=float)
        if points.ndim!=2 or points.shape[1]!=3:raise ValueError('Empty full scene cloud')
        return Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(points)+[t.x,t.y,t.z]

    def pose_ik(self,start,target):
        req=GetPositionIK.Request();q=req.ik_request;q.group_name='arm';q.ik_link_name='link6'
        q.robot_state.joint_state=start;q.robot_state.is_diff=True;q.avoid_collisions=True
        q.pose_stamped=PoseStamped(pose=copy.deepcopy(target));q.pose_stamped.header.frame_id='base_link'
        q.timeout.sec=0;q.timeout.nanosec=200000000
        r=self.call(self.ik,req)
        return r.error_code.val==1,r.error_code.val

    def hover_plan(self,start,target):
        return self.pose_plan(start,target,label='Hover')

    def pose_plan(self,start,target,label='Direct grasp',keep_orientation=False):
        req=GetMotionPlan.Request();m=req.motion_plan_request;m.group_name='arm';m.start_state.joint_state=start;m.start_state.is_diff=True
        m.allowed_planning_time=10.;m.num_planning_attempts=3;m.max_velocity_scaling_factor=APPROACH_VELOCITY_SCALE;m.max_acceleration_scaling_factor=APPROACH_ACCELERATION_SCALE
        c=Constraints();pc=PositionConstraint();pc.header.frame_id='base_link';pc.link_name='link6';pc.weight=1.
        pc.constraint_region=BoundingVolume(primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[.001])],primitive_poses=[copy.deepcopy(target)])
        oc=OrientationConstraint();oc.header.frame_id='base_link';oc.link_name='link6';oc.orientation=target.orientation;oc.weight=1.
        oc.absolute_x_axis_tolerance=.005;oc.absolute_y_axis_tolerance=.005;oc.absolute_z_axis_tolerance=.005
        c.position_constraints=[pc];c.orientation_constraints=[oc];m.goal_constraints=[c]
        if keep_orientation:
            upright=copy.deepcopy(oc);upright.absolute_x_axis_tolerance=.04;upright.absolute_y_axis_tolerance=.04;upright.absolute_z_axis_tolerance=.04
            m.path_constraints=Constraints(orientation_constraints=[upright])
        for attempt in range(3):
            r=self.call(self.motion,req).motion_plan_response
            if r.error_code.val!=-2:break
            if attempt<2:
                print(f'{label}: MoveIt rejected invalid/colliding path; retry {attempt+2}/3 with same target and collision checks',flush=True)
        if r.error_code.val!=1:
            detail=' (invalid path; inspect MoveIt collision contacts)' if r.error_code.val==-2 else ''
            raise ValueError(f'{label} planning failed: {r.error_code.val}'+detail)
        # Uniformly slow the MoveIt timing to the same limits enforced by the
        # bridge, including supplied derivatives and each segment's mean speed.
        trajectory=r.trajectory.joint_trajectory
        previous=self.model.feedback(start);last_time=0.;factor=1.
        for point in trajectory.points:
            stamp=point.time_from_start.sec+point.time_from_start.nanosec*1e-9
            for i,name in enumerate(trajectory.joint_names):
                cap=min(ARM_EXECUTION_CAP,self.model.limits[name][2])
                if point.velocities:factor=max(factor,abs(point.velocities[i])/cap)
                if stamp>last_time:factor=max(factor,abs(point.positions[i]-previous[name])/((stamp-last_time)*cap))
                previous[name]=point.positions[i]
            last_time=stamp
        if factor>1.:
            factor*=1.001
            for point in trajectory.points:
                ns=round((point.time_from_start.sec*1e9+point.time_from_start.nanosec)*factor)
                point.time_from_start.sec,point.time_from_start.nanosec=divmod(ns,1000000000)
                point.velocities=[v/factor for v in point.velocities]
                point.accelerations=[a/factor**2 for a in point.accelerations]
        self.validate(r.trajectory,start)
        final=self.pose(self.state_after(start,r.trajectory.joint_trajectory));a,ra=pose_array(final);b,rb=pose_array(target)
        if np.linalg.norm(a-b)>.002 or (ra.inv()*rb).magnitude()>.015:raise ValueError(f'{label} endpoint mismatch')
        return r.trajectory

    def valid_state(self,js):
        req=GetStateValidity.Request();req.robot_state.joint_state=js;req.robot_state.is_diff=True;req.group_name=''
        result=self.call(self.valid,req)
        if not result.valid:
            pairs=sorted({f'{c.contact_body_1} <-> {c.contact_body_2}' for c in result.contacts})
            width=dict(zip(js.name,js.position)).get('joint7',float('nan'))*2000
            raise ValueError(f'Predicted arm/gripper state is in collision at opening={width:.2f}mm: '+('; '.join(pairs) or 'no contact details returned'))

    def show(self,start,trajectories):
        d=DisplayTrajectory();d.model_id='piper';d.trajectory_start.joint_state=start;d.trajectory=trajectories
        for _ in range(3):
            self.display.publish(d);until=time.monotonic()+.2;self.wait(lambda:time.monotonic()>=until,2)

    def wait_idle_after_result(self):
        # An action result can arrive before the 10Hz readiness timer publishes
        # active=false. Require NEW feedback after the result, not an old idle flag.
        completed=time.monotonic()
        def idle():
            queues=[self.hardware,self.ready,self.states]
            if any(not q or q[-1][0]<completed for q in queues):return False
            return not json.loads(self.ready[-1][1].data).get('active',True)
        self.wait(idle,3,error='Action completed but fresh idle feedback not confirmed; no next stage sent')
        self.inputs()

    def arm(self,trajectory):
        self.runner.handle=None;self.runner.finished=False
        self.runner.execute_arm_plan(self.model,SimpleNamespace(trajectory=trajectory),self.hardware,self.ready,self.states)
        self.wait_idle_after_result()

    def close_gripper(self,target,contact=None):
        result=self._move_gripper(target,contact=contact,closing=True)
        if contact is not None:self.runner.contact_guard=lambda h:contact_matches(h,contact)
        return result

    def open_gripper(self,target):
        result=self._move_gripper(target,closing=False)
        self.runner.contact_guard=None
        return result

    def _move_gripper(self,target,contact=None,closing=True):
        start,q=self.inputs();opening=q['joint7']*2
        if not .005<=target<=.07 or not 0<=opening<=.07:raise ValueError('Invalid gripper target/current opening')
        if closing and not target<opening:raise ValueError('Invalid close target/current opening')
        if not closing and not target>opening:raise ValueError('Invalid open target/current opening')
        if not self.gripper.wait_for_server(timeout_sec=5):raise RuntimeError('Gripper action unavailable')
        goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=['joint7']
        requested_speed=GRIPPER_CLOSE_SPEED if closing else GRIPPER_OPEN_SPEED
        points,duration,effective=gripper_points(opening,target,requested_speed,GRIPPER_MIN_DURATION,
                                               2*self.model.limits['joint7'][2])
        goal.trajectory.points=points
        print(f"Gripper {'close' if closing else 'open'}: {opening*1000:.1f} -> {target*1000:.1f}mm; "
              f"duration={duration:.3f}s, speed cap={effective*1000:.1f}mm/s",flush=True)
        if effective<requested_speed:
            print('Requested gripper speed exceeds configured pipeline cap; using bounded timing',flush=True)
        if contact is not None:
            gap=contact['observed_opening_m']-target
            if not .001<gap<=.005 or abs(contact['command_opening_m']-target)>1e-6:
                raise ValueError('Invalid taught contact gap')
            if not all(k in json.loads(self.hardware[-1][1].data) for k in ['gripper_effort_nm','gripper_enabled','gripper_fault','gripper_feedback_age_sec']):
                raise ValueError('Gateway lacks contact telemetry; restart updated gateway before an automatic grasp')
            h=json.loads(self.hardware[-1][1].data);bstate=json.loads(self.ready[-1][1].data)
            if h.get('gripper_fault') or not h.get('gripper_enabled') or not bstate.get('gripper_contact_tolerance_supported'):
                raise ValueError('Require enabled healthy gripper and updated contact-aware bridge')
            goal.goal_tolerance=[JointTolerance(name='joint7',position=min(.003,(gap+.001)/2))]
        self.gripper_finished=False;self.gripper_handle=None
        f=self.gripper.send_goal_async(goal);self.wait(f.done,8);self.gripper_handle=f.result()
        if not self.gripper_handle.accepted:raise RuntimeError('Gripper goal rejected')
        f=self.gripper_handle.get_result_async();end=time.monotonic()+duration+12
        while not f.done():
            self.wait(lambda:True,.2)
            if time.monotonic()>end:raise RuntimeError('Gripper timeout; no lift')
            if any(not q or time.monotonic()-q[-1][0]>.5 for q in [self.hardware,self.ready,self.states]):raise RuntimeError('Feedback lost while closing; no lift')
            self.runner.check_hardware(json.loads(self.hardware[-1][1].data),json.loads(self.ready[-1][1].data),active_allowed=True)
        r=f.result();self.gripper_finished=True
        if r.status!=4 or r.result.error_code!=0:raise RuntimeError(f'Gripper failed: {r.result.error_string}; no lift')
        self.wait_idle_after_result()
        if contact is None:
            if abs(self.model.feedback(self.states[-1][1])['joint7']*2-target)>.001:
                raise RuntimeError('Gripper opening not confirmed; no lift')
        else:
            seen=[];last_received=-1.
            def settled_contact():
                nonlocal last_received
                self.inputs()
                received=self.hardware[-1][0]
                if received==last_received:return False
                last_received=received;h=json.loads(self.hardware[-1][1].data)
                if not contact_matches(h,contact):seen.clear();return False
                seen.append((time.monotonic(),h['gripper_opening_m']))
                while len(seen)>1 and seen[-1][0]-seen[0][0]>.7:seen.pop(0)
                return len(seen)>=5 and seen[-1][0]-seen[0][0]>=.3 and max(w for _,w in seen)-min(w for _,w in seen)<.001
            self.wait(settled_contact,3,error='Taught contact position/effort not stable; no lift')
        actual=self.model.feedback(self.states[-1][1]);original=self.model.feedback(start)
        if max(abs(actual[n]-original[n]) for n in ARM)>.002:raise RuntimeError('Arm drifted while closing; no lift')

    def close(self):
        if self.gripper_handle is not None and not self.gripper_finished:
            try:
                f=self.gripper_handle.cancel_goal_async();self.wait(f.done,3)
            except Exception as exc:print(f'Gripper cancellation not confirmed: {exc}')
        self.runner.close()

