from .speed_settings import GRIPPER_OPENING_CAP
import json
import math
import threading
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectoryPoint

from .model import ARM, JOINTS, Model, interpolate, seconds, validate_trajectory


def joint_command(whole,names,stamp):
    msg=JointState();msg.header.stamp=stamp
    msg.name=list(ARM);msg.position=[whole[n] for n in ARM]
    if names==('joint7',):
        msg.name.append('gripper');msg.position.append(2*whole['joint7'])
    return msg


def bounded_goal_tolerances(names,jaw_tolerance,arm_tolerance,requested):
    limits={n:(jaw_tolerance if n=='joint7' else arm_tolerance) for n in names}
    for tolerance in requested:
        if tolerance.position:
            cap=.003 if tolerance.name=='joint7' else limits[tolerance.name]
            limits[tolerance.name]=min(cap,tolerance.position)
    return limits


class TrajectoryBridge(Node):
    def __init__(self):
        super().__init__('piper_trajectory_bridge')
        self.model = Model(self.declare_parameter('model_path', str(Path(get_package_share_directory('piper_description')) / 'urdf/piper_description.urdf')).value)
        self.timeout = self.declare_parameter('feedback_timeout', 0.3).value
        self.arm_speed = self.declare_parameter('max_arm_velocity', 0.2).value
        self.jaw_speed = self.declare_parameter('max_jaw_velocity', GRIPPER_OPENING_CAP/2).value
        self.start_tolerance = self.declare_parameter('start_tolerance', 0.02).value
        self.goal_tolerance = self.declare_parameter('goal_tolerance', 0.01).value
        self.jaw_tolerance = self.declare_parameter('jaw_tolerance', 0.0005).value
        for value in (self.timeout,self.arm_speed,self.jaw_speed,self.start_tolerance,self.goal_tolerance,self.jaw_tolerance):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Bridge limits/timeouts must be positive and finite')
        self.lock = threading.RLock()
        self.measured = None
        self.feedback_at = self.gateway_at = 0.0
        self.gateway = self.armed = self.busy = False
        self.feedback_error = 'No joint feedback'
        self.group = ReentrantCallbackGroup()
        self.command_pub = self.create_publisher(JointState, '/moveit/joint_commands', 10)
        self.visual_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.readiness_pub = self.create_publisher(String, '/piper_bridge/readiness', 10)
        self.create_subscription(JointState, '/joint_states_single', self.feedback, rclpy.qos.qos_profile_sensor_data, callback_group=self.group)
        self.create_subscription(Bool, '/piper_hardware/outputs_enabled', self.gateway_feedback, 10, callback_group=self.group)
        self.create_service(SetBool, '~/enable_execution', self.enable, callback_group=self.group)
        self.create_timer(0.1,self.watchdog,callback_group=self.group)
        self.servers = [ActionServer(self, FollowJointTrajectory, path,
            execute_callback=lambda handle, names=names: self.execute(handle,names),
            goal_callback=lambda goal, names=names: self.accept(goal,names),
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group)
            for path,names in [('/arm_controller/follow_joint_trajectory',ARM),('/gripper_controller/follow_joint_trajectory',('joint7',))]]

    def feedback(self,msg):
        with self.lock:
            try:
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
                age = self.get_clock().now().nanoseconds*1e-9-stamp
                if stamp <= 0 or not -0.1 <= age <= self.timeout:
                    raise ValueError('Stale feedback timestamp')
                # Preserve invalid readings for diagnostics; valid boundary-noise
                # readings use the same normalized state as trajectory validation.
                values = dict(zip(msg.name,msg.position))
                if len(msg.name) != len(msg.position) or len(values) != len(msg.name):
                    raise ValueError('Feedback name mismatch')
                raw = [values[n] for n in ARM]
                width = values.get('gripper',2*values.get('joint7',float('nan')))
                if not all(math.isfinite(v) for v in raw+[width]):
                    raise ValueError('Nonfinite feedback')
                visual = JointState();visual.header=msg.header
                visual.name=list(JOINTS);visual.position=raw+[max(0.0,min(self.model.max_opening,width))/2]
                try:
                    measured=self.model.feedback(msg)
                except (ValueError,KeyError):
                    self.visual_pub.publish(visual)
                    raise
                visual.position=[measured[n] for n in JOINTS]
                self.visual_pub.publish(visual)
                self.measured=measured
                self.feedback_at=time.monotonic();self.feedback_error=''
            except (ValueError,KeyError) as e:
                self.feedback_error=str(e);self.feedback_at=0.0;self.armed=False

    def gateway_feedback(self,msg):
        with self.lock:
            self.gateway=msg.data;self.gateway_at=time.monotonic()
            if not msg.data:self.armed=False

    def reason(self):
        if time.monotonic()-self.feedback_at > self.timeout:
            return self.feedback_error or 'Joint feedback timeout'
        if not self.gateway or time.monotonic()-self.gateway_at > self.timeout:
            return 'Hardware output gate disabled or stale'
        return ''

    def enable(self,request,response):
        with self.lock:
            if not request.data:
                self.armed=False;response.success=True;response.message='Execution disabled; active action will stop publishing';return response
            error=self.reason()
            if self.busy or error:
                response.success=False;response.message=error or 'Action is active';return response
            self.armed=True;response.success=True;response.message='Execution enabled; no command sent until a new action goal';return response

    def watchdog(self):
        with self.lock:
            error=self.reason()
            if error:self.armed=False
            self.readiness_pub.publish(String(data=json.dumps({'gripper_contact_tolerance_supported':True,'execution_enabled':self.armed,'active':self.busy,'reason':error or 'ready'})))

    def validate(self,goal,names):
        if not self.armed or self.reason():
            raise ValueError(self.reason() or 'Execution not enabled')
        stamp=goal.trajectory.header.stamp.sec+goal.trajectory.header.stamp.nanosec*1e-9
        delay=stamp-self.get_clock().now().nanoseconds*1e-9 if stamp else 0.0
        if delay < -0.1 or delay > 5:
            raise ValueError('Trajectory start time is stale or over five seconds ahead')
        for tolerance in list(goal.path_tolerance)+list(goal.goal_tolerance):
            if tolerance.name not in names or any(not math.isfinite(v) or v < 0 for v in (tolerance.position,tolerance.velocity,tolerance.acceleration)):
                raise ValueError('Invalid/disabled action tolerance')
            if tolerance.velocity or tolerance.acceleration:
                raise ValueError('Velocity/acceleration feedback tolerances are unsupported')
        if seconds(goal.goal_time_tolerance) < 0 or seconds(goal.goal_time_tolerance) > 5:
            raise ValueError('Goal time tolerance must be in [0,5] seconds')
        return validate_trajectory(goal.trajectory,names,self.measured,self.model,self.arm_speed,self.jaw_speed,self.start_tolerance)

    def accept(self,goal,names):
        with self.lock:
            if self.busy:return GoalResponse.REJECT
            try:self.validate(goal,names)
            except (ValueError,KeyError) as e:
                self.get_logger().warning(f'Trajectory rejected: {e}');return GoalResponse.REJECT
            self.busy=True
            return GoalResponse.ACCEPT

    def execute(self,handle,names):
        result=FollowJointTrajectory.Result()
        try:
            with self.lock:
                times,points=self.validate(handle.request,names)
                held=dict(self.measured)
            stamp=handle.request.trajectory.header.stamp.sec+handle.request.trajectory.header.stamp.nanosec*1e-9
            delay=max(0.0,stamp-self.get_clock().now().nanoseconds*1e-9) if stamp else 0.0
            start=time.monotonic()+delay
            finish=start+times[-1]+(seconds(handle.request.goal_time_tolerance) or 3.0)
            # Contact-aware clients may request up to 3mm per-finger endpoint
            # tolerance. They must separately verify live taught-contact evidence.
            goal_limits=bounded_goal_tolerances(names,self.jaw_tolerance,self.goal_tolerance,handle.request.goal_tolerance)
            path_limits={n:(0.005 if n=='joint7' else 0.1) for n in names}
            for t in handle.request.path_tolerance:
                if t.position:path_limits[t.name]=min(path_limits[t.name],t.position)
            while rclpy.ok():
                with self.lock:
                    if handle.is_cancel_requested:
                        self.armed=False;handle.canceled();result.error_string='Canceled; output stopped, not a hardware emergency stop';return result
                    if not self.armed or self.reason():
                        raise RuntimeError(self.reason() or 'Execution gate disabled')
                    elapsed=time.monotonic()-start
                    if elapsed < 0:
                        pass
                    else:
                        positions=interpolate(times,points,elapsed)
                        actual=[self.measured[n] for n in names]
                        if any(abs(a-d)>path_limits[n] for n,a,d in zip(names,actual,positions)):
                            raise RuntimeError('Path tracking tolerance exceeded')
                        if elapsed>=times[-1] and all(abs(a-b)<=goal_limits[n] for n,a,b in zip(names,actual,points[-1])):
                            handle.succeed();result.error_code=result.SUCCESSFUL;return result
                        if time.monotonic()>finish:
                            raise RuntimeError('Goal tracking timeout')
                        whole=dict(held);whole.update(zip(names,positions))
                        # Preserve the previous hardware clamp target for arm-only actions.
                        msg=joint_command(whole,names,self.get_clock().now().to_msg())
                        self.command_pub.publish(msg)
                        feedback=FollowJointTrajectory.Feedback();feedback.header=msg.header;feedback.joint_names=list(names)
                        feedback.desired=JointTrajectoryPoint(positions=positions)
                        feedback.actual=JointTrajectoryPoint(positions=actual)
                        feedback.error=JointTrajectoryPoint(positions=[d-a for d,a in zip(positions,actual)])
                        handle.publish_feedback(feedback)
                time.sleep(0.01)
            raise RuntimeError('ROS shutdown')
        except (ValueError,KeyError,RuntimeError) as e:
            with self.lock:self.armed=False
            result.error_code=result.PATH_TOLERANCE_VIOLATED;result.error_string=str(e)
            if handle.is_active:handle.abort()
            return result
        finally:
            with self.lock:self.busy=False


def main(args=None):
    rclpy.init(args=args);node=TrajectoryBridge();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
