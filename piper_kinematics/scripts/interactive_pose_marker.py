#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class InteractivePoseMarker(Node):
    def __init__(self):
        super().__init__('interactive_pose_marker')
        
        # 参数
        self.marker_name = self.declare_parameter('marker_name', 'target_pose').value
        self.reference_frame = self.declare_parameter('reference_frame', 'base_link').value
        self.initial_position = self.declare_parameter('initial_position', [0.17016839981079102, 0.0, 0.24231570959091187]).value
        self.initial_orientation = self.declare_parameter('initial_orientation', [-8.328101586130288e-08, 0.6931365132331848, 4.590471613941105e-10, 0.7208064198493958]).value
        self.marker_scale = self.declare_parameter('marker_scale', 0.2).value
        self.pose_topic = self.declare_parameter('pose_topic', '/target_pose').value
        
        if len(self.initial_position) != 3 or not all(math.isfinite(v) for v in self.initial_position):
            raise ValueError('initial_position must contain three finite values')
        if len(self.initial_orientation) != 4 or not all(math.isfinite(v) for v in self.initial_orientation):
            raise ValueError('initial_orientation must contain four finite values')
        norm = math.sqrt(sum(v * v for v in self.initial_orientation))
        if norm < 1e-12 or not math.isfinite(self.marker_scale) or self.marker_scale <= 0:
            raise ValueError('orientation must be nonzero and marker_scale positive')
        self.initial_orientation = [v / norm for v in self.initial_orientation]

        # 创建交互标记服务器
        self.server = InteractiveMarkerServer(self, "interactive_marker")
        
        # 创建位姿发布者
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        
        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 初始化标记
        self.create_marker()
        self.pose_initialized = not self.declare_parameter('sync_initial_pose', True).value
        self.current_pose_sub = self.create_subscription(
            PoseStamped, '/ik/current_pose', self.initialize_pose, 10)
        
        self.get_logger().info(f"Interactive pose marker {self.marker_name!r} is ready in frame {self.reference_frame!r}")
    
    def initialize_pose(self, msg):
        if self.pose_initialized or msg.header.frame_id != self.reference_frame:
            return
        self.server.setPose(self.marker_name, msg.pose, msg.header)
        self.server.applyChanges()
        self.pose_initialized = True

    def create_marker(self):
        """创建一个6自由度交互式标记"""
        # 创建交互标记
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.reference_frame
        int_marker.header.stamp = self.get_clock().now().to_msg()
        int_marker.name = self.marker_name
        int_marker.description = f"{self.marker_name} (6DOF Control)"
        int_marker.pose.position.x = self.initial_position[0]
        int_marker.pose.position.y = self.initial_position[1]
        int_marker.pose.position.z = self.initial_position[2]
        int_marker.pose.orientation.x = self.initial_orientation[0]
        int_marker.pose.orientation.y = self.initial_orientation[1]
        int_marker.pose.orientation.z = self.initial_orientation[2]
        int_marker.pose.orientation.w = self.initial_orientation[3]
        int_marker.scale = self.marker_scale

        # 添加6自由度控制
        self.add_6dof_control(int_marker)
        
        # 添加到服务器
        self.server.insert(int_marker, feedback_callback=self.marker_feedback_cb)
        self.server.applyChanges()
    
    def add_6dof_control(self, int_marker):
        """为标记添加6自由度控制"""
        # 旋转控制
        for axis in ['x', 'y', 'z']:
            control = self.make_arrow_control(axis, mode=InteractiveMarkerControl.ROTATE_AXIS)
            int_marker.controls.append(control)
        
        # 平移控制
        for axis in ['x', 'y', 'z']:
            control = self.make_arrow_control(axis, mode=InteractiveMarkerControl.MOVE_AXIS)
            int_marker.controls.append(control)
    
    def make_arrow_control(self, axis, mode):
        """创建箭头控制"""
        control = InteractiveMarkerControl()
        
        if axis == 'x':
            control.orientation.w = 0.7071
            control.orientation.x = 0.7071
            control.orientation.y = 0.0
            control.orientation.z = 0.0
            control.name = f"rotate_{axis}" if mode == InteractiveMarkerControl.ROTATE_AXIS else f"move_{axis}"
        elif axis == 'y':
            control.orientation.w = 0.7071
            control.orientation.x = 0.0
            control.orientation.y = 0.0
            control.orientation.z = 0.7071
            control.name = f"rotate_{axis}" if mode == InteractiveMarkerControl.ROTATE_AXIS else f"move_{axis}"
        elif axis == 'z':
            control.orientation.w = 0.7071
            control.orientation.x = 0.0
            control.orientation.y = 0.7071
            control.orientation.z = 0.0
            control.name = f"rotate_{axis}" if mode == InteractiveMarkerControl.ROTATE_AXIS else f"move_{axis}"
        
        control.interaction_mode = mode
        control.always_visible = True
        return control
    
    def marker_feedback_cb(self, feedback):
        """标记移动时的回调函数"""
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return
        self.pose_initialized = True
        # 发布位姿
        pose_msg = PoseStamped()
        pose_msg.header = feedback.header
        pose_msg.pose = feedback.pose
        self.pose_pub.publish(pose_msg)
        
        transform = TransformStamped()
        transform.header.frame_id = feedback.header.frame_id
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.child_frame_id = feedback.marker_name
        transform.transform.translation.x = feedback.pose.position.x
        transform.transform.translation.y = feedback.pose.position.y
        transform.transform.translation.z = feedback.pose.position.z
        transform.transform.rotation = feedback.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = InteractivePoseMarker()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.server.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
