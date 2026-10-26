"""Add a conservative tabletop to MoveIt; sends no hardware commands."""
import time
import rclpy
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from .scene_contacts import allow_table_mount


def main():
    rclpy.init()
    node = rclpy.create_node('graspgen_add_table')
    def call(service_type, name, request):
        client = node.create_client(service_type, name)
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError(f'{name} unavailable; start MoveIt first')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'{name} timed out')
        return future.result()
    try:
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = 'graspgen_table'
        obj.operation = CollisionObject.ADD
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[2.0, 2.0, 0.05])
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.z = -0.025
        obj.primitives = [box]
        obj.primitive_poses = [pose]
        acm_query=GetPlanningScene.Request()
        acm_query.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        existing=call(GetPlanningScene,'/get_planning_scene',acm_query).scene.allowed_collision_matrix
        scene = PlanningScene()
        scene.allowed_collision_matrix=allow_table_mount(existing)
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [obj]
        request = ApplyPlanningScene.Request(scene=scene)
        if not call(ApplyPlanningScene, '/apply_planning_scene', request).success:
            raise RuntimeError('MoveIt rejected table scene')
        query = GetPlanningScene.Request()
        query.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY|PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        result = call(GetPlanningScene, '/get_planning_scene', query)
        tables = [o for o in result.scene.world.collision_objects if o.id == obj.id]
        if len(tables) != 1 or not tables[0].primitives:
            raise RuntimeError('Table absent from MoveIt scene readback')
        acm=result.scene.allowed_collision_matrix
        i=acm.entry_names.index('base_link');j=acm.entry_names.index('graspgen_table')
        if not acm.entry_values[i].enabled[j] or not acm.entry_values[j].enabled[i]:
            raise RuntimeError('Fixed base/table mounting contact exemption missing')
        print('仅允许固定基座base_link与桌面的安装接触；其他连杆和夹指仍检查碰撞。')
        print('桌面已加入并读回确认：base_link，2×2米保守覆盖，顶面Z=0。')
        print('未发送运动指令。MoveIt重启后需重新运行；本命令不自动添加相机支架。')
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
