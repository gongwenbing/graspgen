"""Scoped target cylinder; only fingers and support table may contact it."""
import copy
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (PlanningScene, PlanningSceneComponents, CollisionObject,
                            AttachedCollisionObject, AllowedCollisionEntry)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene

OBJECT_ID = 'graspgen_target_bottle'


def allow_pairs(matrix, enabled):
    result = copy.deepcopy(matrix)
    count = len(result.entry_names)
    if len(result.entry_values) != count or any(len(r.enabled) != count for r in result.entry_values):
        raise ValueError('Invalid collision matrix')
    names = [OBJECT_ID, 'link7', 'link8', 'graspgen_table', 'graspgen_base_height_guard']
    for name in names:
        if name not in result.entry_names:
            result.entry_names.append(name)
            for row in result.entry_values:row.enabled.append(False)
            result.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(result.entry_names)))
    a = result.entry_names.index(OBJECT_ID)
    for name in names[1:]:
        b = result.entry_names.index(name)
        result.entry_values[a].enabled[b] = bool(enabled and (name in ('graspgen_table','graspgen_base_height_guard') or enabled == 'contact'))
        result.entry_values[b].enabled[a] = result.entry_values[a].enabled[b]
    return result


class BottleScene:
    def __init__(self, runtime, center, diameter, height, table_z=0., source_scene_points=None):
        self.rt = runtime
        self.apply_client = runtime.node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not self.apply_client.wait_for_service(timeout_sec=5):raise RuntimeError('Scene apply service unavailable')
        self.center = np.array([center[0], center[1], table_z+height/2])
        self.table_z=float(table_z)
        self.diameter = diameter; self.height = height
        self.source_center=self.center.copy();self.original_world=None
        # Do not replace an object retained after a prior possibly-held grasp.
        scene = self.read()
        from .table_geometry import verify_table_object
        tables=[o for o in scene.world.collision_objects if o.id=='graspgen_table']
        if len(tables)!=1:raise ValueError('Missing table')
        verify_table_object(tables[0],self.table_z)
        if self.table_z<0:
            guards=[o for o in scene.world.collision_objects if o.id=='graspgen_base_height_guard']
            if len(guards)!=1:raise ValueError('Re-run add_table to install conservative base-height guard')
            verify_table_object(guards[0],0.)
        if any(o.object.id == OBJECT_ID for o in scene.robot_state.attached_collision_objects):
            raise ValueError('A possibly held bottle is still attached; inspect before a new grasp')
        existing=[o for o in scene.world.collision_objects if o.id==OBJECT_ID]
        if existing:
            obj=existing[0];q=obj.pose.orientation;p=obj.pose.position
            quat=[q.x,q.y,q.z,q.w]
            rotation=Rotation.from_quat(quat) if np.linalg.norm(quat)>1e-8 else Rotation.identity()
            local=obj.primitive_poses[0].position
            center=rotation.apply([local.x,local.y,local.z])+[p.x,p.y,p.z]
            if np.linalg.norm(center[:2]-self.center[:2])>.03:
                from .table_placement import old_bottle_site_is_clear
                points=source_scene_points() if source_scene_points is not None else []
                if obj.header.frame_id!='base_link' or not old_bottle_site_is_clear(points,center[:2],self.table_z,self.diameter):
                    raise ValueError('Old bottle location occupied or not fully observed; show old position and new bottle together to camera before retrying')
                print('Previous bottle location verified empty from matching scene cloud; updating selected bottle model',flush=True)
                # The old position is visibly empty; never restore its stale model.
            else:
                self.original_world=copy.deepcopy(obj)
        self.active = False

    def read(self):
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        return self.rt.call(self.rt.runner.scene, req).scene

    def object(self):
        obj = CollisionObject(); obj.id = OBJECT_ID; obj.header.frame_id = 'base_link'; obj.pose.orientation.w=1.; obj.operation = CollisionObject.ADD
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[self.height, self.diameter/2+.002])]
        pose = Pose(); pose.position.x, pose.position.y, pose.position.z = map(float, self.center); pose.orientation.w=1.
        obj.primitive_poses = [pose]
        return obj

    def apply(self, scene):
        scene.is_diff = True; scene.robot_state.is_diff = True
        if not self.rt.call(self.apply_client, ApplyPlanningScene.Request(scene=scene)).success:
            raise RuntimeError('Target collision scene update rejected')

    def world(self, contact=False):
        scene = PlanningScene(); scene.world.collision_objects = [self.object()]
        attached = AttachedCollisionObject(); attached.link_name='link6'; attached.object.id=OBJECT_ID
        attached.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects=([attached] if any(o.object.id==OBJECT_ID for o in self.read().robot_state.attached_collision_objects) else [])
        scene.allowed_collision_matrix=allow_pairs(self.read().allowed_collision_matrix, 'contact' if contact else 'support')
        self.apply(scene); self.active = True
        readback = self.read()
        if not any(o.id == OBJECT_ID for o in readback.world.collision_objects):
            raise RuntimeError('Target world object not confirmed')

    def source(self):
        self.center=self.source_center.copy();self.world()

    def placed(self,xy,contact=False):
        self.center=np.array([xy[0],xy[1],self.table_z+self.height/2]);self.world(contact=contact)

    def contacts(self):
        scene=PlanningScene();scene.allowed_collision_matrix=allow_pairs(self.read().allowed_collision_matrix,'contact');self.apply(scene)

    def attach(self, flange_pose):
        p=flange_pose.position; q=flange_pose.orientation
        rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]); translation=np.array([p.x,p.y,p.z])
        obj=self.object();obj.header.frame_id='link6'
        local=rotation.inv().apply(self.center-translation)
        pose=obj.primitive_poses[0];pose.position.x,pose.position.y,pose.position.z=map(float,local)
        pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=map(float,rotation.inv().as_quat())
        attached=AttachedCollisionObject(link_name='link6',object=obj,touch_links=['link7','link8'])
        removal=CollisionObject(id=OBJECT_ID,operation=CollisionObject.REMOVE)
        scene=PlanningScene() # Adding an attachment atomically removes its world object.
        scene.robot_state.attached_collision_objects=[attached]
        scene.allowed_collision_matrix=allow_pairs(self.read().allowed_collision_matrix,'contact')
        self.apply(scene)
        if not any(o.object.id == OBJECT_ID for o in self.read().robot_state.attached_collision_objects):
            raise RuntimeError('Target attachment not confirmed')

    def attachment(self):
        found=[o for o in self.read().robot_state.attached_collision_objects if o.object.id==OBJECT_ID]
        if len(found)!=1:raise RuntimeError('Held bottle attachment missing')
        return copy.deepcopy(found[0])

    def restore_attachment(self,attached):
        scene=PlanningScene()
        scene.robot_state.attached_collision_objects=[copy.deepcopy(attached)]
        scene.allowed_collision_matrix=allow_pairs(self.read().allowed_collision_matrix,'contact')
        self.apply(scene)

    def cleanup(self):
        if not self.active:return
        scene=PlanningScene();scene.world.collision_objects=[copy.deepcopy(self.original_world) if self.original_world else CollisionObject(id=OBJECT_ID,operation=CollisionObject.REMOVE)]
        attached=AttachedCollisionObject(link_name='link6');attached.object.id=OBJECT_ID;attached.object.operation=CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects=([attached] if any(o.object.id==OBJECT_ID for o in self.read().robot_state.attached_collision_objects) else [])
        scene.allowed_collision_matrix=allow_pairs(self.read().allowed_collision_matrix,'support' if self.original_world else False)
        self.apply(scene)
