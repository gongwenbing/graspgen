"""Geometry checks for the simulation-only vision planning adapter."""
import math
import sys
from pathlib import Path

import pytest
from geometry_msgs.msg import Pose, PoseStamped

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from plan_vision_pregrasp import bottle_collision, goal_constraints


def test_constraints_preserve_vision_position_and_orientation():
    target = PoseStamped()
    target.header.frame_id = 'base_link'
    target.pose.position.x = .4
    target.pose.position.z = .33
    target.pose.orientation.y = 1.
    target.pose.orientation.w = 0.
    c = goal_constraints(target)
    assert c.position_constraints[0].link_name == 'link6'
    assert c.position_constraints[0].constraint_region.primitive_poses[0].position == target.pose.position
    assert c.orientation_constraints[0].orientation == target.pose.orientation
    target.header.frame_id = 'camera_color_optical_frame'
    with pytest.raises(ValueError, match='base_link'):
        goal_constraints(target)


def test_bottle_compound_collisions_rotate_with_gazebo_link():
    pose = Pose()
    pose.position.x = .4
    pose.orientation.y = math.sqrt(.5)
    pose.orientation.w = math.sqrt(.5)
    obj = bottle_collision('vision_bottle', pose)
    assert obj.header.frame_id == 'world'
    assert len(obj.primitives) == 3
    assert list(obj.primitives[0].dimensions) == pytest.approx([.091, .0182])
    assert list(obj.primitives[2].dimensions) == pytest.approx([.0308, .0091])
    center = obj.primitive_poses[0]
    assert center.position.x == pytest.approx(.4455)
    assert center.position.z == pytest.approx(0.)


def test_invalid_orientation_is_rejected():
    target = PoseStamped()
    target.header.frame_id = 'base_link'
    target.pose.orientation.w = 0.
    with pytest.raises(ValueError, match='normalized'):
        goal_constraints(target)


def test_observation_camera_faces_bottle_with_depth_and_frame_margins():
    import numpy as np
    from observe_bottle import camera_view, bottle_fits
    center = np.array([.35, 0., .094])
    camera = camera_view(center, np.array([-.32, 0., .36]))
    optical = np.linalg.inv(camera) @ [*center, 1.]
    assert optical[:2] == pytest.approx([0., 0.], abs=1e-12)
    assert optical[2] == pytest.approx(math.hypot(.32, .36))
    assert np.linalg.det(camera[:3, :3]) == pytest.approx(1.)
    assert bottle_fits(camera, center)
    too_close = camera_view(center, np.array([-.1, 0., .1]))
    assert not bottle_fits(too_close, center)


def test_upright_bottle_is_preferred_to_inverted_bottle():
    import numpy as np
    from observe_bottle import camera_view, image_pose_cost
    center = np.array([.35, 0., .094])
    upright = camera_view(center, np.array([-.32, 0., .36]))
    inverted = camera_view(center, np.array([-.32, 0., .36]), math.pi)
    costs = image_pose_cost(np.array([upright, inverted]), center)
    assert costs[0] < costs[1]


def test_simulation_tilt_keeps_tcp_position_with_general_offset():
    import numpy as np
    from scipy.spatial.transform import Rotation
    from plan_vision_pregrasp import adjusted_hover
    p = PoseStamped()
    p.header.frame_id = 'base_link'
    p.pose.position.x = .336
    p.pose.position.z = .354
    p.pose.orientation.y = 1.
    p.pose.orientation.w = 0.
    offset = np.array([.002, -.003, .1358])
    before = np.array([.336, 0., .354]) + Rotation.from_quat([0., 1., 0., 0.]).apply(offset)
    result = adjusted_hover(p, offset, 30., 180.)
    q = result.pose.orientation
    r = Rotation.from_quat([q.x, q.y, q.z, q.w])
    xyz = result.pose.position
    after = np.array([xyz.x, xyz.y, xyz.z]) + r.apply(offset)
    assert after == pytest.approx(before, abs=1e-12)
    assert r.apply([0., 0., 1.])[2] == pytest.approx(-math.cos(math.radians(30)))
    with pytest.raises(ValueError, match='0..45'):
        adjusted_hover(p, offset, 46., 0.)


def test_contact_pose_places_tilted_pad_midpoint_on_body_center():
    import numpy as np
    from scipy.spatial.transform import Rotation
    from verify_bottle_grasp import contact_target, pose_matrix
    wrist, bottle = Pose(), Pose()
    bottle.position.x = .35
    bottle.position.z = 1.015
    q = Rotation.from_euler('y', 150., degrees=True).as_quat()
    wrist.orientation.x, wrist.orientation.y, wrist.orientation.z, wrist.orientation.w = map(float, q)
    target = contact_target(wrist, bottle, .13)
    t = pose_matrix(target)
    pad = (t @ [0., 0., .1358-.038, 1.])[:3]
    assert pad == pytest.approx([.35, 0., 1.08], abs=1e-12)
    assert t[:3, :3] == pytest.approx(pose_matrix(wrist)[:3, :3])
    # A very short bottle would put the tilted fingertip too close to the table.
    with pytest.raises(ValueError, match='too close'):
        contact_target(wrist, bottle, .02)


def test_level_jaw_axis_keeps_tcp_fixed_and_tool_tilted():
    import numpy as np
    from scipy.spatial.transform import Rotation
    from verify_bottle_grasp import level_opening_direction, pose_matrix
    wrist = Pose()
    wrist.position.x, wrist.position.z = .27, 1.35
    q = Rotation.from_euler('xyz', [15., 150., 10.], degrees=True).as_quat()
    wrist.orientation.x, wrist.orientation.y, wrist.orientation.z, wrist.orientation.w = map(float, q)
    before = pose_matrix(wrist)
    after = pose_matrix(level_opening_direction(wrist))
    assert after[2, 1] == pytest.approx(0., abs=1e-12)
    assert np.linalg.det(after[:3, :3]) == pytest.approx(1.)
    assert after @ [0., 0., .1358, 1.] == pytest.approx(before @ [0., 0., .1358, 1.], abs=1e-12)
    assert -.95 < after[2, 2] < -.7  # Tool remains tilted downward.


def test_contact_and_arm_motion_alone_do_not_prove_a_lift():
    from verify_bottle_grasp import lift_verified
    assert lift_verified(.01, .001, True)
    assert not lift_verified(0., .01, True)  # Both contacts exist but the bottle slid.
    assert not lift_verified(.01, .001, False)
    assert not lift_verified(float('nan'), .001, True)
    assert lift_verified(.002, .0002, True, distance=.002)
    assert not lift_verified(0., .002, True, distance=.002)


def test_planning_attachment_does_not_remove_world_object_twice():
    from types import SimpleNamespace
    from verify_bottle_grasp import VerifyBottle
    calls = []
    fake = SimpleNamespace(cube_name='vision_bottle', apply_scene=object(), attached=False)
    def service(client, request):
        calls.append(request)
        return SimpleNamespace(success=True)
    fake.service = service
    wrist, bottle = Pose(), Pose()
    bottle.position.x = .35
    VerifyBottle.attach_planning_geometry(fake, wrist, bottle)
    assert fake.attached
    assert len(calls[0].scene.robot_state.attached_collision_objects) == 1
    assert not calls[0].scene.world.collision_objects


def test_smaller_bottle_preserves_uniform_density_scaling():
    import xml.etree.ElementTree as ET
    from lxml import etree
    from bottle_model import model_path, dimensions
    # gazebo_ros spawn_entity passes file contents as Unicode to lxml.
    assert etree.fromstring(model_path().read_text()).tag == 'sdf'
    small = ET.parse(model_path()).getroot()
    data = dimensions()
    assert 2*data['radius'] == pytest.approx(.0364)
    assert data['height'] == pytest.approx(.1316)
    assert float(small.findtext('model/link/inertial/mass')) == pytest.approx(.18*.7**3)
    assert float(small.findtext('model/link/inertial/inertia/ixx')) == pytest.approx(.0005*.7**5)


def test_near_bottle_search_finds_views_and_matches_scalar_urdf_fk():
    import xml.etree.ElementTree as ET
    import numpy as np
    from scipy.spatial.transform import Rotation
    from observe_bottle import sample_joint_views, bottle_fits
    model = Path(__file__).resolve().parents[3] / 'piper_description/urdf/piper_description_gazebo.xacro'
    # Use the source model's mount defaults.
    root = ET.parse(model).getroot()
    tag = '{http://ros.org/wiki/xacro}arg'
    mount = np.eye(4)
    mount[:3, 3] = [float(v) for v in root.find(f"{tag}[@name='camera_mount_xyz']").get('default').split()]
    mount[:3, :3] = Rotation.from_euler('xyz', [float(v) for v in root.find(f"{tag}[@name='camera_mount_rpy']").get('default').split()]).as_matrix()
    color = np.eye(4); color[1, 3] = -.059
    optical = np.eye(4); optical[:3, :3] = Rotation.from_euler('xyz', [-math.pi/2, 0., -math.pi/2]).as_matrix()
    mount = mount @ color @ optical
    center = np.array([.35, 0., .094])
    candidates = sample_joint_views(model, mount, center, np.array([0., 1., -1., 0., 1., 0.]), count=2000)
    assert candidates, 'Regression: random-only and fixed-view search found no views at 0.35m.'
    for candidate in candidates:
        fk = np.eye(4)
        for i, angle in enumerate(candidate, 1):
            joint = root.find(f"joint[@name='joint{i}']")
            assert float(joint.find('limit').get('lower')) <= angle <= float(joint.find('limit').get('upper'))
            origin = joint.find('origin')
            fixed = np.eye(4)
            fixed[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]
            fixed[:3, :3] = Rotation.from_euler('xyz', [float(v) for v in origin.get('rpy', '0 0 0').split()]).as_matrix()
            moving = np.eye(4); moving[:3, :3] = Rotation.from_euler('z', angle).as_matrix()
            fk = fk @ fixed @ moving
        assert bottle_fits(fk @ mount, center)
