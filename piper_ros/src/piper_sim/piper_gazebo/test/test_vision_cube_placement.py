import sys
from pathlib import Path
import pytest
from geometry_msgs.msg import Pose, Quaternion

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from vision_cube_to_basket import placement_targets, BASKET_SLOTS
from stack_cubes import inverse_pose
from sync_pick_scene import compose_pose


def test_higher_transfer_keeps_payload_and_tool_orientation():
    basket = Pose();basket.position.x = .22;basket.position.y = .16;basket.position.z = 1.015
    cube = Pose();cube.position.x = .4;cube.position.z = 1.085
    wrist = Pose();wrist.position.x = .4;wrist.position.z = 1.21
    wrist.orientation = Quaternion(x=0., y=1., z=0., w=0.)
    relative = compose_pose(inverse_pose(wrist), cube)
    above, transfer, release = placement_targets(basket, cube, relative, 0, .09)
    assert above.position.z == pytest.approx(1.137)
    assert above.position.x == pytest.approx(.16)
    assert above.position.y == pytest.approx(.16)
    assert transfer.orientation.y == pytest.approx(wrist.orientation.y)
    assert transfer.orientation.w == pytest.approx(wrist.orientation.w)
    assert compose_pose(transfer, relative).position.z == pytest.approx(above.position.z)
    assert compose_pose(release, relative).position.z == pytest.approx(1.047)


def test_three_slots_keep_high_transfer_within_near_side_of_basket():
    import math
    for index, (x, y) in enumerate(BASKET_SLOTS):
        assert abs(x)+.02 < .11 and abs(y)+.02 < .11
        assert math.hypot(.22+x, .16+y) < .28
        for previous in BASKET_SLOTS[:index]:
            assert math.dist((x, y), previous) >= .059
