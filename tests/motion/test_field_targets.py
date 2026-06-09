from TeamControl.robot.ball_nav import (
    is_target_in_field_box,
    sanitize_field_target,
    wall_brake,
)
from TeamControl.robot.constants import HALF_LEN, HALF_WID, ROBOT_RADIUS


def test_target_inside_field_box_is_unchanged():
    assert sanitize_field_target((100.0, -200.0)) == (100.0, -200.0)


def test_outside_target_is_offset_inward_by_robot_radius():
    target = sanitize_field_target((HALF_LEN + 500.0, HALF_WID + 500.0))

    assert target == (
        HALF_LEN - ROBOT_RADIUS,
        HALF_WID - ROBOT_RADIUS,
    )
    assert is_target_in_field_box(target)


def test_outside_target_can_be_rejected():
    assert sanitize_field_target(
        (HALF_LEN + 1.0, 0.0),
        reject_outside=True,
    ) is None


def test_legacy_wall_brake_no_longer_slows_velocity_near_edge():
    assert wall_brake(HALF_LEN, HALF_WID, 1.0, -0.5) == (1.0, -0.5)
