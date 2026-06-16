from TeamControl.robot.ball_nav import (
    is_target_in_field_box,
    sanitize_field_target,
    wall_brake,
)
from TeamControl.robot.constants import ROBOT_RADIUS
from TeamControl.world.field_config import get_live_bounds


def test_target_inside_field_box_is_unchanged():
    assert sanitize_field_target((100.0, -200.0)) == (100.0, -200.0)


def test_outside_target_is_offset_inward_by_robot_radius():
    # sanitize_field_target reads live field bounds (not the static
    # robot/constants.py HALF_LEN/HALF_WID, which describe a different
    # field size) -- see FieldGeometryCache in ball_nav.py.
    x_min, x_max, y_min, y_max = get_live_bounds()
    target = sanitize_field_target((x_max + 500.0, y_max + 500.0))

    assert target == (
        x_max - ROBOT_RADIUS,
        y_max - ROBOT_RADIUS,
    )
    assert is_target_in_field_box(target)


def test_outside_target_can_be_rejected():
    x_min, x_max, y_min, y_max = get_live_bounds()
    assert sanitize_field_target(
        (x_max + 1.0, 0.0),
        reject_outside=True,
    ) is None


def test_legacy_wall_brake_no_longer_slows_velocity_near_edge():
    # wall_brake is a deprecated no-op shim -- the position args are
    # irrelevant to the result, just need *some* (x, y).
    x_min, x_max, y_min, y_max = get_live_bounds()
    assert wall_brake(x_max, y_max, 1.0, -0.5) == (1.0, -0.5)
