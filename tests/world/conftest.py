"""Reset module-level live-field state between tests so test order doesn't matter."""

import pytest
import TeamControl.world.field_config as fc


@pytest.fixture(autouse=True)
def reset_live_field_state():
    """Restore live bounds and defence to the hardcoded defaults after every test."""
    yield
    fc._live_bounds = (
        float(fc.FIELD_X_MIN), float(fc.FIELD_X_MAX),
        float(fc.FIELD_Y_MIN), float(fc.FIELD_Y_MAX),
    )
    fc._live_defence = (
        float(fc.DEFENCE_X_MM),
        float(fc.DEFENCE_Y_MM),
        float(fc.GOAL_HALF_WIDTH_MM),
        float(fc.GOAL_DEPTH_MM),
    )
