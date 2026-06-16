# configurations of field
TEAM_IS_POSITIVE = True

FIELD_LENGTH_MM = 4000
FIELD_WIDTH_MM = 3000
DEFENCE_X_MM = 1000
DEFENCE_Y_MM = 1000

GOAL_WIDTH_MM = 1000          # total opening between the two posts
GOAL_HALF_WIDTH_MM = 500      # ± Y from field centre
GOAL_DEPTH_MM = 180           # how far the box extends past the end line

FIELD_X_MIN = -FIELD_LENGTH_MM / 2
FIELD_X_MAX = FIELD_LENGTH_MM / 2
FIELD_Y_MIN = -FIELD_WIDTH_MM / 2
FIELD_Y_MAX = FIELD_WIDTH_MM / 2

ROBOT_RADIUS_MM = 70.0
SAFE_MARGIN = 30.0  #mm
BALL_R =  21.5 #mm

# Dashboard field/manual placement feedback.
DASHBOARD_BALL_PLACE_CONFIRM_SECONDS = 0.5
DASHBOARD_BALL_PLACE_CONFIRM_TOLERANCE_MM = 80.0


GRID_SPACING_MM = 500
BOUNDARY_SAMPLE_SPACING_MM = 500


# Active Voronoi/map-planning defaults.
VORONOI_MIN_CLEARANCE_MM = ROBOT_RADIUS_MM + SAFE_MARGIN
VORONOI_BOUNDARY_INSET_MM = 100.0
VORONOI_DENSITY_PERCENT = 60.0
VORONOI_MAX_DENSITY_NODES = 140
VORONOI_RENDER_DENSITY_PERCENT = 10.0
VORONOI_RENDER_MAX_DENSITY_NODES = 80
VORONOI_GENERATOR_MAX_DENSITY_NODES = 240
VORONOI_OBSTACLE_COST_WEIGHT = 2.0
VORONOI_GENERATOR_OBSTACLE_COST_WEIGHT = 0.0
VORONOI_HORIZON_MS = 250
VORONOI_TARGET_DEAD_ZONE_MM = 150.0
VORONOI_CONNECTION_COUNT = 8
VORONOI_CONNECTION_RADIUS_MM = 2200.0
VORONOI_ENDPOINT_REACH_MM = 70.0
VORONOI_ESCAPE_MARGIN_MM = 120.0
VORONOI_MIN_ESCAPE_STEP_MM = 250.0


# Voronoi navigator behaviour.
VORONOI_CHASE_SPEED_SCALE = 0.80
VORONOI_PRECISION_SPEED_SCALE = 0.75
VORONOI_WAYPOINT_REACHED_MM = 180.0
VORONOI_TARGET_STOP_MM = 80.0
VORONOI_TARGET_OFFSET_MM = VORONOI_TARGET_STOP_MM      # stop this far from the ball in planner-test mode
VORONOI_OUT_OF_FIELD_SPEED_SCALE = 0.1                 # velocity multiplier when robot is outside field
VORONOI_BOUNDARY_DECEL_ZONE_MM = 400.0                 # mm inside boundary where linear ramp begins
VORONOI_BOUNDARY_NEAR_SPEED_SCALE = 0.05               # speed floor (fraction of MAX_SPEED) at the boundary wall
VORONOI_BOUNDARY_HARD_STOP_MM = 30.0                   # zero boundary-approaching velocity within this distance
VORONOI_FIELD_TARGET_MARGIN_MM = 150.0
VORONOI_PRECISION_RAMP_DIST_MM = 260.0
VORONOI_CHASE_RAMP_DIST_MM = 450.0
VORONOI_MIN_SPEED = 0.06
VORONOI_PRECISION_MIN_SPEED = 0.03
VORONOI_SMOOTH_ALPHA = 0.35
VORONOI_POSSESSION_DIST_MM = 90.0
VORONOI_POSSESSION_ANGLE_RAD = 0.015
VORONOI_STEAL_FRONT_DIST_MM = 650.0
VORONOI_STEAL_FRONT_ANGLE_RAD = 0.35

# ---------------------------------------------------------------------------
# Live field bounds
# Updated when an SSL-Vision geometry packet arrives (via update_live_bounds).
# Falls back to the hardcoded FIELD_LENGTH_MM x FIELD_WIDTH_MM defaults
# until then -- robot/constants.py's HALF_LEN/FIELD_LENGTH etc. read this
# same live state (via __getattr__), so there's a single source of truth
# for field size instead of two independently-hardcoded values.
# ---------------------------------------------------------------------------
_live_bounds: tuple[float, float, float, float] = (
    float(FIELD_X_MIN), float(FIELD_X_MAX),
    float(FIELD_Y_MIN), float(FIELD_Y_MAX),
)


def get_live_bounds() -> tuple[float, float, float, float]:
    """Return (x_min, x_max, y_min, y_max) in mm.

    Uses real SSL-Vision field dimensions once geometry has been received;
    falls back to the FIELD_LENGTH_MM x FIELD_WIDTH_MM defaults until then.
    """
    return _live_bounds


def get_live_half_length() -> float:
    """Half the live field length (mm) -- x_max from get_live_bounds()."""
    _, x_max, _, _ = get_live_bounds()
    return x_max


def get_live_half_width() -> float:
    """Half the live field width (mm) -- y_max from get_live_bounds()."""
    _, _, _, y_max = get_live_bounds()
    return y_max


def get_live_penalty_depth() -> float:
    """Live penalty-box depth (mm) -- defence_x from get_live_defence()."""
    return get_live_defence()[0]


def get_live_penalty_half_width() -> float:
    """Live penalty-box half-width (mm) -- defence_y from get_live_defence()."""
    return get_live_defence()[1]


def get_live_goal_half_width() -> float:
    """Live goal half-width (mm) -- goal_half_width from get_live_defence()."""
    return get_live_defence()[2]


def get_live_goal_depth() -> float:
    """Live goal depth (mm) -- goal_depth from get_live_defence()."""
    return get_live_defence()[3]


def get_live_max_advance(margin: float = 50.0) -> float:
    """Goalie's own-box advance limit -- penalty depth minus a clearance margin."""
    return get_live_penalty_depth() - margin


def update_live_bounds(field_length_mm: float, field_width_mm: float) -> None:
    """Refresh live bounds from an SSL-Vision geometry packet."""
    global _live_bounds
    _live_bounds = (
        -field_length_mm / 2.0,
         field_length_mm / 2.0,
        -field_width_mm / 2.0,
         field_width_mm / 2.0,
    )


# ---------------------------------------------------------------------------
# Live defence / goal geometry
# Updated alongside live bounds when an SSL-Vision geometry packet arrives.
# Stores half-widths to match the convention used in voronoi_dijkstra.py.
# Falls back to the hardcoded defaults until geometry is received.
# ---------------------------------------------------------------------------
_live_defence: tuple[float, float, float, float] = (
    float(DEFENCE_X_MM),       # penalty area depth
    float(DEFENCE_Y_MM),       # penalty area half-width
    float(GOAL_HALF_WIDTH_MM), # goal half-width
    float(GOAL_DEPTH_MM),      # goal depth
)


def get_live_defence() -> tuple[float, float, float, float]:
    """Return (defence_x_mm, defence_y_mm, goal_half_width_mm, goal_depth_mm).

    All values in mm.  defence_y and goal_half_width are half-widths (±Y).
    Uses real SSL-Vision geometry once a packet has been received.
    """
    return _live_defence


def update_live_defence(
    penalty_area_depth: float,
    penalty_area_width: float,
    goal_width: float,
    goal_depth: float,
) -> None:
    """Refresh live defence/goal geometry from an SSL-Vision geometry packet.

    penalty_area_width and goal_width are the *total* widths from the proto;
    they are halved here to match the ±Y half-width convention used internally.
    """
    global _live_defence
    _live_defence = (
        float(penalty_area_depth),
        float(penalty_area_width) / 2.0,
        float(goal_width) / 2.0,
        float(goal_depth),
    )
