# Active World Map

## Purpose

`WorldMap` sits between SSL Vision and path planning. It turns delayed frame
observations into a lightweight tracked world:

```text
SSL Vision detection frame
    -> Frame
    -> WorldSnapshot
    -> WorldMap
    -> predicted planning obstacles
    -> Voronoi planner and local navigator
```

The map stores observations in world coordinates. Convert to a robot-relative
frame only when a controller needs to issue movement commands.

## Vision Timestamps

SSL Vision detection frames provide:

```proto
required double t_capture = 2;
required double t_sent = 3;
```

Both values are seconds. `t_capture` is the important timestamp for tracking
because it describes when the camera observed the field. `t_sent` is useful for
measuring vision processing and network delay.

`Frame` preserves both values. When multiple cameras contribute to one frame,
the combined frame keeps the latest values.

Robot velocity uses differences between SSL `t_capture` values. Observation
freshness uses a separate local receipt timestamp from `time.time()`. Do not
subtract an SSL capture timestamp directly from the local Unix clock: SSL
sources are not required to use the same clock origin.

## Robot Tracking

Each fresh robot observation looks backward at the previous observation for the
same `(team_is_yellow, robot_id)`:

```python
new_obs.update_vel_from(old_obs)
```

Velocity is estimated in world coordinates:

```python
vx = (new_x - old_x) / dt_s
vy = (new_y - old_y) / dt_s
```

For short-horizon planning, predicted position is:

```python
predicted_x = x + vx * horizon_s
predicted_y = y + vy * horizon_s
```

## Planning Obstacles

Use:

```python
obstacles = wm.get_planning_obstacles(
    now_s=time.time(),
    horizon_ms=200,
    ignore_robots={(True, 0)},
)
```

Each immutable `PlanningObstacle` contains:

```text
robot_id
team_is_yellow
pos_mm
radius_mm
vel_mmps
observation_age_ms
prediction_horizon_ms
```

The planning horizon includes observation delay:

```python
prediction_horizon_ms = observation_age_ms + requested_horizon_ms
```

The effective radius expands with speed:

```python
radius_mm = safe_radius_mm + speed_mmps * prediction_horizon_s
```

The Voronoi planner should build from this frozen view, follow only a short path
segment, revalidate that segment against the newest map, and replan frequently.

## Ball Tracking

`Frame` preserves raw ball candidates from every camera. For compatibility with
older callers, `frame.ball` still exposes the first raw observation. New
tracking code should use all candidates through `WorldSnapshot.ball_candidates`.

`WorldMap` validates and ranks candidates before accepting one:

1. Reject candidates below the confidence threshold.
2. Reject candidates outside the received field dimensions.
3. Predict the previous ball position at the new capture timestamp.
4. Reject candidates that are too far from the prediction.
5. Select the highest-confidence remaining candidate.
6. Use distance from the predicted position as a confidence tie-breaker.
7. Preserve the last valid position when the ball briefly disappears.

The current tuning values are:

```python
BALL_MIN_CONFIDENCE = 0.1
BALL_BASE_TOLERANCE_MM = 150.0
BALL_TOLERANCE_RATE_MMPS = 7000.0
```

Useful map state:

```text
ball
ball_vel_mmps
ball_visible
ball_last_seen_s
last_rejected_ball_pos_mm
last_ball_rejection_reason
possible_ball_left_field_pos_mm
ball_left_field_pos_mm
```

`possible_ball_left_field_pos_mm` comes from an out-of-bounds vision
observation. `ball_left_field_pos_mm` comes from a confirmed game-controller
event. Keep those meanings separate.

Do not sort purely by distance. A false detection slightly closer to the
prediction should not automatically beat a much stronger observation.

## Qt Debug Renderer

The Qt command center has a `World Map` tab for inspecting tracked state. Its
checkboxes are generated from serializable `RenderLayer` objects, so layers can
be hidden independently. The built-in layers are:

```text
Robots
Velocity vectors
Predicted clearance
Ball
```

Velocity arrows show `250 ms` of travel. Predicted-clearance circles are hidden
by default and include both the requested horizon and current observation age.

Future maps should provide their own layers without importing Qt:

```python
voronoi = RenderLayer(
    "Voronoi edges",
    polylines=(RenderPolyline(points_mm=edge, color="#ffffff"),),
)
render_data = world_map.get_render_data(extra_layers=(voronoi,))
```

The canvas automatically adds a checkbox for the new layer.

The canvas starts with local field defaults, then switches to the latest
`SSL_GeometryFieldSize` received from vision. A changed geometry updates the
home field, world-map field, and calibration field without restarting the UI.
Debug render frames are requested at `10 Hz` only while the `World Map` tab is
visible, keeping the normal dashboard path lightweight.

## Voronoi Integration Plan

The next contained implementation steps are:

1. Convert `PlanningObstacle`s into clearance circles.
2. Merge overlapping expanded circles into blocked clusters.
3. Build a Voronoi graph from a frozen planning view.
4. Reject graph edges that violate dynamic clearance.
5. Follow only the next waypoint or short segment.
6. Revalidate the immediate segment and replan when the live map changes.

The global planner proposes a route. The local navigator remains responsible
for braking and replanning when moving obstacles invalidate that route.

## Field-Edge Targets

Movement code sanitizes world-frame targets before driving. A target outside
the playable field is offset inward to an inset box, rather than slowing every
command merely because the robot is close to a wall. Callers that cannot use an
offset target can explicitly reject it with:

```python
sanitize_field_target(target, reject_outside=True)
```

Distance-to-target deceleration and obstacle avoidance remain separate safety
behaviors.
