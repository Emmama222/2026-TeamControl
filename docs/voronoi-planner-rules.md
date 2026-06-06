# Voronoi Planner Rules

This document describes the Dijkstra-based planner that sits on top of the
bounded Voronoi map generator in `src/TeamControl/world/map/`.

The implementation is separate from the legacy
`src/TeamControl/voronoi_planner/` package:

```text
src/TeamControl/planner/
src/TeamControl/planner/voronoi_dijkstra.py
src/TeamControl/robot/voronoi_navigator.py
```

## Inputs

The planner should receive:

- current robot position
- target point
- `WorldMap`
- optional previous planner state
- optional controlled robot identity for ignored obstacles

The planner should build or receive a bounded Voronoi map that already includes:

- virtual grid sites
- predicted robot obstacles from `WorldMap.get_planning_obstacles()`
- safe graph nodes
- safe graph edges
- obstacle-aware edge costs

## Rule 0: Edge Weighting Near Obstacles

Each edge cost should increase when obstacles are close to that edge.

The generator and planner weight safe edges by obstacle proximity. Unsafe edges
are still removed first; weighting only ranks the remaining choices.

Base weighting:

```text
cost = edge_length * (1 + obstacle_cost_weight * min_clearance / edge_clearance)
```

Additional nearby-obstacle risk is cumulative, so two obstacles near the same
corridor make that corridor more expensive than one obstacle:

```text
cost = edge_length * (1 + obstacle_cost_weight * (clearance_risk + obstacle_risk_sum))
```

Where each nearby obstacle contributes:

```text
obstacle_penalty = weight * min_clearance / obstacle_edge_clearance
```

Notes:

- Unsafe edges are still rejected before planning.
- Weighting should only rank safe choices.
- Wider corridors should naturally win over narrow corridors.

## Rule 1: Clamp Target To Playable Area

When the planner receives a target point, it first checks whether the target is
inside the playable area.

The first clamp is the field rectangle.

Default field:

```text
x: -4500 to 4500
y: -3000 to 3000
```

If the target is outside the field, clamp it to the nearest point inside the
field.

Clamping should preserve the coordinate that is already valid where possible:

```text
(5000, 1000)  -> (4500, 1000)
(1000, 4000)  -> (1000, 3000)
(5000, 4000)  -> (4500, 3000)
```

Later we may change this to use the inset navigation bounds instead of the raw
field boundary.

## Rule 2: Always Check Direct Path

Before planning through the graph, the planner checks direct path safety.

Use the existing world-map path check:

```python
world_map.is_path_free(start_pos, target_pos, ...)
```

This keeps the planner aligned with the existing obstacle model in
`src/TeamControl/world/map/`.

If the direct path is free:

```text
active_target_pose = target_pos
waypoints = []
```

No Dijkstra search is needed.

In the current `PlannerAPI` flow, `VoronoiWaypointManager.update()` also returns
this decision as:

```python
planner_output.is_path_free
```

If `is_path_free` is `True`, `active_target_pose` is the field-clamped target
and no replan is performed. The planner also clears any previous planned
waypoints in this case. If it is `False`, the manager checks whether an
existing waypoint path can still be used before running a fresh Dijkstra plan.

## Rule 2a: Reuse Existing Path When Valid

If the direct path is not free, the planner may reuse a previous planned path
instead of rebuilding from scratch.

The cached path is stored as a queue of remaining waypoints. When the controller
reports `robot_reached_current_waypoint`, the manager pops the first waypoint
from that queue before deciding the next active target.

Every tick then checks the direct path to the final goal:

- If the direct path is free, clear all queued waypoints and return the final
  target directly.
- If the direct path is blocked and the next queued waypoint is still reachable,
  keep following that queued waypoint.
- If the direct path becomes blocked again after a direct-free tick, the queue
  will be empty, so the planner generates a fresh route.

The cached queue is valid only if all of these are true:

1. The new target is still within the dead-zone radius of the last target.
2. There are remaining queued waypoints for the similar target.
3. The direct path from the current position to the next waypoint is still free.

If any check fails, the cached path is invalid and the planner must generate a
new path.

Suggested state:

```python
@dataclass
class PlannerState:
    last_target_mm: tuple[float, float] | None
    waypoints_mm: tuple[tuple[float, float], ...]
    generated_at_s: float
```

Suggested configuration:

```python
target_dead_zone_mm = 150.0
```

## New Path Plan

When direct path and cached path both fail:

1. Generate or refresh the obstacle-aware Voronoi map.
2. Build the obstacle-aware navigation graph.
3. Append the final target point if it is not already reached.
4. Cache the target and generated waypoints in planner state.

## Expected Public API

Main class:

```python
class VoronoiDijkstraPlanner:
    def plan(
        self,
        world_map,
        start_pos_mm: tuple[float, float],
        target_pos_mm: tuple[float, float],
        *,
        now_s: float | None = None,
        ignore_robots: set[tuple[bool, int]] | None = None,
        previous_state: PlannerState | None = None,
    ) -> PlanResult:
        ...
```

`VoronoiDijkstraPlanner` is the low-level graph search. Robot behavior and the
future Skill Intent Executor should normally call `PlannerAPI` instead:

```python
planner_output = planner.plan(planner_input)
```

The planner API owns the per-robot route cache. `WorldModel` remains the source
of obstacle/world snapshots.

## Waypoint Manager Adapter

The Skill Intent Executor can use the task-PDF shaped adapter:

```python
from TeamControl.planner import PlannerAPI, PlannerInput

planner = PlannerAPI()
obstacles = wm.get_planning_obstacles(
    now_s=now_s,
    horizon_ms=250,
    ignore_robots=((is_yellow, robot_id),),
)

planner_output = planner.plan(
    PlannerInput(
        robot_id=robot_id,
        is_yellow=is_yellow,
        current_pose=current_pose,
        target_pose=target_pose,
        obstacles=obstacles,
        clearance_mm=200,
        robot_reached_current_waypoint=pd_output.robot_reached_target,
    )
)
```

`planner_output.active_target_pose` is the target the PD Controller should
track. If there is an active waypoint, it is returned first. If the direct path
is free, the field-clamped target is returned.

`WorldModel` should provide world snapshots, obstacle snapshots, and render
data. The planner API owns route state and waypoint decisions. This keeps the
world layer from turning into a behavior/planning service.

## Ball-Steal Clearance Exception

`voronoi_test` normally keeps clearance rules enabled while following the ball.
The only exception is a narrow ball-steal case.

An obstacle can be ignored at the ball target only when it is the specific robot
that possesses the ball. Possession is defined as:

```text
distance(robot, ball) < 90 mm
abs(angle_to_ball_in_robot_frame) <= 0.015 rad
ball is in front of the robot
```

The chasing robot must also be in front of the possessor before the possessor
obstacle is ignored. This prevents the planner from globally disabling
clearance whenever the ball happens to sit inside another robot's radius.

## Planned Path Debug Layer

`voronoi_test` publishes each robot's current planner route to the UI engine.
The background `WorldMapRenderWorker` renders those routes as a `Planned paths`
layer on the Map Debug canvas.

The path layer contains:

- the robot's current position
- the active waypoint list, if a Voronoi route is being followed

This layer is separate from the yellow Voronoi graph edges. It shows the actual
route the robot process is currently using.

When `planner_output.is_path_free` is `True`, the robot is using the direct
free path, so the planned-path debug layer receives an empty point list for that
robot. This clears any stale reroute polyline instead of drawing a direct-target
line as a planned route.
