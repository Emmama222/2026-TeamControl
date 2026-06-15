# Voronoi Navigator — Stripped Behaviours

`voronoi_navigator.py` was intentionally simplified to a **bare integrator** for
troubleshooting the path planner.  The table below lists everything that was
removed, why it matters, and where it should live when the production navigator
is built.

---

## What was stripped

| Feature | Constants / symbols removed | Why it was there |
|---|---|---|
| **Face-target stop** | `FACE_TARGET_DIST_MM`, `FACE_TARGET_ANGLE_RAD` | Within dribble range the robot stops translating and rotates to face the ball first |
| **Penalty-box guard** | `is_goalie`, `_OWN_POSITIVE_SIDE`, `_OWN_BOX_X_EDGE`, `is_in_penalty_box` | Non-goalie robots clamp their target to the near edge of the own penalty box |
| **Possession / steal detection** | `_steal_ignore_keys`, `_robot_is_in_front_of_possessor`, `POSSESSION_DIST_MM`, `POSSESSION_ANGLE_RAD`, `STEAL_FRONT_DIST_MM`, `STEAL_FRONT_ANGLE_RAD` | Lets a robot challenge a possessing opponent by temporarily ignoring them as an obstacle |
| **Precision approach mode** | `PRECISION_APPROACH_SPEED`, `VORONOI_PRECISION_RAMP_DIST_MM`, `VORONOI_PRECISION_MIN_SPEED`, `VORONOI_PRECISION_SPEED_SCALE` | Slows to dribble speed and tightens the deceleration ramp when `plan.endpoint_precision_mode` is set |
| **Exponential velocity smoothing** | `SMOOTH_ALPHA`, `sm_vx / sm_vy / sm_w` | Low-pass filter on the output command to reduce jitter from frame-to-frame planner changes |
| **Field-margin target sanitisation** | `sanitize_field_target(margin=VORONOI_FIELD_TARGET_MARGIN_MM)` | ~~Moved into the planner~~ — `VoronoiDijkstraPlanner.plan()` now clamps the target to the field inset by `VORONOI_FIELD_TARGET_MARGIN_MM` before planning. No longer needed in the game navigator. |

The constants `FACE_TARGET_DIST_MM` and `FACE_TARGET_ANGLE_RAD` are still
defined in `src/TeamControl/robot/constants.py`.  The goalie config keys
(`goalie_yellow_id`, `goalie_blue_id`) are still parsed by
`src/TeamControl/utils/yaml_config.py` and declared in
`src/TeamControl/utils/ipconfig.yaml`.

---

## Where to add them back

All of these belong in a **game-layer navigator**, not in the integrator.
The recommended location is a new file:

```
src/TeamControl/robot/voronoi_game_navigator.py
```

It would:

1. Use the same `PlannerAPI` / `PlannerInput` loop as `voronoi_navigator.py`.
2. Accept `is_goalie: bool` (from `preset.goalie_yellow_id` / `goalie_blue_id`
   in `engine.py`) and apply the penalty-box guard.
3. Call `_steal_ignore_keys` to optionally pass
   `ignored_obstacle_keys_containing_target` to the planner.
4. Switch between `CHASE_SPEED` and `PRECISION_APPROACH_SPEED` based on
   `plan.endpoint_precision_mode`.
5. Apply the face-target stop when within `FACE_TARGET_DIST_MM`.
6. Apply the exponential smoothing on `vx / vy / w` before dispatch.
7. ~~Use `sanitize_field_target`~~ — target sanitisation is now handled inside the planner itself.

`engine.py` mode `"voronoi_test"` keeps using `run_voronoi_navigator` (the
integrator).  A future `"match"` or `"team"` mode would use
`run_voronoi_game_navigator` instead.

---

## Current integrator loop (what remains)

```
refresh cache
  → get ball position (raw)
  → get robot position
  → call PlannerAPI
  → move_toward(active_waypoint, CHASE_SPEED)
  → face ball (simple proportional ω)
  → send RobotCommand
```

No game rules, no role awareness, no multi-robot coordination.  Useful for
verifying that the Voronoi/Dijkstra planner generates correct waypoints in
isolation.
