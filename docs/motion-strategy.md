# Motion Strategy - TeamControl Division B

## Motion Systems

The PD controller (formerly "System C", `motion/controller.py`) has been
removed — it didn't work on the server-deployed/competition side. PD-based
control may come back later as robot-onboard logic, but it's not part of
this codebase for this competition. The two remaining systems:

| System | Module | Used by | Best for |
|---|---|---|---|
| **A — Proportional ramp** | `ball_nav.move_toward()` | striker, goalie, navigator, team, voronoi_navigator, voronoi_game_navigator | All active game behaviour — this is now the only actively-developed system |
| **B — PD with zone caps** | `Movement.py / RobotMovement` | behaviour tree, sandbox | Legacy/frozen — do not add new callers |

### When to use each

**System A (`ball_nav.move_toward`)** — use for everything. The target
(ball, opponent, open space) moves every frame, so a derivative term would
wrongly brake on target motion rather than robot motion. The proportional
ramp handles reactive tracking well, and the Voronoi planner's waypoint
output is fed through it the same way (see
`voronoi_navigator.py`/`voronoi_game_navigator.py`). Field-boundary safety
(decel zone, hard stop, out-of-field crawl) is `ball_nav.apply_boundary_braking()`
— ported from the old PD controller's `field_limit` option so the Voronoi
navigators kept that behaviour after the PD removal.

**System B (`Movement.py`)** — do not add new callers. The `RobotMovement`
class is the historical middle ground; the zone caps are useful for the
behaviour tree's structured decision flow. It has its own embedded copy of
a PD-style controller (kept as-is, untouched by the PD removal — it was
never an import of the deleted `motion/` package, just a separate, already-
deprecated implementation). `calculateBallVelocity()` in this file has
speed levels that are 10x too low (0.02–0.10 m/s instead of 0.2–1.0 m/s) —
do not use it for real robot speeds.

---

## Angular Velocity Ceiling

`MAX_W = MAX_W_RAW * W_CLAMP_PCT = 0.5 * 0.60 = 0.30 rad/s`

This is the angular-velocity ceiling used directly by `ball_nav`-based
navigation (e.g. the `clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)` pattern in
`voronoi_navigator.py`/`voronoi_game_navigator.py`). The legacy tuning
values `angular_normal_speed = 0.5` and `angular_fast_speed = 0.6` (in
`tuning.json`) are used only by System B's proportional layer — they
exceed `MAX_W` and are intentionally separate. To raise the ceiling,
increase `max_w_raw` or `w_clamp_pct` in `tuning.json`.

---

## Student Summary

- Ball-chasing game logic → `ball_nav.move_toward()`
- Voronoi-planner-driven navigation (test or match mode) → `ball_nav.move_toward()` +
  `ball_nav.apply_boundary_braking()`
- Behaviour tree (legacy) → `Movement.py (RobotMovement)` as-is, no new code there
