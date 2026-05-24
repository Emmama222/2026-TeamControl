# Motion Strategy - TeamControl Division B

## Three Motion Systems

The codebase has three distinct motion systems.  Choose the right one for the task.

| System | Module | Used by | Best for |
|---|---|---|---|
| **A — Proportional ramp** | `ball_nav.move_toward()` | striker, goalie, navigator, team | Reactive game behaviour with a moving ball target |
| **B — PD with zone caps** | `Movement.py / RobotMovement` | behaviour tree, voronoi, sandbox | BT layer; keep as-is, but no new callers |
| **C — PD with deadline** | `motion/controller.py / RobotMotionController` | PD calibration UI, go-to-point | Precise static-target navigation |

### When to use each

**System A (`ball_nav.move_toward`)** — use for all active game behaviour.  The target
(ball, opponent, open space) moves every frame, so a derivative term would wrongly brake
on target motion rather than robot motion.  The proportional ramp with `wall_brake` and
arc-navigation handles reactive tracking well.

**System B (`Movement.py`)** — do not add new callers.  The `RobotMovement` class is the
historical middle ground; the zone caps are useful for the behaviour tree's structured
decision flow.  The embedded `PDController` is a copy of `motion/pd.py` (identical code).
`calculateBallVelocity()` in this file has speed levels that are 10x too low (0.02–0.10 m/s
instead of 0.2–1.0 m/s) — do not use it for real robot speeds.

**System C (`motion/controller.py`)** — use for calibration and precise point navigation.
The D term damps overshoot at a fixed target, per-robot hardware compensation
(`movement_calibration.json`) corrects for real-robot drift and dead-zones, and
deadline-based speed scaling gives the BT predictable timing.

---

## PD Strategy Options (within System C)

`strategy.py` exposes two strategies for callers of `RobotMotionController`:

| Option | What it does | Use it? |
|---|---|---|
| **A - Sequential** | Turn to face the target, then drive straight. | **Yes, default** |
| **B - Legacy combined** | Drive and turn at the same time using the old API. | **No — removed** |
| **C - Guarded combined** | If heading error is large, turn first. Otherwise drive and turn together with scaling. | After Option A is stable in grSim |

### Option A - Sequential Movement (default)

Per tick:

1. If the robot is not facing the target direction, rotate only.
2. If the robot is facing the target direction, drive only.

```python
if not mv.is_facing_dir(current_pos[2], target_theta):
    w = mv.rotational_motion(current_pos[2], target_theta, deadline)
    dispatch_q.put((RobotCommand(id, 0, 0, w, 0, 0, yellow), 0.15))
else:
    vx, vy = mv.translational_motion(current_pos, target_xy, deadline)
    dispatch_q.put((RobotCommand(id, vx, vy, 0, 0, 0, yellow), 0.15))
```

Why this is the default:

- When the robot spins, only rotation code is active.
- When the robot drives badly, only translation code is active.
- Logs are easier to read because `w` and `vx/vy` are not changing at the same time.

### Option C - Guarded Combined Movement

`general_motion()` is Option C.  Use it after Option A is stable in grSim.

```python
vx, vy, w = mv.general_motion(current_pos, target_xy, target_theta, deadline)
```

- If heading error > 60 degrees: rotate only.
- Otherwise: drive and rotate together with scaling.
- Reduce `vx/vy` when facing the wrong way; reduce `w` when still far from target.

Tuning: increase `BLEND_DIST` in `constants.py` (default 300 mm) if the robot spins
too much during combined movement.

---

## Angular Velocity Ceiling

`MAX_W = MAX_W_RAW * W_CLAMP_PCT = 0.5 * 0.60 = 0.30 rad/s`

The PD controller's angular output is capped at 0.30 rad/s.  The legacy tuning values
`angular_normal_speed = 0.5` and `angular_fast_speed = 0.6` (in `tuning.json`) are used
only by System A/B proportional layers — they exceed the PD cap and are intentionally
separate.  To raise the PD ceiling, increase `max_w_raw` or `w_clamp_pct` in `tuning.json`.

---

## Student Summary

- Ball-chasing game logic → `ball_nav.move_toward()`
- Calibration or precise go-to-point → `motion/controller.py (RobotMotionController)`
- Behaviour tree / voronoi → `Movement.py (RobotMovement)` as-is, no new code there
- Within `RobotMotionController`: start with Option A (sequential); upgrade to Option C only after Option A is stable
