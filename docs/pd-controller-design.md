# PD Controller Design

## Big Picture

The movement controller turns a target into robot commands:

```text
Vision / Game Controller / Robot Receiver
        -> Behaviour Tree
        -> Intent Executor
        -> Movement.py
        -> RobotCommand(vx, vy, w, kick, dribble)
```

For now, the intended executor strategy is **Motion Strategy Option A**:

1. Rotate until the robot faces the target direction.
2. Drive toward the target position.

`general_motion()` is available, but it is the experimental Option C path.

---

## Key Variables

| Name | Meaning |
|---|---|
| `vx` | Forward/backward speed in robot frame, m/s |
| `vy` | Sideways speed in robot frame, m/s |
| `w` | Angular speed, rad/s |
| `current_pos` | Robot pose: `(x, y, theta)` |
| `target_xy` | Target position: `(x, y)` |
| `target_theta` | Target heading angle |
| `deadline` | Absolute time the robot should try to arrive by |

---

## Time-Based Speed

The behaviour tree does not directly choose speed. It chooses a deadline.

Each tick, the controller asks:

> How fast do I need to move from here to arrive by the deadline?

```python
time_remaining = max(deadline - time.monotonic(), 0.001)
speed = min(dist_mm / 1000.0 / time_remaining, MAX_SPEED)
w_limit = min(angle_rad / time_remaining, MAX_W)
```

`Mode` is just a shortcut for common deadlines:

| Mode | Time budget | Meaning |
|---|---:|---|
| `FAST` | 0.1 s | Move as fast as allowed |
| `NORM` | 1.0 s | Normal movement |
| `SLOW` | 2.0 s | Slower, careful approach |

```python
deadline = time.monotonic() + Mode.NORM.value
```

---

## Main API

Use one persistent `RobotMovement` per robot. This matters because the PD
controller remembers the previous tick.

```python
from TeamControl.robot.Movement import Mode, get_movement

mv = get_movement(robot_id, is_yellow)
```

Important functions:

```python
# Check if robot is close enough to the position target
mv.is_close_to_target(current_xy, target_xy, threshold_mm=100.0)

# Check if robot is facing the target heading
mv.is_facing_dir(current_theta, target_theta, threshold_rad=0.1)

# Rotate only. Returns w.
mv.rotational_motion(current_theta, target_theta, deadline)

# Drive only. Returns vx, vy.
mv.translational_motion(current_pos, target_xy, deadline)

# Experimental combined movement. Returns vx, vy, w.
mv.general_motion(current_pos, target_xy, target_theta, deadline)
```

---

## Recommended Executor Logic

This is Option A from `motion-strategy.md`.

```python
if not mv.is_facing_dir(current_pos[2], target_theta):
    w = mv.rotational_motion(current_pos[2], target_theta, deadline)
    dispatch_q.put((RobotCommand(id, 0, 0, w, 0, 0, yellow), 0.15))
else:
    vx, vy = mv.translational_motion(current_pos, target_xy, deadline)
    dispatch_q.put((RobotCommand(id, vx, vy, 0, 0, 0, yellow), 0.15))
```

Why this is the default:

- Simple to debug.
- Turning and driving problems are separated.
- Good enough for Division B reliability.

---

## Experimental Combined Movement

`general_motion()` is Option C.

```python
vx, vy, w = mv.general_motion(current_pos, target_xy, target_theta, deadline)
```

It does this:

```text
if heading_error > 60 degrees:
    rotate only
else:
    drive and rotate together with scaling
```

Scaling means:

- If the robot is facing the wrong way, reduce `vx/vy`.
- If the robot is far from the target position, reduce `w`.

This helps reduce drift and local spinning, but it is still harder to debug
than Option A.

---

## Legacy API

Avoid this for the new Division B strategy:

```python
vx, vy, w = mv.velocity_to_target(
    robot_pos,
    target=linear_xy,
    turning_target=facing_pos,
)
```

This is the older combined movement API. It can still be useful for old code,
but it should not be the main shipping path.

---

## Tuning

PD gains live in `tuning.json`:

```json
{
  "turn_kp": 1.0,
  "turn_kd": 0.1,
  "linear_kp": 0.002,
  "linear_kd": 0.0005
}
```

Plain meaning:

- `kp`: how hard the robot pushes toward the target.
- `kd`: how much the robot damps/brakes as the error changes.
- Higher values are not always better; too high can cause oscillation.

Option C also uses:

```python
BLEND_DIST = 300.0
```

Increase `BLEND_DIST` if the robot spins too much during combined movement.

---

## Not Yet Integrated

| Piece | Current status |
|---|---|
| `RobotIntent` dataclass | Not built; only `Intent` enum exists |
| `IntentExecutor` | Missing |
| Behaviour tree outputs intents | Not yet; striker still writes direct velocities |
| Logger in executor | Missing |

Until these are built, treat this document as the movement design target, not
as a fully integrated architecture.
