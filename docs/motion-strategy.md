# Motion Strategy - TeamControl Division B

## Short Decision

Use **Option A** first.

Option A is slower, but it is the easiest to test and debug: the robot turns
first, then drives. For Division B, reliable movement matters more than the
fastest possible path.

| Option | What it does | Use it? |
|---|---|---|
| **A - Sequential** | Turn to face the target, then drive straight. | **Yes, default** |
| **B - Legacy combined** | Drive and turn at the same time using the old API. | **No** |
| **C - Guarded combined** | If heading error is large, turn first. Otherwise drive and turn together with scaling. | Later, after Option A works in grSim |

---

## Option A - Sequential Movement

This is the shipping/default strategy.

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

Why this is easiest:

- When the robot spins, only rotation code is active.
- When the robot drives badly, only translation code is active.
- Logs are easier to read because `w` and `vx/vy` are not changing at the same time.

---

## Option B - Legacy Combined Movement

Do not ship this path.

```python
vx, vy, w = mv.velocity_to_target(
    robot_pos,
    target=linear_xy,
    turning_target=facing_pos,
)
dispatch_q.put((RobotCommand(id, vx, vy, w, 0, 0, yellow), 0.15))
```

This old API calculates driving and turning together. It can be fast, but it
does not have the guard/scaling used by Option C, so the robot may spin in
place or drift sideways.

Keep it only for legacy callers and comparison.

---

## Option C - Guarded Combined Movement

This is the experimental upgrade after Option A is stable.

```python
vx, vy, w = mv.general_motion(current_pos, target_xy, target_theta, deadline)
dispatch_q.put((RobotCommand(id, vx, vy, w, 0, 0, yellow), 0.15))
```

How it works:

- If heading error is greater than 60 degrees, rotate only.
- Otherwise, drive and rotate together.
- Translation is reduced when the robot is facing the wrong way.
- Rotation is reduced when the robot is still far from the target position.

Tuning:

- If Option C spins too much, increase `BLEND_DIST` in `constants.py`.
- Default: `BLEND_DIST = 300 mm`.

---

## Student Summary

Start with Option A because it separates the problem into two simple parts:

- **Turning problem:** is the robot facing the right direction?
- **Driving problem:** is the robot moving to the right point?

Only try Option C once Option A works. Avoid Option B unless you are reading
old code.
