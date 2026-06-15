"""Simple integrator that exercises the Voronoi/Dijkstra planner."""

from __future__ import annotations

import math
import time

from TeamControl.cache import TickCache
from TeamControl.network.robot_command import RobotCommand
from TeamControl.planner import PlannerAPI, PlannerInput
from TeamControl.robot.ball_nav import clamp, move_toward, rotation_compensate
from TeamControl.robot.constants import (
    CRUISE_SPEED,
    FACE_TARGET_ANGLE_RAD,
    FACE_TARGET_DIST_MM,
    LOOP_RATE,
    MAX_W,
    TURN_GAIN,
)
from TeamControl.world.field_config import (
    FIELD_X_MAX,
    FIELD_X_MIN,
    FIELD_Y_MAX,
    FIELD_Y_MIN,
    VORONOI_CHASE_RAMP_DIST_MM,
    VORONOI_CHASE_SPEED_SCALE,
    VORONOI_DENSITY_PERCENT,
    VORONOI_HORIZON_MS,
    VORONOI_MAX_DENSITY_NODES,
    VORONOI_MIN_SPEED,
    VORONOI_OUT_OF_FIELD_SPEED_SCALE,
    VORONOI_TARGET_OFFSET_MM,
    VORONOI_TARGET_STOP_MM,
    VORONOI_WAYPOINT_REACHED_MM,
)
from TeamControl.world.transform_cords import world2robot


CHASE_SPEED = CRUISE_SPEED * VORONOI_CHASE_SPEED_SCALE
WAYPOINT_REACHED_MM = VORONOI_WAYPOINT_REACHED_MM
TARGET_STOP_MM = VORONOI_TARGET_STOP_MM


def run_voronoi_navigator(
    is_running,
    dispatch_q,
    wm,
    robot_id,
    is_yellow,
    planner_path_q=None,
):
    """Chase the ball using Voronoi/Dijkstra waypoints."""
    cache = TickCache(wm)
    planner = PlannerAPI(
        density_percent=VORONOI_DENSITY_PERCENT,
        max_density_nodes=VORONOI_MAX_DENSITY_NODES,
    )
    active_target = None

    while is_running.is_set():
        now = time.time()
        if not cache.refresh(now):
            time.sleep(LOOP_RATE)
            continue
        if not cache.ball.visible:
            _send_stop(dispatch_q, robot_id, is_yellow)
            time.sleep(LOOP_RATE)
            continue

        rpos = cache.robots.get_position(is_yellow, robot_id)
        if rpos is None:
            time.sleep(LOOP_RATE)
            continue

        ball = cache.ball.position
        ignore_robots = ((bool(is_yellow), int(robot_id)),)

        reached = (
            active_target is not None
            and math.hypot(active_target[0] - rpos[0], active_target[1] - rpos[1])
            <= WAYPOINT_REACHED_MM
        )

        try:
            obstacles = wm.get_planning_obstacles(
                now_s=now,
                horizon_ms=VORONOI_HORIZON_MS,
                ignore_robots=ignore_robots,
            )
            plan = planner.plan(PlannerInput(
                robot_id=robot_id,
                is_yellow=is_yellow,
                current_pose=(float(rpos[0]), float(rpos[1]), float(rpos[2])),
                target_pose=(float(ball[0]), float(ball[1]), 0.0),
                obstacles=obstacles,
                clearance_mm=0.0,
                robot_reached_current_waypoint=reached,
                now_s=now,
            ))
        except Exception:
            plan = None

        if not plan:
            _send_stop(dispatch_q, robot_id, is_yellow)
            time.sleep(LOOP_RATE)
            continue

        active_target = plan.active_target_pose
        _publish_planned_path(
            planner_path_q,
            robot_id=robot_id,
            is_yellow=is_yellow,
            robot_pose=rpos,
            plan=plan,
            now_s=now,
        )

        rx, ry = float(rpos[0]), float(rpos[1])
        outside_field = (
            rx < FIELD_X_MIN or rx > FIELD_X_MAX
            or ry < FIELD_Y_MIN or ry > FIELD_Y_MAX
        )

        # If outside the field, drive straight back to the nearest boundary
        # point and scale velocity to 10% to prevent runaway overshoot.
        movement_target = (
            (max(FIELD_X_MIN, min(FIELD_X_MAX, rx)),
             max(FIELD_Y_MIN, min(FIELD_Y_MAX, ry)))
            if outside_field else active_target
        )

        rel_move = world2robot(rpos, movement_target)
        rel_ball = world2robot(rpos, ball)

        nav_vx, nav_vy = move_toward(
            rel_move,
            CHASE_SPEED,
            ramp_dist=VORONOI_CHASE_RAMP_DIST_MM,
            stop_dist=TARGET_STOP_MM,
            min_speed=VORONOI_MIN_SPEED,
        )

        if outside_field:
            nav_vx *= VORONOI_OUT_OF_FIELD_SPEED_SCALE
            nav_vy *= VORONOI_OUT_OF_FIELD_SPEED_SCALE

        dist_to_ball = math.hypot(rel_ball[0], rel_ball[1])
        ang_ball = math.atan2(rel_ball[1], rel_ball[0])

        # Stop once within VORONOI_TARGET_OFFSET_MM of the ball.
        if dist_to_ball < VORONOI_TARGET_OFFSET_MM:
            nav_vx, nav_vy = 0.0, 0.0

        # Face the ball before moving when within dribble range.
        if dist_to_ball < FACE_TARGET_DIST_MM and abs(ang_ball) > FACE_TARGET_ANGLE_RAD:
            nav_vx, nav_vy = 0.0, 0.0

        w = 0.0 if abs(ang_ball) < 0.05 else clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)

        out_vx, out_vy = rotation_compensate(nav_vx, nav_vy, w)
        dispatch_q.put((
            RobotCommand(
                robot_id=robot_id,
                vx=out_vx,
                vy=out_vy,
                w=w,
                kick=0,
                dribble=0,
                isYellow=is_yellow,
            ),
            0.15,
        ))
        time.sleep(LOOP_RATE)


def _publish_planned_path(
    planner_path_q,
    *,
    robot_id: int,
    is_yellow: bool,
    robot_pose,
    plan,
    now_s: float,
) -> None:
    if planner_path_q is None:
        return
    points = ()
    if not plan.is_path_free and plan.waypoints:
        points = (
            (float(robot_pose[0]), float(robot_pose[1])),
            *((float(p[0]), float(p[1])) for p in plan.waypoints),
        )
    try:
        planner_path_q.put_nowait({
            "robot_id": int(robot_id),
            "is_yellow": bool(is_yellow),
            "points": points,
            "timestamp_s": float(now_s),
            "is_path_free": bool(plan.is_path_free),
            "need_reroute": bool(plan.need_reroute),
            "did_reroute": bool(plan.did_reroute),
        })
    except Exception:
        pass


def _send_stop(dispatch_q, robot_id: int, is_yellow: bool) -> None:
    dispatch_q.put((
        RobotCommand(
            robot_id=robot_id,
            vx=0.0,
            vy=0.0,
            w=0.0,
            kick=0,
            dribble=0,
            isYellow=is_yellow,
        ),
        0.15,
    ))
