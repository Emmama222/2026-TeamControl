"""Ball-chasing navigator that follows Voronoi/Dijkstra waypoints."""

from __future__ import annotations

import math
import time

from TeamControl.cache import TickCache
from TeamControl.network.robot_command import RobotCommand
from TeamControl.planner import PlannerAPI, PlannerInput
from TeamControl.robot.ball_nav import (
    clamp,
    move_toward,
    rotation_compensate,
    sanitize_field_target,
)
from TeamControl.robot.constants import (
    CRUISE_SPEED,
    DRIBBLE_SPEED,
    LOOP_RATE,
    MAX_W,
    TURN_GAIN,
)
from TeamControl.world.field_config import (
    VORONOI_CHASE_RAMP_DIST_MM,
    VORONOI_CHASE_SPEED_SCALE,
    VORONOI_DENSITY_PERCENT,
    VORONOI_FIELD_TARGET_MARGIN_MM,
    VORONOI_HORIZON_MS,
    VORONOI_MAX_DENSITY_NODES,
    VORONOI_MIN_SPEED,
    VORONOI_POSSESSION_ANGLE_RAD,
    VORONOI_POSSESSION_DIST_MM,
    VORONOI_PRECISION_MIN_SPEED,
    VORONOI_PRECISION_RAMP_DIST_MM,
    VORONOI_PRECISION_SPEED_SCALE,
    VORONOI_SMOOTH_ALPHA,
    VORONOI_STEAL_FRONT_ANGLE_RAD,
    VORONOI_STEAL_FRONT_DIST_MM,
    VORONOI_TARGET_STOP_MM,
    VORONOI_WAYPOINT_REACHED_MM,
)
from TeamControl.world.transform_cords import world2robot


CHASE_SPEED = CRUISE_SPEED * VORONOI_CHASE_SPEED_SCALE
PRECISION_APPROACH_SPEED = DRIBBLE_SPEED * VORONOI_PRECISION_SPEED_SCALE
WAYPOINT_REACHED_MM = VORONOI_WAYPOINT_REACHED_MM
TARGET_STOP_MM = VORONOI_TARGET_STOP_MM
SMOOTH_ALPHA = VORONOI_SMOOTH_ALPHA
POSSESSION_DIST_MM = VORONOI_POSSESSION_DIST_MM
POSSESSION_ANGLE_RAD = VORONOI_POSSESSION_ANGLE_RAD
STEAL_FRONT_DIST_MM = VORONOI_STEAL_FRONT_DIST_MM
STEAL_FRONT_ANGLE_RAD = VORONOI_STEAL_FRONT_ANGLE_RAD


def run_voronoi_navigator(
    is_running,
    dispatch_q,
    wm,
    robot_id,
    is_yellow,
    planner_path_q=None,
):
    """Run one robot through the live WorldMap Voronoi planner."""
    cache = TickCache(wm)
    planner = PlannerAPI(
        density_percent=VORONOI_DENSITY_PERCENT,
        max_density_nodes=VORONOI_MAX_DENSITY_NODES,
    )
    active_target = None
    sm_vx, sm_vy, sm_w = 0.0, 0.0, 0.0

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

        target = sanitize_field_target(
            cache.ball.position,
            margin=VORONOI_FIELD_TARGET_MARGIN_MM,
        )
        if target is None:
            _send_stop(dispatch_q, robot_id, is_yellow)
            time.sleep(LOOP_RATE)
            continue

        ignore_robots = ((bool(is_yellow), int(robot_id)),)
        reached_current_waypoint = (
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
            steal_ignore_keys = _steal_ignore_keys(
                cache,
                is_yellow=is_yellow,
                robot_id=robot_id,
                robot_pose=rpos,
                ball_pos=target,
            )
            plan = planner.plan(
                PlannerInput(
                    robot_id=robot_id,
                    is_yellow=is_yellow,
                    current_pose=(float(rpos[0]), float(rpos[1]), float(rpos[2])),
                    target_pose=(float(target[0]), float(target[1]), 0.0),
                    obstacles=obstacles,
                    clearance_mm=0.0,
                    robot_reached_current_waypoint=reached_current_waypoint,
                    ignored_obstacle_keys_containing_target=steal_ignore_keys,
                    now_s=now,
                )
            )
        except Exception:
            plan = None

        if not plan:
            _send_stop(dispatch_q, robot_id, is_yellow)
            time.sleep(LOOP_RATE)
            continue

        if (
            not plan.is_path_free
            and not plan.waypoints
            and not plan.endpoint_precision_mode
        ):
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
        movement_target = active_target
        rel_move = world2robot(rpos, movement_target)
        rel_ball = world2robot(rpos, target)
        distance_to_target = math.hypot(rel_move[0], rel_move[1])
        target_speed = (
            PRECISION_APPROACH_SPEED
            if plan.endpoint_precision_mode
            else CHASE_SPEED
        )

        nav_vx, nav_vy = move_toward(
            rel_move,
            target_speed,
            ramp_dist=(
                VORONOI_PRECISION_RAMP_DIST_MM
                if plan.endpoint_precision_mode
                else VORONOI_CHASE_RAMP_DIST_MM
            ),
            stop_dist=(
                TARGET_STOP_MM
                if plan.is_path_free or plan.endpoint_precision_mode
                else WAYPOINT_REACHED_MM
            ),
            min_speed=(
                VORONOI_PRECISION_MIN_SPEED
                if plan.endpoint_precision_mode
                else VORONOI_MIN_SPEED
            ),
        )

        ang_ball = math.atan2(rel_ball[1], rel_ball[0])
        raw_w = 0.0 if abs(ang_ball) < 0.05 else clamp(
            ang_ball * TURN_GAIN,
            -MAX_W,
            MAX_W,
        )
        if distance_to_target < WAYPOINT_REACHED_MM and not plan.is_path_free:
            nav_vx, nav_vy = 0.0, 0.0

        a = SMOOTH_ALPHA
        sm_vx = a * sm_vx + (1.0 - a) * nav_vx
        sm_vy = a * sm_vy + (1.0 - a) * nav_vy
        sm_w = a * sm_w + (1.0 - a) * raw_w

        out_vx, out_vy = rotation_compensate(sm_vx, sm_vy, sm_w)
        dispatch_q.put((
            RobotCommand(
                robot_id=robot_id,
                vx=out_vx,
                vy=out_vy,
                w=sm_w,
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
            *((float(point[0]), float(point[1])) for point in plan.waypoints),
        )
    try:
        planner_path_q.put_nowait(
            {
                "robot_id": int(robot_id),
                "is_yellow": bool(is_yellow),
                "points": points,
                "timestamp_s": float(now_s),
                "is_path_free": bool(plan.is_path_free),
                "need_reroute": bool(plan.need_reroute),
                "did_reroute": bool(plan.did_reroute),
            }
        )
    except Exception:
        pass


def _steal_ignore_keys(
    cache: TickCache,
    *,
    is_yellow: bool,
    robot_id: int,
    robot_pose,
    ball_pos,
) -> tuple[tuple[bool, int], ...]:
    """Return possessed-ball obstacle keys that this robot may challenge."""
    for other_yellow in (True, False):
        for other_id, other_pose in cache.robots.iter_team(other_yellow):
            if other_yellow == is_yellow and other_id == robot_id:
                continue
            rel_ball = cache.robots.relative_to_ball(other_yellow, other_id, ball_pos)
            if rel_ball is None:
                continue
            _, ball_dist, ball_angle = rel_ball
            if (
                ball_dist >= POSSESSION_DIST_MM
                or abs(ball_angle) > POSSESSION_ANGLE_RAD
            ):
                continue
            if not _robot_is_in_front_of_possessor(robot_pose, other_pose):
                continue
            return ((bool(other_yellow), int(other_id)),)
    return ()


def _robot_is_in_front_of_possessor(robot_pose, possessor_pose) -> bool:
    rel_robot = world2robot(possessor_pose, robot_pose)
    if rel_robot is None:
        return False
    distance = math.hypot(rel_robot[0], rel_robot[1])
    if distance > STEAL_FRONT_DIST_MM or rel_robot[0] <= 0:
        return False
    return abs(math.atan2(rel_robot[1], rel_robot[0])) <= STEAL_FRONT_ANGLE_RAD


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
