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
    LOOP_RATE,
    MAX_W,
    TURN_GAIN,
)
from TeamControl.world.transform_cords import world2robot


CHASE_SPEED = CRUISE_SPEED * 0.80
WAYPOINT_REACHED_MM = 180.0
TARGET_STOP_MM = 80.0
SMOOTH_ALPHA = 0.35
POSSESSION_DIST_MM = 90.0
POSSESSION_ANGLE_RAD = 0.015
STEAL_FRONT_DIST_MM = 650.0
STEAL_FRONT_ANGLE_RAD = 0.35


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
    planner = PlannerAPI(density_percent=60.0, max_density_nodes=140)
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

        target = sanitize_field_target(cache.ball.position, margin=100.0)
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
                horizon_ms=250,
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

        if not plan.is_path_free and not plan.waypoints:
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

        nav_vx, nav_vy = move_toward(
            rel_move,
            CHASE_SPEED,
            ramp_dist=450.0,
            stop_dist=TARGET_STOP_MM if plan.is_path_free else WAYPOINT_REACHED_MM,
            min_speed=0.06,
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
    points = [(float(robot_pose[0]), float(robot_pose[1]))]
    if plan.waypoints:
        points.extend((float(point[0]), float(point[1])) for point in plan.waypoints)
    elif plan.active_target_pose is not None:
        points.append((float(plan.active_target_pose[0]), float(plan.active_target_pose[1])))
    try:
        planner_path_q.put_nowait(
            {
                "robot_id": int(robot_id),
                "is_yellow": bool(is_yellow),
                "points": tuple(points),
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
