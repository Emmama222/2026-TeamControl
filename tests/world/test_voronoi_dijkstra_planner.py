import time

from TeamControl.planner.voronoi_dijkstra import (
    PlannerState,
    VoronoiDijkstraPlanner,
)
from TeamControl.world.field_config import FIELD_X_MAX, FIELD_Y_MIN
from TeamControl.world.map.obstacles import Obstacle
from TeamControl.world.map.world_map import WorldMap


def test_voronoi_planner_clamps_target_and_uses_direct_path():
    world_map = WorldMap()
    planner = VoronoiDijkstraPlanner()

    result = planner.plan(
        world_map,
        (0.0, 0.0),
        (FIELD_X_MAX + 500.0, FIELD_Y_MIN - 500.0),
    )

    assert result.used_direct_path is True
    assert result.target_mm == (FIELD_X_MAX, FIELD_Y_MIN)
    assert result.waypoints_mm == ()


def test_voronoi_planner_allows_main_field_robot_to_target_penalty_box_for_now():
    world_map = WorldMap()
    planner = VoronoiDijkstraPlanner()

    result = planner.plan(
        world_map,
        (-2500.0, 0.0),
        (-4000.0, 0.0),
    )

    assert result.used_direct_path is True
    assert result.target_mm == (-4000.0, 0.0)


def test_voronoi_planner_allows_penalty_robot_to_target_main_field_for_now():
    world_map = WorldMap()
    planner = VoronoiDijkstraPlanner()

    result = planner.plan(
        world_map,
        (-4000.0, 0.0),
        (0.0, 0.0),
    )

    assert result.used_direct_path is True
    assert result.target_mm == (0.0, 0.0)


def test_voronoi_planner_reuses_valid_previous_path_for_similar_target():
    now_s = time.time()
    world_map = WorldMap()
    world_map.obs = [
        Obstacle(
            timestamp=now_s,
            robot_id=1,
            isYellow=True,
            pos_mm=(0.0, 0.0, 0.0),
            received_at_s=now_s,
        )
    ]
    planner = VoronoiDijkstraPlanner(target_dead_zone_mm=200.0)
    state = PlannerState(
        last_target_mm=(1000.0, 0.0),
        waypoints_mm=((-1000.0, 1000.0), (1000.0, 0.0)),
    )

    result = planner.plan(
        world_map,
        (-2000.0, 0.0),
        (1050.0, 0.0),
        now_s=now_s,
        previous_state=state,
    )

    assert result.used_direct_path is False
    assert result.reused_previous is True
    assert result.waypoints_mm == state.waypoints_mm


def test_voronoi_planner_finds_detour_when_direct_path_is_blocked():
    now_s = time.time()
    world_map = WorldMap()
    world_map.obs = [
        Obstacle(
            timestamp=now_s,
            robot_id=1,
            isYellow=True,
            pos_mm=(0.0, 0.0, 0.0),
            received_at_s=now_s,
        )
    ]
    planner = VoronoiDijkstraPlanner(
        density_percent=60.0,
        max_density_nodes=120,
        connection_count=10,
    )

    result = planner.plan(
        world_map,
        (-2000.0, 0.0),
        (2000.0, 0.0),
        now_s=now_s,
    )

    assert result.used_direct_path is False
    assert result.waypoints_mm
    assert result.waypoints_mm[-1] == result.target_mm


def test_voronoi_planner_returns_escape_waypoint_when_start_inside_obstacle_clearance():
    now_s = time.time()
    world_map = WorldMap()
    world_map.obs = [
        Obstacle(
            timestamp=now_s,
            robot_id=1,
            isYellow=True,
            pos_mm=(0.0, 0.0, 0.0),
            received_at_s=now_s,
        )
    ]
    planner = VoronoiDijkstraPlanner(
        density_percent=60.0,
        max_density_nodes=120,
        connection_count=10,
    )

    result = planner.plan(
        world_map,
        (50.0, 0.0),
        (2000.0, 0.0),
        now_s=now_s,
    )

    assert result.used_direct_path is False
    assert result.waypoints_mm
    assert result.waypoints_mm[0][0] > 210.0
