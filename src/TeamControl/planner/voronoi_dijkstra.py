"""Dijkstra path planning over the bounded Voronoi world map."""

from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
from math import hypot
from typing import Iterable

from TeamControl.world.field_config import (
    FIELD_X_MAX,
    FIELD_X_MIN,
    FIELD_Y_MAX,
    FIELD_Y_MIN,
)
from TeamControl.world.map.geometry import distance_2_segment
from TeamControl.world.map.voronoi_generator import (
    VoronoiObstacle,
    generate_voronoi_map_from_world_map,
)


Point = tuple[float, float]
RobotKey = tuple[bool, int]


@dataclass(slots=True)
class PlannerState:
    """Per-robot reusable path state."""

    last_target_mm: Point | None = None
    waypoints_mm: tuple[Point, ...] = ()
    generated_at_s: float = 0.0


@dataclass(frozen=True, slots=True)
class PlanResult:
    """Serializable result returned through the WorldModel proxy."""

    target_mm: Point
    waypoints_mm: tuple[Point, ...]
    reused_previous: bool = False
    used_direct_path: bool = False


class VoronoiDijkstraPlanner:
    """Plan center-point waypoints while respecting WorldMap path clearance."""

    START_ID = -1
    TARGET_ID = -2

    def __init__(
        self,
        *,
        target_dead_zone_mm: float = 150.0,
        connection_count: int = 8,
        connection_radius_mm: float = 2200.0,
        horizon_ms: int | float = 250,
        density_percent: float = 60.0,
        max_density_nodes: int = 140,
        obstacle_cost_weight: float = 2.0,
        boundary_inset_mm: float = 100.0,
    ) -> None:
        self.target_dead_zone_mm = float(target_dead_zone_mm)
        self.connection_count = int(connection_count)
        self.connection_radius_mm = float(connection_radius_mm)
        self.horizon_ms = horizon_ms
        self.density_percent = float(density_percent)
        self.max_density_nodes = int(max_density_nodes)
        self.obstacle_cost_weight = float(obstacle_cost_weight)
        self.boundary_inset_mm = float(boundary_inset_mm)

    def plan(
        self,
        world_map,
        start_pos_mm: Point,
        target_pos_mm: Point,
        *,
        now_s: float | None = None,
        ignore_robots: set[RobotKey] | None = None,
        previous_state: PlannerState | None = None,
    ) -> PlanResult:
        """Return waypoints from *start_pos_mm* to a clamped target."""
        if ignore_robots is None:
            ignore_robots = set()
        if previous_state is None:
            previous_state = PlannerState()

        start = _point2(start_pos_mm)
        target = clamp_to_field(_point2(target_pos_mm))

        if world_map.is_path_free(
            start,
            target,
            ignore_robots=ignore_robots,
            horizon_ms=self.horizon_ms,
        ):
            return PlanResult(
                target_mm=target,
                waypoints_mm=(),
                used_direct_path=True,
            )

        if self._previous_path_is_valid(
            world_map,
            start,
            target,
            previous_state,
            ignore_robots,
        ):
            return PlanResult(
                target_mm=target,
                waypoints_mm=previous_state.waypoints_mm,
                reused_previous=True,
            )

        voronoi_map = generate_voronoi_map_from_world_map(
            world_map,
            now_s=now_s,
            horizon_ms=self.horizon_ms,
            ignore_robots=ignore_robots,
            density_percent=self.density_percent,
            max_density_nodes=self.max_density_nodes,
            obstacle_cost_weight=self.obstacle_cost_weight,
            boundary_inset_mm=self.boundary_inset_mm,
        )
        node_pos = {node.id: (node.x, node.y) for node in voronoi_map.nodes}
        adjacency: dict[int, list[tuple[int, float]]] = {}
        for edge in voronoi_map.edges:
            adjacency.setdefault(edge.start_id, []).append((edge.end_id, edge.cost))
            adjacency.setdefault(edge.end_id, []).append((edge.start_id, edge.cost))

        obstacles = tuple(voronoi_map.obstacles)
        self._connect_temporary_node(
            adjacency,
            node_pos,
            world_map,
            self.START_ID,
            start,
            ignore_robots,
            obstacles,
        )
        self._connect_temporary_node(
            adjacency,
            node_pos,
            world_map,
            self.TARGET_ID,
            target,
            ignore_robots,
            obstacles,
        )

        if self.START_ID not in adjacency or self.TARGET_ID not in adjacency:
            return PlanResult(target_mm=target, waypoints_mm=())

        ids = self._dijkstra(adjacency, self.START_ID, self.TARGET_ID)
        if not ids:
            return PlanResult(target_mm=target, waypoints_mm=())

        waypoints = tuple(
            target if node_id == self.TARGET_ID else node_pos[node_id]
            for node_id in ids[1:]
        )
        return PlanResult(target_mm=target, waypoints_mm=waypoints)

    def _previous_path_is_valid(
        self,
        world_map,
        start: Point,
        target: Point,
        previous_state: PlannerState,
        ignore_robots: set[RobotKey],
    ) -> bool:
        if previous_state.last_target_mm is None or not previous_state.waypoints_mm:
            return False
        if _distance(previous_state.last_target_mm, target) > self.target_dead_zone_mm:
            return False

        next_waypoint = previous_state.waypoints_mm[0]
        return world_map.is_path_free(
            start,
            next_waypoint,
            ignore_robots=ignore_robots,
            horizon_ms=self.horizon_ms,
        )

    def _connect_temporary_node(
        self,
        adjacency: dict[int, list[tuple[int, float]]],
        node_pos: dict[int, Point],
        world_map,
        temp_id: int,
        temp_pos: Point,
        ignore_robots: set[RobotKey],
        obstacles: tuple[VoronoiObstacle, ...],
    ) -> None:
        node_pos[temp_id] = temp_pos
        candidates = sorted(
            (
                (_distance(temp_pos, pos), node_id, pos)
                for node_id, pos in node_pos.items()
                if node_id >= 0
            ),
            key=lambda item: item[0],
        )
        connected = 0
        for distance, node_id, pos in candidates:
            if distance > self.connection_radius_mm and connected > 0:
                break
            if not world_map.is_path_free(
                temp_pos,
                pos,
                ignore_robots=ignore_robots,
                horizon_ms=self.horizon_ms,
            ):
                continue
            cost = distance * self._obstacle_cost_multiplier(temp_pos, pos, obstacles)
            adjacency.setdefault(temp_id, []).append((node_id, cost))
            adjacency.setdefault(node_id, []).append((temp_id, cost))
            connected += 1
            if connected >= self.connection_count:
                break

    def _obstacle_cost_multiplier(
        self,
        start: Point,
        end: Point,
        obstacles: Iterable[VoronoiObstacle],
    ) -> float:
        if self.obstacle_cost_weight <= 0:
            return 1.0
        risk = 0.0
        influence_mm = max(1.0, self.boundary_inset_mm + 600.0)
        for obstacle in obstacles:
            clearance = (
                distance_2_segment(obstacle.pos_mm, start, end)
                - obstacle.radius_mm
            )
            if clearance <= 0:
                risk += 2.0
            elif clearance < influence_mm:
                risk += (influence_mm - clearance) / influence_mm
        return 1.0 + self.obstacle_cost_weight * risk

    def _dijkstra(
        self,
        adjacency: dict[int, list[tuple[int, float]]],
        start_id: int,
        target_id: int,
    ) -> list[int]:
        distances = {start_id: 0.0}
        previous: dict[int, int] = {}
        queue = [(0.0, start_id)]

        while queue:
            cost, node_id = heappop(queue)
            if cost > distances.get(node_id, float("inf")):
                continue
            if node_id == target_id:
                break
            for next_id, edge_cost in adjacency.get(node_id, ()):
                next_cost = cost + edge_cost
                if next_cost < distances.get(next_id, float("inf")):
                    distances[next_id] = next_cost
                    previous[next_id] = node_id
                    heappush(queue, (next_cost, next_id))

        if target_id not in distances:
            return []

        path = [target_id]
        while path[-1] != start_id:
            path.append(previous[path[-1]])
        path.reverse()
        return path


def _point2(point: tuple[float, ...] | list[float]) -> Point:
    return (float(point[0]), float(point[1]))


def clamp_to_field(point: Point) -> Point:
    """Clamp a point to the full playable field rectangle."""
    return (
        max(FIELD_X_MIN, min(FIELD_X_MAX, float(point[0]))),
        max(FIELD_Y_MIN, min(FIELD_Y_MAX, float(point[1]))),
    )


def _distance(a: Point, b: Point) -> float:
    return hypot(a[0] - b[0], a[1] - b[1])
