"""Background worker for debug world-map render generation."""

from __future__ import annotations

import time
from multiprocessing import Queue

from TeamControl.process_workers.worker import BaseWorker
from TeamControl.world.map.renderer import (
    BALL,
    BLUE,
    MapRenderData,
    PREDICTION,
    RenderCircle,
    RenderLayer,
    RenderPolyline,
    RenderRobot,
    RenderVector,
    VELOCITY,
    YELLOW,
)
from TeamControl.world.map.voronoi_generator import generate_bounded_voronoi_map


class WorldMapRenderWorker(BaseWorker):
    """Generate complete debug map render data outside the UI/world processes."""

    def __init__(self, is_running, logger):
        super().__init__(is_running=is_running, logger=logger)
        self.delay_time = 0.005

    def setup(self, *args):
        self.request_q: Queue = args[0]
        self.response_q: Queue = args[1]
        self.logger.info("[world-map-render] setup completed")

    def step(self):
        request = self._latest_request()
        if request is None:
            time.sleep(self.delay_time)
            return

        started_s = time.perf_counter()
        try:
            render_data, voronoi_ms = _build_render_data(request)
            generation_ms = (time.perf_counter() - started_s) * 1000.0
            self.response_q.put(
                {
                    "render_data": render_data,
                    "generation_ms": generation_ms,
                    "voronoi_generation_ms": voronoi_ms,
                    "request_id": request.get("request_id", 0),
                }
            )
        except Exception as exc:
            self.response_q.put(
                {
                    "error": str(exc),
                    "request_id": request.get("request_id", 0),
                }
            )

    def _latest_request(self):
        latest = None
        while True:
            try:
                latest = self.request_q.get_nowait()
            except Exception:
                return latest


def _build_render_data(request: dict) -> tuple[MapRenderData, float | None]:
    obstacles = tuple(request.get("obstacles", ()))
    planning_obstacles = tuple(request.get("planning_obstacles", ()))
    ball = request.get("ball")
    ball_visible = bool(request.get("ball_visible", False))
    ball_vel_mmps = tuple(request.get("ball_vel_mmps", (0.0, 0.0)))
    planner_paths = tuple(request.get("planner_paths", ()))
    velocity_vector_seconds = float(request.get("velocity_vector_seconds", 0.25))

    robots = []
    velocity_vectors = []
    for obs in obstacles:
        color = YELLOW if obs.isYellow else BLUE
        center = (obs.pos_mm[0], obs.pos_mm[1])
        robots.append(
            RenderRobot(
                center_mm=center,
                orientation_rad=obs.pos_mm[2],
                color=color,
                label=str(obs.robot_id),
            )
        )
        velocity_vectors.append(
            RenderVector(
                start_mm=center,
                end_mm=(
                    center[0] + obs.vel_mmps[0] * velocity_vector_seconds,
                    center[1] + obs.vel_mmps[1] * velocity_vector_seconds,
                ),
                color=VELOCITY,
                label=f"{obs.speed_mmps:.0f} mm/s",
            )
        )

    predicted_circles = tuple(
        RenderCircle(
            center_mm=obs.pos_mm,
            radius_mm=obs.radius_mm,
            color=PREDICTION,
            label=str(getattr(obs, "robot_id", getattr(obs, "label", ""))),
        )
        for obs in planning_obstacles
    )

    ball_circles = ()
    ball_vectors = ()
    if ball is not None:
        ball_color = BALL if ball_visible else "#a86320"
        ball_circles = (
            RenderCircle(ball, 21.5, ball_color, filled=True),
        )
        ball_vectors = (
            RenderVector(
                start_mm=ball,
                end_mm=(
                    ball[0] + ball_vel_mmps[0] * velocity_vector_seconds,
                    ball[1] + ball_vel_mmps[1] * velocity_vector_seconds,
                ),
                color=ball_color,
                label=f"{ball_vel_mmps}",
            ),
        )

    layers = [
        RenderLayer("Robots", robots=tuple(robots)),
        RenderLayer("Velocity vectors", vectors=tuple(velocity_vectors)),
        RenderLayer(
            "Predicted clearance",
            circles=predicted_circles,
            visible_by_default=False,
        ),
        RenderLayer("Ball", circles=ball_circles, vectors=ball_vectors),
    ]

    path_polylines = []
    for path in planner_paths:
        points = tuple(tuple(point[:2]) for point in path.get("points", ()))
        if len(points) < 2:
            continue
        color = YELLOW if path.get("is_yellow", True) else BLUE
        path_polylines.append(
            RenderPolyline(
                points_mm=points,
                color=color,
                closed=False,
            )
        )
    if path_polylines:
        layers.append(
            RenderLayer(
                "Planned paths",
                polylines=tuple(path_polylines),
                visible_by_default=True,
            )
        )

    voronoi_ms = None
    if request.get("include_voronoi", False):
        started_s = time.perf_counter()
        voronoi_map = generate_bounded_voronoi_map(
            placement_mode="density_grid",
            density_percent=float(request.get("density_percent", 10.0)),
            max_density_nodes=int(request.get("max_density_nodes", 80)),
            obstacle_cost_weight=float(request.get("obstacle_cost_weight", 2.0)),
            obstacles=planning_obstacles,
        )
        voronoi_ms = (time.perf_counter() - started_s) * 1000.0
        layers.append(
            voronoi_map.render_layer(
                "Voronoi map",
                visible_by_default=False,
            )
        )

    return MapRenderData(layers=tuple(layers)), voronoi_ms


VoronoiMapWorker = WorldMapRenderWorker
