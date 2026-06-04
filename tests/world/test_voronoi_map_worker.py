from multiprocessing import Event, Queue

from TeamControl.process_workers.voronoi_map_runner import WorldMapRenderWorker
from TeamControl.world.map.renderer import MapRenderData
from TeamControl.world.map.voronoi_generator import VoronoiObstacle


def test_world_map_render_worker_generates_complete_render_data():
    request_q = Queue()
    response_q = Queue()
    request_q.put(
        {
            "request_id": 1,
            "density_percent": 10,
            "max_density_nodes": 80,
            "planning_obstacles": (
                VoronoiObstacle((0.0, 0.0), radius_mm=120.0, label="obs"),
            ),
            "include_voronoi": True,
            "ball": (100.0, 200.0),
            "ball_visible": True,
            "planner_paths": (
                {
                    "robot_id": 0,
                    "is_yellow": True,
                    "points": ((0.0, 0.0), (100.0, 0.0)),
                    "timestamp_s": 1.0,
                },
            ),
        }
    )

    worker = WorldMapRenderWorker(Event(), logger=None)
    worker.setup(request_q, response_q)
    worker.step()

    response = response_q.get(timeout=1)
    assert response["request_id"] == 1
    assert response["generation_ms"] >= 0.0
    assert response["voronoi_generation_ms"] >= 0.0
    assert isinstance(response["render_data"], MapRenderData)
    assert response["render_data"].layer("Robots") is not None
    assert response["render_data"].layer("Ball") is not None
    assert response["render_data"].layer("Planned paths") is not None
    assert response["render_data"].layer("Voronoi map") is not None
