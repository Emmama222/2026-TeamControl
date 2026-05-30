from typing import Optional
from TeamControl.world.map.obstacles import Obstacles

R = 90.0 # mm -  robot Radius
class WorldMap:
    def __init__(self, horizon_ms=20,field=None, snapshot=None) -> None:
        self.horizon_ms = horizon_ms #ms
        self.update(snapshot,field)

    def update(self, snapshot,field=None) -> None:
        if field is not None:
            self.field = self._create_field(field)
        if snapshot is not None:
            self.obs = self._create_obs_from_snap(snapshot)
        

    def _create_field(self,field):
        pass
    
    def _create_obs_from_snap(self,snapshot):
        pass
        
    def get_render_data(self):
        pass

    def get_all_obs(self):
        return self.obs

    def find_closest_robot(self,start_pos, team_is_yellow:Optional[bool]=None) -> tuple[int, list[float]]:
        """locate the closest robot to start_pos base (optional: on team color
        """

        robot_id = 0
        robot_pos = [0.0, 0.0, 0.0]
        return (robot_id, robot_pos)  # return the closest robotid, located at pos

    def get_nearby_teammates(
        self, robot_id: int, team_is_yellow: bool, distance: int = 500
    ) -> list[int]:
        """
        Base on current RobotID and TeamIsYellow, within the distance range, get the sorted nearby teammate
        returns a list of sorted teammates robot_ids base on location relative to robot
        """
        return []

    def get_nearby_enemy(self, robot_id: int, team_is_yellow: bool, distance: int = 500) -> list[int]:
        """
        Base on current RobotID and TeamIsYellow, within the distance range, get the sorted nearby enemy
        returns a list of sorted enemies robot_ids base on location relative to robot
        defaults 50 mm
        """
        return []
    
    def distance_2_segment(
            point: tuple[float, float],
            start: tuple[float, float],
            end: tuple[float, float],
        ) -> float:
            """Return shortest distance from point to line segment start-end."""
            px, py = point
            sx, sy = start
            ex, ey = end

            dx = ex - sx
            dy = ey - sy

            # Start and end are the same point
            if dx == 0 and dy == 0:
                return hypot(px - sx, py - sy)

            # Project point onto line segment
            t = ((px - sx) * dx + (py - sy) * dy) / (dx * dx + dy * dy)

            # Clamp projection to segment
            t = max(0.0, min(1.0, t))

            closest_x = sx + t * dx
            closest_y = sy + t * dy

            return hypot(px - closest_x, py - closest_y)
    
    def is_path_free(self, start_pos, end_pos, ignored:set[int,bool]|None=None, clearance:int=0) -> bool:
        
        """
        Return True if no obstacle overlaps the robot path corridor.

        clearance:
            Extra required clearance in mm.
            0 means just avoid physical/safe-radius collision.
        """
        if ignored is None:
            ignored = set()

        for obs in self.obstacles:
            if (obs.id,obs.team_is_yellow) in ignored:
                continue
            dist = self.distance_2_segment(point = obs.pos_mm, start=start_pos,end=end_pos )
            actual_clearance = dist -R- obs.safe_radius
            if max_clearance < clearance: 
                return False
        
        return True

    def is_target_in_box(self, target_pos, x_lim: int|float, y_lim: int|float, offset=R) -> bool:
        """checks if the target is within a box"""
        return True
