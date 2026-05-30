from dataclasses import dataclass
from math import hypot

## Standard Robot Radius
R = 90.0  # mm
MARGIN = 30  #mm
BALL_R =  21.5 #mm

@dataclass
class Obstacles:
    robot_id: int
    team_is_yellow: bool
    pos_mm: tuple[float, float, float]  # mm
    vel_mmps: tuple[float, float]  # mm/s
    @property
    def radius(self) -> float:
        """The physical radius of robot"""
        return R # hard radius
    @property
    def safe_radius(self):
        return self.radius + MARGIN
    
    @property
    def speed_mmps(self) -> float:
        """ speed scalar value in mm/s """
        return hypot(self.vel[0], self.vel[1])

    @property
    def speed_mps(self) -> float:
        """ speed scalar value in m/s """
        return self.speed_mmps/1000

    def predicted_pos(self, horizon_ms: int) -> tuple[float, float]:
        """
    	Returns the predicted position of the robot after a given horizon in milliseconds.
    	"""
        dt_s = horizon_ms/1000
        return (
            self.pos_mm[0] + self.vel_mmps[0] * dt_s,
            self.pos_mm[1] + self.vel_mmps[1] * dt_s,
        )

    def dynamic_radius(self, horizon_ms: int) -> float:
        """Returns the variated radius of the robot after a given horizon in milliseconds."""
        dt_s = horizon_ms/1000
        return self.radius + self.speed_mmps() * dt + MARGIN

    def vector_to(self, target_pos: tuple[float, float]) -> tuple[float, float]:
        """ returns a position vector [dx,dy] (in mm) from Obstacle to target"""
        dx = target_pos[0] - self.pos_mm[0]
        dy = target_pos[1] - self.pos_mm[1]
        return (dx, dy)

    def dist_to(self, target_pos: tuple[float, float]) -> float:
        """ returns a scalar distance (in mm) from Obstacle to target
        """
        return hypot(*self.vector_to(target_pos))

    def posses_ball(self, ball_pos) -> bool:
        """ returns True if the ball is within the robot's radius
        """
        if self.dist_to(ball_pos) < self.radius + MARGIN:
            return True
        return False

# Todo : add a  test to use snapshot -> generate obstacle object
if __name__ == "__main__":
    o1 = Obstacles(0, True, [0, 0, 0], [2, 1])
    print(o1.predicted_pos(10))
