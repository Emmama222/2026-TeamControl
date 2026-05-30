class FieldAnalyzer:
	def __init__(self, map) -> None:
		self.map = map
		pass

	def is_path_blocked(self, start_xy, goal_xy, clearance_radius, horizon_ms) -> bool:
		return True  # if it path width is shorter than robot clearance

	def score_confidence(self, robot_id, team_is_yellow) -> float:
		"""
		base on the distance, facing angle, does robot has direct path to shoot, nearby enemies
		evaluate to be the final score confidence
		default = 1.0 max

		"""
		return 1.0

	def pass_confidence(self, robot_id1, robot_id2, team_is_yellow) -> float:
		"""
		base on the distance between, facing angle, direct path, nearby enemies, distance to goal between the robots,
		Evaluate to be the final total score confidence estimate.
		Default = 1.0 max
		"""
		return 1.0

	def ball_trajectory(self, horizon_ms=20) -> tuple[tuple[float, float], tuple[float, float]]:
		"""
		returns the ball traectory in horizon_ms
		Default : 20ms
		"""
		pos, vel = self.map.get_ball_trajectory(horizon_ms)
		return pos, vel

	def robot_trajectory(self, robot_id, team_is_yellow, horizon_ms=20) -> tuple[tuple[float, float], tuple[float, float]]:
		"""
		returns the robot trajectory in horizon_ms
		Default : 20ms
		"""
		pos, vel = self.map.get_robot_trajectory(robot_id, team_is_yellow, horizon_ms)
		return pos, vel
