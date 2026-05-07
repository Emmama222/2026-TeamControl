import numpy as np
import py_trees

# default rule threshould 150


class MoveAwayRobot(py_trees.behaviour.Behaviour):
    def __init__(self, robot_id: int, ball, distance=150):
        name = "MoveAwayRobot" + str(robot_id)
        super(MoveAwayRobot, self).__init__(name=name)
        self.bb = py_trees.blackboard.Client(name=name)
        self.robot_id = robot_id
        self.ball = ball
        self.distance = distance

    def update(self, dt: float) -> py_trees.common.Status:
        robot_pos = self.bb.get_variable(f"robot_pos_{self.robot_id}")
        ball_pos = self.bb.get_variable(f"ball_pos")
        target_pos = move_away_robot_from(robot_pos, ball_pos, self.distance)
        self.bb.set_variable(f"target_pos_{self.robot_id}", target_pos)
        self.bb.set_variable(f"Intent{self.robot_id}", "GOTO")
        return py_trees.common.Status.SUCCESS


def move_away_robot_from(robot_pos, target_pos, threshold=150):
    robot_pos = robot_pos[:2]
    direction_2d = robot_pos - target_pos
    distance = np.linalg.norm(direction_2d)
    # if it is on top of target
    if distance == 0:
        # On top of the target. Pick arbitrary direction to move away.
        unit_vec = np.array([1.0, 0.0])
        distance_to_move = threshold
    elif distance < threshold:
        unit_vec = direction_2d / distance
        distance_to_move = threshold - distance
    else:
        return robot_pos  # no need to move

    # calculate the target position for robot to get to
    target_pos = robot_pos + unit_vec * distance_to_move

    return target_pos


if __name__ == "__main__":
    robot_pos = np.array([0, 100, 0])
    target_pos = np.array([50, 100])
    new_pos = move_away_robot_from(robot_pos, target_pos, threshold=150)
    print(new_pos)
