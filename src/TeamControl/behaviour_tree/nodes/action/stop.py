import py_trees

from TeamControl.behaviour_tree.intent import Intent
from TeamControl.network.robot_command import RobotCommand


class StopRobot(py_trees.behaviours.Success):
    def __init__(self, robot_id, dispatcher_q, name="StopRobot"):
        super(StopRobot, self).__init__(name=name)
        self.bb = py_trees.blackboard.Client(name=name)
        self.dispatcher_q = dispatcher_q
        self.robot_id = robot_id

    def update(self) -> py_trees.common.Status:
        self.intent = Intent.STOP
        robot_command = RobotCommand(
            robot_id=self.robot_id, vx=0, vy=0, w=0, dribble=0, kick=0
        )
        self.dispatcher_q.put(robot_command, 1)
        self.bb.set(f"intent_{self.robot_id}", self.intent)
        return py_trees.common.Status.SUCCESS
