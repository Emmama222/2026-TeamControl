import py_trees
from py_trees.common import Status

from TeamControl.behaviour_tree.nodes.action import StopRobot, Move
from TeamControl.behaviour_tree.intent import Intent as IntentEnum

class StopSequence(py_trees.composites.Sequence):
    def __init__(self, robot_id, dispatcher_q):
        super(StopSequence, self).__init__(
            name=f"StopSequence (RobotID:{robot_id})", memory=False
        )
        self.robot_id = robot_id
        self.dispatcher_q = dispatcher_q
        # self.bb = py_trees.blackboard.Client(name="StopSequence")
        self.add_children(
            [
                StopRobot(robot_id=self.robot_id, dispatcher_q=self.dispatcher_q),
                MoveAway(150),
            ]
        )

