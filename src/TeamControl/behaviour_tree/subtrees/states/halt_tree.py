## halt state sequence
import py_trees

from TeamControl.behaviour_tree.actions.stop_robot import StopRobot


class HaltSequence(py_trees.composites.Sequence):
    def __init__(self, robot_id, dispatcher_q):
        super(HaltSequence, self).__init__(
            name=f"HaltSequence (RobotID:{robot_id})", memory=True
        )
        self.robot_id = robot_id
        self.bb = py_trees.blackboard.Client(name=self.name)

        self.add_child(StopRobot(robot_id=self.robot_id, dispatcher_q=dispatcher_q))
