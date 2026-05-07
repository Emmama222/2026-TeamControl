import py_trees

from TeamControl.SSL.game_controller.common import GameState


class IsRunning(py_trees.behaviour.Behaviour):
    def __init__(self):
        name = "IsRunning"
        super(IsRunning, self).__init__(name)
        self.bb = py_trees.blackboard.Client(name=name)
        # initialise isRunning to false
        self.isRunning = False
        # print("Debug: IsRunning initialized")

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.bb.register_key(key="game_state", access=py_trees.common.Access.READ)

    def update(self):
        self.isRunning = self.bb.game_state == GameState.RUNNING
        if self.isRunning:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class IsStopped(py_trees.behaviour.Behaviour):
    def __init__(self):
        name = "IsStopped"
        super(IsStopped, self).__init__(name)
        self.bb = py_trees.blackboard.Client(name=name)
        # initialise isStopped to false
        self.isStopped = False

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.bb.register_key(key="game_state", access=py_trees.common.Access.READ)

    def update(self):
        self.isStopped = self.bb.game_state == GameState.STOPPED
        if self.isStopped:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class IsHalted(py_trees.behaviour.Behaviour):
    def __init__(self):
        name = "IsHalted"
        super(IsHalted, self).__init__(name)
        self.bb = py_trees.blackboard.Client(name=name)
        # initialise isHalted to false
        self.isHalted = False

    def setup(self, **kwargs):
        # print(f"{self.name} setup")
        super().setup(**kwargs)
        self.bb.register_key(key="game_state", access=py_trees.common.Access.READ)

    def update(self):
        self.isHalted = self.bb.game_state == GameState.HALTED
        if self.isHalted:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class IsOurKickoff(py_trees.behaviour.Behaviour):
    def __init__(self):
        name = "IsOurKickoff"
        super(IsOurKickoff, self).__init__(name)
        self.bb = py_trees.blackboard.Client(name=name)
        # initialise isKickoff to false
        self.isOurKickoff = False

    def update(self):
        self.isOurKickoff = self.bb.game_state == GameState.KICKOFF
        if self.isOurKickoff:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
