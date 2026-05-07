from enum import Enum, auto


class GameState(Enum):
    HALTED = auto()
    STOPPED = auto()
    RUNNING = auto()
    OUR_PREPARE_KICKOFF = auto()

    OUR_FREE_KICK = auto()
    OUR_BALL_PLACEMENT = auto()
    OUR_KICKOFF = auto()

    HALF_TIME = auto()
    OUR_TIME_OUT = auto()

    OUR_PENALTY_SHOOT = auto()
    OUR_PENALTY_DEFEND = auto()
