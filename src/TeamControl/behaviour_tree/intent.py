from enum import Enum, auto


class Intent(Enum):
    ENTER_FIELD = auto()
    EXIT_FIELD = auto()
    MOVE_TO_POSITION = auto()
    SCORING = auto()
    STOP = auto()
    PASS = auto()
    RECEIVE = auto()
