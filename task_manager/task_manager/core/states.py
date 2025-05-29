from enum import Enum, auto

class TaskState(Enum):
    IDLE = auto()
    INITIALIZING = auto()
    MOVING_HOME = auto()
    MOVING_TO_OVERVIEW = auto()
    SEARCHING = auto()
    PROCESSING_DETECTION = auto()
    POINTING = auto()
    RETURNING_HOME = auto()
    COMPLETED = auto()
    ERROR = auto()
    PAUSED = auto()