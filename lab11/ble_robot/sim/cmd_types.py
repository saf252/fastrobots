from enum import Enum, auto

class CMD(Enum):
    def _generate_next_value_(name, start, count, last_values):
        return count
    RUN_MAP = auto()
    DATA_MAP = auto()
    PID_MAP = auto()
    JUST_MAP = auto()
    ROTATE = auto()
    DRIVE = auto()
    CALIBRATE = auto()