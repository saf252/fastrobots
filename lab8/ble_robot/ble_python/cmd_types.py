from enum import Enum, auto

class CMD(Enum):
    def _generate_next_value_(name, start, count, last_values):
        return count
    DRIVE = auto()
    OPEN_LOOP = auto()
    RUN_PID = auto()
    DATA_PID = auto()
    EXTRA_DATA_PID = auto()
    STREAM_TOF1 = auto()
    RUN_STUNT = auto()
    DATA_STUNT = auto()