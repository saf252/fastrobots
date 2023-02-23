from enum import Enum, auto

class CMD(Enum):
    def _generate_next_value_(name, start, count, last_values):
        return count
    GET_IMU_TIME = auto()
    BUF_ALL_TIME = auto()
    GET_IMU_BUF = auto()
    GET_TOF_BUF = auto()
    START_RECORDING_STREAM = auto()
    STOP_RECORDING_STREAM = auto()