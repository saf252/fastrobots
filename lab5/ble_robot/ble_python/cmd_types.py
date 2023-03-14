from enum import Enum, auto

class CMD(Enum):
    def _generate_next_value_(name, start, count, last_values):
        return count
    DRIVE = auto()
    OPEN_LOOP = auto()