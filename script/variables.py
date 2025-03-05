from enum import IntEnum

class RobotState(IntEnum):
    START = 0
    TAKEOFF = 1
    LAND = 2
    HOVER = 3
    APPROACH = 4
    PALM_LAND_READY = 5
    PALM_LAND = 6
    STOP = 7
