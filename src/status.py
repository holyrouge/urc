from enum import Enum


class Status(Enum):
    RUNNING = 0
    FINISHED = 1
    CRASHED = 2
    STOPPED = 3
