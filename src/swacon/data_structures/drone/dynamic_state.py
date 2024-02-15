# TODO: terrible naming :)

from enum import unique, Enum, auto


@unique
class DroneDynamicState(Enum):
    LANDED = auto()
    TAKING_OFF = auto()
    HOVERING = auto()
    LANDING = auto()
    MOVING = auto()
