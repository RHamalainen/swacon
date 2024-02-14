from enum import Enum, auto, unique
from typing import Optional


class Command:
    def __init__(self) -> None:
        pass


class CommandQuit(Command):
    def __init__(self) -> None:
        super().__init__()


class CommandReset(Command):
    def __init__(self) -> None:
        super().__init__()


class CommandTakeoff(Command):
    def __init__(self) -> None:
        super().__init__()


class CommandLand(Command):
    def __init__(self) -> None:
        super().__init__()


class CommandState(Command):
    def __init__(self) -> None:
        super().__init__()


@unique
class TurnDirection(Enum):
    LEFT = auto()
    RIGHT = auto()


class CommandTurn(Command):
    def __init__(self, direction: TurnDirection, angle: float) -> None:
        assert isinstance(direction, TurnDirection)
        assert isinstance(angle, float)
        super().__init__()
        self.direction = direction
        self.angle = angle


@unique
class PositionSetpointType(Enum):
    ABSOLUTE = auto()
    RELATIVE = auto()


class CommandPositionSetpoint(Command):
    def __init__(self, x: float, y: float, z: Optional[float], setpoint_type: PositionSetpointType) -> None:
        assert isinstance(x, float)
        assert isinstance(y, float)
        if z is not None:
            assert isinstance(z, float)
        assert isinstance(setpoint_type, PositionSetpointType)
        super().__init__()
        self.x = x
        self.y = y
        self.z = z
        self.setpoint_type = setpoint_type
