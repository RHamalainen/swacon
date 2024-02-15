from typing import Any


class DroneState:
    def __init__(self, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float, angle: float) -> None:
        assert isinstance(x, float)
        assert isinstance(y, float)
        assert isinstance(z, float)
        assert isinstance(qx, float)
        assert isinstance(qy, float)
        assert isinstance(qz, float)
        assert isinstance(qw, float)
        assert isinstance(angle, float)
        self.x = x
        self.y = y
        self.z = z
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        # Rotation in quarternions
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw
        # Angle in degrees
        self.angle = angle

    def __eq__(self, other: Any) -> bool:  # noqa: C901
        if not isinstance(other, DroneState):
            return False
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.z != other.z:
            return False
        if self.vx != other.vx:
            return False
        if self.vy != other.vy:
            return False
        if self.vz != other.vz:
            return False
        if self.qx != other.qx:
            return False
        if self.qy != other.qy:
            return False
        if self.qz != other.qz:
            return False
        if self.qw != other.qw:
            return False
        if self.angle != other.angle:
            return False
        return True
