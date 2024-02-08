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
