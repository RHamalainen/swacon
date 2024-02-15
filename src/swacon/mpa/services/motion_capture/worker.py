import time
import logging
from typing import Dict
from threading import Lock, Thread

from motioncapture import connect
from scipy.spatial.transform import Rotation

from swacon.mpa.constants import VICON_IP, DRONE_NAME_TO_URI
from swacon.data_structures.drone.state import DroneState

# Used to share results with main thread
DRONE_STATES_LOCK = Lock()
DRONE_STATES: Dict[str, DroneState] = dict()


class MotionCaptureWorker(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.do_continue = True

    def close(self):
        self.do_continue = False

    def run(self):
        logging.info("motion capture worker started")
        mc = connect("vicon", {"hostname": VICON_IP})
        while self.do_continue:
            with DRONE_STATES_LOCK:
                DRONE_STATES.clear()
            mc.waitForNextFrame()
            for name, obj in mc.rigidBodies.items():
                if name in DRONE_NAME_TO_URI.keys():
                    x = obj.position[0]
                    y = obj.position[1]
                    z = obj.position[2]
                    rotation = obj.rotation

                    # self.on_pose[name]([x, y, z, rotation])

                    x = float(x)
                    y = float(y)
                    z = float(z)

                    qx = rotation.x
                    qy = rotation.y
                    qz = rotation.z
                    qw = rotation.w

                    r = Rotation.from_quat([qx, qy, qz, qw])
                    r = r.as_euler("zxy", degrees=True)
                    angle = r[0]

                    p = DroneState(x, y, z, qx, qy, qz, qw, angle)
                    with DRONE_STATES_LOCK:
                        DRONE_STATES[name] = p
            with DRONE_STATES_LOCK:
                print(DRONE_STATES)
            time.sleep(0.5)
        logging.info("motion capture worker finished")
