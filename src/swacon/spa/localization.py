import time
import logging
import datetime
from copy import deepcopy
from typing import List, Dict, Optional, Tuple
from threading import Lock, Thread

from scipy.spatial.transform import Rotation
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie as SCF

import swacon.spa.constants as constants
from swacon.data_structures.drone.state import DroneState


# Used to share results with main thread
DRONE_STATES_LOCK = Lock()
DRONE_STATES: Dict[str, DroneState] = dict()
DRONE_PREVIOUS_STATES: Dict[str, DroneState] = dict()

DRONE_STATES_TIME_SERIES_LOCK = Lock()
DRONE_STATES_TIME_SERIES: Dict[str, List[Tuple[str, Optional[DroneState]]]] = dict()

CONNECTED_NAME_TO_DRONE: Dict[str, SCF] = dict()


class MotionCaptureWorker(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.do_continue = True

    def close(self):
        self.do_continue = False

    def run(self):
        from motioncapture import connect

        logging.info("motion capture worker started")
        mc = connect("vicon", {"hostname": constants.VICON_IP})
        while self.do_continue:
            with DRONE_STATES_LOCK:
                DRONE_STATES.clear()
            mc.waitForNextFrame()
            timestamp = datetime.datetime.now().isoformat(timespec="milliseconds")
            for name, obj in mc.rigidBodies.items():
                if name in constants.DRONE_NAME_TO_URI.keys():
                    x = obj.position[0]
                    y = obj.position[1]
                    z = obj.position[2]
                    rotation = obj.rotation

                    x = float(x)
                    y = float(y)
                    z = float(z)

                    qx = rotation.x
                    qy = rotation.y
                    qz = rotation.z
                    qw = rotation.w

                    if name in CONNECTED_NAME_TO_DRONE.keys():
                        CONNECTED_NAME_TO_DRONE[name].cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)

                    r = Rotation.from_quat([qx, qy, qz, qw])
                    r = r.as_euler("zxy", degrees=True)
                    angle = r[0]

                    p = DroneState(x, y, z, qx, qy, qz, qw, angle)
                    if name in DRONE_STATES.keys():
                        if name in DRONE_PREVIOUS_STATES.keys():
                            # Previous position
                            pp = DRONE_PREVIOUS_STATES[name]
                            vx = p.x - pp.x
                            vy = p.y - pp.y
                            vz = p.z - pp.z
                            p.vx = vx
                            p.vy = vy
                            p.vz = vz
                            DRONE_PREVIOUS_STATES[name] = DRONE_STATES[name]
                        else:
                            DRONE_PREVIOUS_STATES[name] = p
                    with DRONE_STATES_LOCK:
                        DRONE_STATES[name] = p

                    with DRONE_STATES_TIME_SERIES_LOCK:
                        if name not in DRONE_STATES_TIME_SERIES.keys():
                            DRONE_STATES_TIME_SERIES[name] = list()
                        DRONE_STATES_TIME_SERIES[name].append((deepcopy(timestamp), deepcopy(p)))
            time.sleep(0.01)
        logging.info("motion capture worker finished")
