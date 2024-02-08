import sys
import time
import pickle
import logging
import datetime
import traceback
from copy import deepcopy
from typing import List, Dict, Set, Optional, Tuple
from pathlib import Path
from threading import Lock

import cflib
import cflib.crtp
import cflib.crazyflie.swarm
from cflib.utils.power_switch import PowerSwitch
from cflib.crazyflie import Crazyflie as CF
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie as SCF
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

import swacon.spa.constants as constants
import swacon.spa.environment as environment
import swacon.spa.localization as localization
from swacon.algorithms.control.collision_avoidance.safety_filter import safety_filter as safety_filter_unwrapped
from swacon.data_structures.drone.state import DroneState
from swacon.spa.localization import MotionCaptureWorker
from swacon.spa.environment_updater import EnvironmentUpdater


SELECTED_DRONES: List[str] = [
    # "cf1",
    # "cf2",
    # "cf3",
    # "cf4",
    # "cf5",
    # "cf6",
]

USE_VORONOI_ALGORITHM: bool = bool(0)
USE_COLLISION_AVOIDANCE_ALGORITHM: bool = bool(1)

OPERATION_TIME = "finite"
# OPERATION_TIME = "infinite"
OPERATION_TIME_SECONDS = 30.0

TAKE_OFF_TIME_SECONDS = 1.0
LANDING_TIME_SECONDS = 1.0

DRONE_NAME_TO_Z = {
    "cf1": 0.2,
    "cf2": 0.3,
    "cf3": 0.4,
    "cf4": 0.5,
    "cf5": 0.6,
    "cf6": 0.7,
}


def wait_for_position_estimator(scf: SCF, print_progress: bool = False) -> None:
    assert isinstance(scf, SCF)
    assert isinstance(print_progress, bool)
    name = constants.DRONE_URI_TO_NAME[scf.cf.link_uri]
    print(f"[{name}] waiting for estimator to find position")

    log_config = LogConfig(name="Kalman Variance", period_in_ms=500)
    log_config.add_variable("kalman.varPX", "float")
    log_config.add_variable("kalman.varPY", "float")
    log_config.add_variable("kalman.varPZ", "float")

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data["kalman.varPX"])
            var_x_history.pop(0)
            var_y_history.append(data["kalman.varPY"])
            var_y_history.pop(0)
            var_z_history.append(data["kalman.varPZ"])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            dx = max_x - min_x
            dy = max_y - min_y
            dz = max_z - min_z

            if print_progress:
                print(f"{dx} {dy} {dz}")

            if dx < threshold and dy < threshold and dz < threshold:
                break


def reset_estimator(scf: SCF, print_progress: bool = False) -> None:
    assert isinstance(scf, SCF)
    assert isinstance(print_progress, bool)

    # When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90
    # degrees. If this is a problem, increase orientation_std_dev a bit. The default value in the firmware is 4.5e-3.
    orientation_std_dev = 8.0e-3

    # adjust_orientation_sensitivity(cf)
    scf.param.set_value("locSrv.extQuatStdDev", orientation_std_dev)

    # activate_kalman_estimator(cf)
    scf.param.set_value("stabilizer.estimator", "2")
    # Set the std deviation for the quaternion data pushed into the
    # kalman filter. The default value seems to be a bit too low.
    scf.param.set_value("locSrv.extQuatStdDev", 0.06)

    # reset_estimator(cf)
    scf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.1)
    scf.param.set_value("kalman.resetEstimation", "0")

    # time.sleep(1)
    wait_for_position_estimator(scf, print_progress)


def safety_filter(agents: Dict[str, DroneState]) -> Dict[str, Tuple[float, float, float, float]]:
    # Transform agents to suitable format
    # name, x, y, vx, vy
    new_agents: Dict[str, Tuple[float, float, float, float]] = dict()
    for name, position in agents.items():
        x = position.x
        y = position.y
        vx = position.vx
        vy = position.vy
        new_agents[name] = (x, y, vx, vy)
    # TODO: use new interface
    return safety_filter_unwrapped(new_agents)


def update_localization(scf: SCF) -> None:
    assert isinstance(scf, SCF)
    name = constants.DRONE_URI_TO_NAME[scf.cf.link_uri]
    localization.CONNECTED_NAME_TO_DRONE[name] = scf
    reset_estimator(scf.cf)


def get_selected_uris() -> List[str]:
    uris: List[str] = list()
    for name in SELECTED_DRONES:
        uris.append(constants.DRONE_NAME_TO_URI[name])
    return uris


def reboot_drone(uri: str) -> bool:
    name = constants.DRONE_URI_TO_NAME[uri]
    try:
        PowerSwitch(uri).stm_power_cycle()
        return True
    except Exception as exception:
        print(f"failed to reboot drone {name}")
        traceback.print_exc()
        return False


def reboot_all_drones(list_of_uris: List[str]) -> bool:
    print(f"rebooting {len(list_of_uris)} drones")
    successes: List[str] = list()
    failures: List[str] = list()
    for i, uri in enumerate(list_of_uris):
        name = constants.DRONE_URI_TO_NAME[uri]
        if reboot_drone(uri):
            print(f" - {i + 1}/{len(list_of_uris)} rebooting drone {name}")
            successes.append(uri)
        else:
            failures.append(uri)
    if len(successes) == 0:
        print(f"failed to reboot all selected drones")
        return False
    else:
        time.sleep(5.0)
        if 0 < len(failures):
            print(f"failed to reboot subset of selected drones")
            return False
        else:
            return True


def wait_until_parameters_are_downloaded(scf: SCF) -> None:
    assert isinstance(scf, SCF)
    name = constants.DRONE_URI_TO_NAME[scf.cf.link_uri]
    while not scf.cf.param.is_updated:
        time.sleep(1.0)
    print(f"[{name}] downloaded parameters")


def take_off(cf: CF) -> None:
    assert isinstance(cf, CF)
    name = constants.DRONE_URI_TO_NAME[cf.link_uri]
    sleep_time = 0.1
    steps = int(TAKE_OFF_TIME_SECONDS / sleep_time)
    target_z = DRONE_NAME_TO_Z[name]
    vz = target_z / TAKE_OFF_TIME_SECONDS
    for t in range(steps):
        cf.commander.send_velocity_world_setpoint(0.0, 0.0, vz, 0.0)
        time.sleep(sleep_time)


def land(cf: CF) -> None:
    assert isinstance(cf, CF)
    name = constants.DRONE_URI_TO_NAME[cf.link_uri]
    sleep_time = 0.1
    steps = int(LANDING_TIME_SECONDS / sleep_time)
    target_z = DRONE_NAME_TO_Z[name]
    vz = -(target_z / LANDING_TIME_SECONDS)
    for t in range(steps):
        cf.commander.send_velocity_world_setpoint(0.0, 0.0, vz, 0.0)
        time.sleep(sleep_time)
    cf.commander.send_stop_setpoint()
    cf.commander.send_notify_setpoint_stop()
    time.sleep(sleep_time)


def loop_iteration(scf: SCF) -> None:
    assert isinstance(scf, SCF)
    name = constants.DRONE_URI_TO_NAME[scf.cf.link_uri]
    with localization.DRONE_STATES_LOCK:
        drone_states = deepcopy(localization.DRONE_STATES)
    drone_state = drone_states.get(name)
    if drone_state is not None:
        setpoint_x = drone_state.x
        setpoint_y = drone_state.y
        setpoint_z = drone_state.z
        setpoint_vx = drone_state.vx
        setpoint_vy = drone_state.vy
        setpoint_vz = drone_state.vz
        setpoint_angle = drone_state.angle

        if USE_VORONOI_ALGORITHM:
            with environment.ENVIRONMENT_LOCK:
                current_environment = deepcopy(environment.ENVIRONMENT)
            result = coverage.solve_control(
                name=name,
                drone_states=drone_states,
                environment=current_environment,
                envelope=environment.LABORATORY_RASTER_RECTANGLE,
                tiles_in_physical_x=environment.TILES_IN_PHYSICAL_X,
                tiles_in_physical_y=environment.TILES_IN_PHYSICAL_Y,
                k_proportional=1.0,
            )
            if result is not None:
                setpoint_x, setpoint_y, setpoint_vx, setpoint_vy = result
            else:
                # Just stay at the same position
                print(f"[{name}] found no control solution")
        else:
            # Just stay at the same position
            pass

        setpoint_x_old = setpoint_x
        setpoint_y_old = setpoint_y
        setpoint_vx_old = setpoint_vx
        setpoint_vy_old = setpoint_vy

        if USE_COLLISION_AVOIDANCE_ALGORITHM:
            # Update just calculated setpoints for current drone
            drone_states[name].x = setpoint_x
            drone_states[name].y = setpoint_y
            drone_states[name].vx = setpoint_vx
            drone_states[name].vy = setpoint_vy
            # Filter setpoints
            result = safety_filter(drone_states)
            setpoint_x, setpoint_y, setpoint_vx, setpoint_vy = result[name]
            print(f"[{name}] ({round(setpoint_x, 3)}, {round(setpoint_y, 3)}) ({round(setpoint_vx, 3)}, {round(setpoint_vy, 3)})")

        if setpoint_x_old != setpoint_x or setpoint_y_old != setpoint_y or setpoint_vx_old != setpoint_vx or setpoint_vy_old != setpoint_vy:
            print(
                f"[{name}] ({round(setpoint_x_old, 3)}, {round(setpoint_y_old, 3)}, {round(setpoint_vx_old, 3)}, {round(setpoint_vy_old, 3)}) filtered to ({round(setpoint_x, 3)}, {round(setpoint_y, 3)}, {round(setpoint_vx, 3)}, {round(setpoint_vy, 3)})"
            )
        scf.cf.commander.send_velocity_world_setpoint(setpoint_vx, setpoint_vy, 0.0, 0.0)
    else:
        print(f"[{name}] found no state")
        # Just stay at the same position
        scf.cf.commander.send_velocity_world_setpoint(0.0, 0.0, 0.0, 0.0)


def run_sequence(scf: SCF) -> None:
    assert isinstance(scf, SCF)
    name = constants.DRONE_URI_TO_NAME[scf.cf.link_uri]
    try:
        take_off(scf.cf)
        try:
            match OPERATION_TIME:
                case "finite":
                    end_time = time.time() + OPERATION_TIME_SECONDS
                    while time.time() < end_time:
                        loop_iteration(scf)
                        time.sleep(0.1)
                case "infinite":
                    while True:
                        loop_iteration(scf)
                        time.sleep(0.1)
                case other:
                    print(f"invalid operation time: {other}")
        except Exception as exception:
            print(f"[{name}] failed sequence")
            traceback.print_exc()
        finally:
            try:
                land(scf.cf)
            except Exception as exception:
                print(f"[{name}] failed to land")
                traceback.print_exc()
    except Exception as exception:
        print(f"[{name}] failed to take off")
        traceback.print_exc()


def record_all_flight_data() -> None:
    try:
        path_folder = constants.PATH_OUTPUT_FLIGHT_DATA_FOLDER
        timestamp = datetime.datetime.now().isoformat(timespec="seconds")
        path = path_folder / Path(f"{timestamp}.pickle")
        with path.open("wb") as stream:
            with localization.DRONE_STATES_TIME_SERIES_LOCK:
                pickle.dump(localization.DRONE_STATES_TIME_SERIES, stream, protocol=pickle.HIGHEST_PROTOCOL)
    except Exception as exception:
        print(f"failed to record flight data")
        traceback.print_exc()


if __name__ == "__main__":
    # logging.basicConfig(level=logging.DEBUG)
    print(f"settings:")
    print(f" - use voronoi coverage algorithm: {USE_VORONOI_ALGORITHM}")
    print(f" - use collision avoidance algorithm: {USE_COLLISION_AVOIDANCE_ALGORITHM}")
    print(f" - operation time: {OPERATION_TIME}")
    if OPERATION_TIME == "finite":
        print(f"   - operation time in seconds: {OPERATION_TIME_SECONDS}")
    environment.initialize()
    print(f"starting environment updated")
    environment_updater = EnvironmentUpdater()
    environment_updater.start()
    print(f"starting motion capture worker")
    motion_capture_worker = MotionCaptureWorker()
    motion_capture_worker.start()
    try:
        cflib.crtp.init_drivers()
        try:
            # TODO: why this is always empty?
            available_interfaces = cflib.crtp.scan_interfaces()
            if 1:
                # if 0 < len(available_interfaces):
                print(f"detected {len(available_interfaces)} interfaces:")
                for i, interface in enumerate(available_interfaces):
                    print(f"{i + 1}/{len(available_interfaces)}: {interface}")
                try:
                    list_of_uris = get_selected_uris()
                    if 0 < len(list_of_uris):
                        if reboot_all_drones(list_of_uris):
                            try:
                                cf_factory = cflib.crazyflie.swarm.CachedCfFactory(rw_cache="./cache")
                                try:
                                    with cflib.crazyflie.swarm.Swarm(uris=list_of_uris, factory=cf_factory) as swarm:
                                        try:
                                            print(f"waiting until parameters are downloaded")
                                            swarm.parallel_safe(wait_until_parameters_are_downloaded)
                                            try:
                                                print(f"downloaded parameters, updating localization")
                                                swarm.parallel_safe(update_localization)
                                                try:
                                                    print(f"updated localization, running sequence")
                                                    swarm.parallel_safe(run_sequence)
                                                    print(f"finished sequence")
                                                except Exception as exception:
                                                    print(f"failed run sequence")
                                                    traceback.print_exc()
                                            except Exception as exception:
                                                print(f"failed update localization")
                                                traceback.print_exc()
                                        except Exception as exception:
                                            print(f"failed download all parameters")
                                            traceback.print_exc()
                                    print(f"finished script")
                                except Exception as exception:
                                    print(f"failed to create crazyflie swarm")
                                    traceback.print_exc()
                            except Exception as exception:
                                print(f"failed to create crazyflie factory")
                                traceback.print_exc()
                        else:
                            print(f"failed to reboot all drones")
                    else:
                        print(f"no drones were selected")
                except Exception as exception:
                    print(f"failed to reboot all drones")
                    traceback.print_exc()
            else:
                print(f"no available interfaces")
        except Exception as exception:
            print(f"failed to initialize cflib drivers")
            traceback.print_exc()
    except Exception as exception:
        print(f"failed to initialize cflib drivers")
        traceback.print_exc()
    finally:
        try:
            environment_updater.close()
            print(f"closed environment updater thread")
        except Exception as exception:
            print(f"failed to close environment updater thread")
            traceback.print_exc()
        try:
            motion_capture_worker.close()
            print(f"closed motion capture thread")
        except Exception as exception:
            print(f"failed to close motion capture thread")
            traceback.print_exc()
        try:
            record_all_flight_data()
        except Exception as exception:
            print(f"failed to record all flight data")
            traceback.print_exc()
