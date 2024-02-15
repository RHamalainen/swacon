import os
import sys
import time
import signal
import traceback
from queue import Queue
from typing import Dict, Optional, Any
from pathlib import Path
from logging import getLogger, DEBUG as LOGGING_LEVEL_DEBUG
from threading import Thread, Event

import rpyc
from rpyc import Service, ThreadedServer, Connection, discover, connect
from rpyc.utils.registry import TCPRegistryClient
from rpyc.utils.factory import DiscoveryError
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie
from cflib.utils.power_switch import PowerSwitch
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from shapely import Point

import swacon.mpa.services.common.service_configuration  # noqa: F401
from swacon.mpa.constants import LABORATORY_PHYSICAL_RECTANGLE
from swacon.data_structures.drone.state import DroneState
from swacon.data_structures.drone.dynamic_state import DroneDynamicState
from swacon.data_structures.drone.command import Command, CommandLand, CommandPositionSetpoint, CommandQuit, CommandReset, CommandState, CommandTakeoff, CommandTurn, PositionSetpointType


# FIXME: all drone services share the same aliases, how to distinguish e.g. drone cf1 from cf2?


CF_STATE_TO_STRING = {
    0: "disconnected",
    1: "initialized",
    2: "connected",
    3: "setup_finished",
}


# TODO: maybe decompose into drone and service
class DroneService(Service):
    def __init__(self, name: str, uri: str, no_motion_capture: bool, no_radio: bool) -> None:
        assert isinstance(name, str)
        assert isinstance(uri, str)
        assert isinstance(no_motion_capture, bool)

        self.name = name
        self.uri = uri
        self.no_motion_capture = no_motion_capture
        self.no_radio = no_radio
        self.ALIASES = [self.name]
        super().__init__()
        self.logger = self.setup_logging(self.name)
        self.queue: Queue[Command] = Queue()
        self.drone_logger_battery = LogConfig("Battery", 1000)
        self.drone_logger_battery.add_variable("pm.vbat", "float")
        self.drone_logger_battery.add_variable("pm.state", "int8_t")
        # self.drone_logger_battery.add_variable("supervisor.info", "uint16_t")
        self.drone_logger_state_estimate = LogConfig("StateEstimate", 1000)
        self.drone_logger_state_estimate.add_variable("stateEstimate.x", "float")
        self.drone_logger_state_estimate.add_variable("stateEstimate.y", "float")
        self.drone_logger_state_estimate.add_variable("stateEstimate.z", "float")
        self.drone_logger_state_estimate.add_variable("stateEstimate.vx", "float")
        self.drone_logger_state_estimate.add_variable("stateEstimate.vy", "float")
        self.drone_logger_state_estimate.add_variable("stateEstimate.vz", "float")
        if self.no_radio:
            self.scf = None
        else:
            self.scf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache="./cache"))
            self.scf.cf.connected.add_callback(self.callback_connected)
            self.scf.cf.connection_failed.add_callback(self.callback_connection_failed)
            self.scf.cf.connection_lost.add_callback(self.callback_connection_lost)
            # self.scf.cf.connection_requested.add_callback()
            # self.scf.cf.log.add_config(self.drone_logger)
            # self.drone_logger.data_received_cb.add_callback(self.callback_data_received)
            # self.drone_logger.error_cb.add_callback(self.callback_logger_error)
            self.scf.cf.disconnected.add_callback(self.callback_disconnected)
            self.scf.cf.fully_connected.add_callback(self.callback_fully_connected)
            self.scf.cf.link_established.add_callback(self.callback_link_established)
            self.scf.cf.link_quality_updated.add_callback(self.callback_link_quality_updated)
            # self.scf.cf.add_port_callback()
            state = self.scf.cf.state
            string = CF_STATE_TO_STRING[state]
            self.logger.info(f"drone state: {string}")

        self.takeoff_z = 0.3
        self.landing_z = 0.0

        self.server: Optional[ThreadedServer] = None

        if self.no_motion_capture:
            self.mcc = None
        else:
            self.mcc = self.connect_to_motion_capture_service()
            if self.mcc is None:
                self.logger.critical(f"failed to establish connection to motion capture service")
                self.kill_service()
                return

        self.failures_to_get_position_estimate = 0
        self.position_estimate_failure_threshold = 5
        self.handle_update_finished = Event()
        self.stop_handle_update = Event()
        self.handle_update = Thread(target=self.update_thread, name=f"{self.name}_update_thread")
        self.handle_update.start()

        self.handle_control_finished = Event()
        self.stop_handle_control = Event()
        self.handle_control = Thread(target=self.control_thread, name=f"{self.name}_control_thread")
        self.handle_control.start()

        self.stop_handle_monitor = Event()
        self.handle_monitor = Thread(target=self.monitor_thread, name=f"{self.name}_monitor_thread")
        self.handle_monitor.start()

        self.reseting = False

        self.connected = False

        self.battery_voltage_threshold = 3.75

    def callback_disconnected(self, uri: str) -> None:
        assert isinstance(uri, str)
        if self.reseting:
            self.logger.info(f"disconnected as part of the reset procedure")
        else:
            self.logger.critical(f"disconnected from {uri}")
            self.kill_process()
        self.connected = False

    def callback_fully_connected(self, uri: str) -> None:
        assert isinstance(uri, str)
        self.logger.info(f"fully connected to {uri}")
        self.connected = True

    def callback_link_established(self, uri: str) -> None:
        assert isinstance(uri, str)
        self.logger.info(f"link established to {uri}")

    def callback_link_quality_updated(self, quality: float) -> None:
        assert isinstance(quality, float)
        # TODO: when link quality is too bad?

    def callback_data_received_battery(self, a: int, b: Dict[str, Any], c: LogConfig) -> None:
        assert isinstance(a, int)
        assert isinstance(b, dict)
        assert isinstance(c, LogConfig)
        try:
            battery_voltage = b["pm.vbat"]
            assert isinstance(battery_voltage, float)
            if battery_voltage <= self.battery_voltage_threshold:
                battery_voltage = round(battery_voltage, 2)
                self.logger.critical(f"battery voltage {battery_voltage} is under {self.battery_voltage_threshold}")
                self.kill_service()
            else:
                battery_voltage = round(battery_voltage, 2)
                self.logger.info(f"battery voltage is {battery_voltage}")
        except KeyError:
            self.logger.error(f"could not get battery voltage")
        try:
            supervisor_info = b["supervisor.info"]
            assert isinstance(supervisor_info, int)

            binary = f"{supervisor_info:032b}"
            # self.logger.info(f"supervisor.info = {supervisor_info} = {binary}")
            bit0 = binary[0] == "1"
            bit1 = binary[1] == "1"
            bit2 = binary[2] == "1"
            bit3 = binary[3] == "1"
            bit4 = binary[4] == "1"
            bit5 = binary[5] == "1"
            bit6 = binary[6] == "1"

            can_be_armed = bit0
            is_armed = bit1
            automatically_armed = bit2
            can_fly = bit3
            is_flying = bit4
            is_tumbled = bit5
            is_locked = bit6

            self.logger.info(
                f"can be armed: {can_be_armed}, is armed: {is_armed}, auto armed: {automatically_armed}, can fly: {can_fly}, is flying: {is_flying}, is tumbled: {is_tumbled}, is locked: {is_locked}"
            )
        except KeyError:
            pass
            # self.logger.error(f"could not get supervisor information")

    def callback_logger_error_battery(self, data) -> None:
        print(data)

    def callback_data_received_state_estimate(self, a: int, b: Dict[str, Any], c: LogConfig) -> None:
        assert isinstance(a, int)
        assert isinstance(b, dict)
        assert isinstance(c, LogConfig)
        x = b["stateEstimate.x"]
        y = b["stateEstimate.y"]
        z = b["stateEstimate.z"]
        vx = b["stateEstimate.vx"]
        vy = b["stateEstimate.vy"]
        vz = b["stateEstimate.vz"]

        x = round(x, 2)
        y = round(y, 2)
        z = round(z, 2)
        vx = round(vx, 2)
        vy = round(vy, 2)
        vz = round(vz, 2)
        self.logger.debug(f"SE: x=({x}, {y}, {z}) v=({vx}, {vy}, {vz})")

    def callback_logger_error_state_estimate(self, data) -> None:
        print(data)

    def callback_connected(self, uri: str) -> None:
        assert isinstance(uri, str)
        self.logger.info(f"connected to {uri}")

        self.scf.cf.log.reset()

        self.scf.cf.log.add_config(self.drone_logger_battery)
        self.drone_logger_battery.data_received_cb.add_callback(self.callback_data_received_battery)
        self.drone_logger_battery.error_cb.add_callback(self.callback_logger_error_battery)
        self.drone_logger_battery.start()

        self.scf.cf.log.add_config(self.drone_logger_state_estimate)
        self.drone_logger_state_estimate.data_received_cb.add_callback(self.callback_data_received_state_estimate)
        self.drone_logger_state_estimate.error_cb.add_callback(self.callback_logger_error_state_estimate)
        self.drone_logger_state_estimate.start()

        self.connected = True

    def callback_connection_failed(self, uri: str, error: str) -> None:
        assert isinstance(uri, str)
        assert isinstance(error, str)
        self.logger.error(f"connection failed to {uri}: {error}")
        self.connected = False

    def callback_connection_lost(self, uri: str, error: str) -> None:
        assert isinstance(uri, str)
        assert isinstance(error, str)
        self.logger.error(f"lost connection to {uri}: {error}")
        self.connected = False

    def setup_logging(self, file_name: str):
        from logging import StreamHandler, FileHandler, Formatter

        assert isinstance(file_name, str)
        # Log to console and file
        log_path = Path(__file__).parent.parent / f"logs/{file_name}"
        formatter_string = "{levelname}|{threadName}|{name}|{asctime}|{funcName}|{lineno}|{message}"
        formatter_console = Formatter(formatter_string, style="{")
        handler_console = StreamHandler(sys.stdout)
        handler_console.setLevel(LOGGING_LEVEL_DEBUG)
        handler_console.setFormatter(formatter_console)
        formatter_file = Formatter(formatter_string, style="{")
        handler_file = FileHandler(log_path, "w")
        handler_file.setLevel(LOGGING_LEVEL_DEBUG)
        handler_file.setFormatter(formatter_file)
        logger = getLogger(f"{self.name}_logger")
        logger.propagate = False
        # logger.handlers.clear()
        logger.addHandler(handler_console)
        logger.addHandler(handler_file)
        logger.setLevel(LOGGING_LEVEL_DEBUG)
        logger.debug(f"logger initialized")
        return logger

    def kill_process(self):
        pid = os.getpid()
        os.kill(pid, signal.SIGTERM)

    def kill_service(self):
        self.disconnect_from_drone()
        self.kill_process()

    def exposed_stop_service(self):
        self.kill_service()

    def exposed_send_command(self, command: Command):
        assert isinstance(command, Command)
        self.queue.put(command)
        self.logger.info(f"received new command, current queue length is {self.queue.qsize()}")

    def connect_to_motion_capture_service(self) -> Optional[Connection]:
        try:
            service = "motioncapture"
            registrar = TCPRegistryClient("localhost")
            providers = discover(service, "localhost", registrar)
            if len(providers) == 1:
                provider = providers[0]
                host, port = provider
                connection = connect(host, port, keepalive=True)
                assert isinstance(connection, Connection)
                return connection
            elif 1 < len(providers):
                self.logger.error(f"found too many ({len(providers)}) for service {service}")
                return None
            # TODO: check is this correct
            return None
        except DiscoveryError as error:
            self.logger.error(f"{error}")
            return None

    def clear_command_queue(self) -> None:
        while not self.queue.empty():
            _command = self.queue.get()  # noqa: F841
        self.logger.info(f"command queue cleared")

    def reboot_drone(self) -> None:
        self.logger.info(f"reboot starts")
        try:
            PowerSwitch(self.uri).stm_power_cycle()
            time.sleep(5)
            self.logger.info(f"reboot finished")
        except Exception as error:
            self.logger.critical(f"reboot failed: {error}")
            self.kill_service()

    def connect_to_drone(self) -> None:
        if self.scf.is_link_open():
            self.logger.info(f"already connected")
        else:
            self.logger.info(f"not connected, connecting")
            try:
                self.scf.open_link()
            except Exception as error:
                self.logger.critical(f"failed to connect to drone: {error}")
                self.kill_service()
            self.logger.info(f"now connected, updating parameters")
            self.scf.wait_for_params()
            self.logger.info(f"parameters updated")

    def disconnect_from_drone(self) -> None:
        if self.scf.is_link_open():
            self.logger.info(f"connected, disconnecting")
            self.scf.close_link()
            self.logger.info(f"now disconnected")
        else:
            self.logger.info(f"already disconnected")

    def reset_estimator(self) -> bool:
        """Reset drone's state estimator

        ### Result
        - `True` if drone's state estimator reset succeeded
        - `False` if drone's state estimator reset failed
        """

        # When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90
        # degrees. If this is a problem, increase orientation_std_dev a bit. The default value in the firmware is 4.5e-3.
        orientation_std_dev = 8.0e-3

        # adjust_orientation_sensitivity(cf)
        self.scf.cf.param.set_value("locSrv.extQuatStdDev", orientation_std_dev)

        # activate_kalman_estimator(cf)
        self.scf.cf.param.set_value("stabilizer.estimator", "2")
        # Set the std deviation for the quaternion data pushed into the
        # kalman filter. The default value seems to be a bit too low.
        self.scf.cf.param.set_value("locSrv.extQuatStdDev", 0.06)

        # reset_estimator(cf)
        self.scf.cf.param.set_value("kalman.resetEstimation", "1")
        time.sleep(0.1)
        self.scf.cf.param.set_value("kalman.resetEstimation", "0")

        # time.sleep(1)
        # wait_for_position_estimator(cf, print_progress)
        log_config = LogConfig(name="Kalman Variance", period_in_ms=500)
        log_config.add_variable("kalman.varPX", "float")
        log_config.add_variable("kalman.varPY", "float")
        log_config.add_variable("kalman.varPZ", "float")

        var_y_history = [1000] * 10
        var_x_history = [1000] * 10
        var_z_history = [1000] * 10

        threshold = 0.001
        timeout_seconds = 10.0

        self.logger.debug(f"starting reset estimator loop")
        success = False
        try:
            # TODO: stuck somewhere here...
            with SyncLogger(self.scf, log_config) as logger:
                start_time = time.time()
                time_old = time.time()
                for log_entry in logger:
                    time_now = time.time()
                    duration = time_now - time_old
                    self.logger.debug(f"last iteration took {duration} seconds")
                    time_old = time_now

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

                    # if print_progress:
                    # print(f"{dx} {dy} {dz}")
                    time_now = time.time()
                    duration = time_now - start_time
                    self.logger.debug(f"{duration} {dx} {dy} {dz}")

                    if dx < threshold and dy < threshold and dz < threshold:
                        success = True
                        break
                    if timeout_seconds < duration:
                        self.logger.critical(f"failed to reset state estimator in under {timeout_seconds} seconds")
                        self.kill_service()
        except Exception:
            traceback.print_exc()
            success = False
        self.logger.debug(f"finished reset estimator loop")
        return success

    def update_state_estimate(self) -> None:
        """Reset drone's position estimates"""
        self.logger.info(f"estimator reset starts")
        if self.no_motion_capture:
            # There is no need to reset state estimate without data from motion capture system
            pass
        else:
            # We want to update drone's state estimate with data from motion capture system
            if self.reset_estimator():
                self.logger.info(f"estimator reset succeeded")
            else:
                self.logger.error(f"estimator reset failed")
        self.logger.info(f"estimator reset finished")

    def reset_drone(self) -> None:
        self.clear_command_queue()
        self.reseting = True
        self.reboot_drone()
        self.connect_to_drone()
        self.reseting = False
        self.update_state_estimate()

    def get_current_position(self) -> Optional[DroneState]:
        try:
            positions = self.mcc.root.get_positions()
            positions = rpyc.classic.obtain(positions)
            if positions is None:
                self.logger.warning(f"did not receive drone positions")
                return None
            else:
                # print(positions)
                if isinstance(positions, dict):
                    if self.name in positions.keys():
                        position = positions[self.name]
                        if isinstance(position, DroneState):
                            return position
                        else:
                            self.logger.warning(f"received {type(position)}, expected DronePosition")
                            return None
                    else:
                        self.logger.warning(f"did not receive position")
                        return None
                else:
                    self.logger.warning(f"received {type(position)}, expected dictionary")
                    return None
        except EOFError:
            self.logger.warning(f"lost connection to motion capture service")
            return None

    def update_thread_logic(self) -> None:
        self.logger.info(f"update loop starts")
        do_continue = True
        time_old = time.time()
        while do_continue:
            if self.stop_handle_update.is_set():
                self.logger.critical(f"update thread is asked to stop")
                do_continue = False
            time_now = time.time()
            duration = time_now - time_old
            if 5.0 < duration:
                self.logger.debug(f"update thread is alive")
                time_old = time_now
            if self.no_motion_capture:
                # There is no need to update drone's state estimate without motion capture system
                time.sleep(1.0)
            else:
                # We want to update drone's state estimate with data from motion capture system
                position = self.get_current_position()
                if position is None:
                    self.logger.warning(f"failed to update position estimate")
                    self.failures_to_get_position_estimate += 1
                    if self.position_estimate_failure_threshold <= self.failures_to_get_position_estimate:
                        self.logger.critical(f"failed to update position estimate too many times")
                        do_continue = False
                    time.sleep(1.0)
                else:
                    self.scf.cf.extpos.send_extpose(position.x, position.y, position.z, position.qx, position.qy, position.qz, position.qw)
                    self.failures_to_get_position_estimate = 0

    def update_thread(self) -> None:
        try:
            self.update_thread_logic()
        except Exception:
            traceback.print_exc()
        finally:
            self.handle_update_finished.set()
            self.logger.info(f"update loop finished")

    def control_thread_logic(self) -> None:  # noqa: C901
        do_continue = True
        do_reset = True
        # TODO: solve if we are actually landed currently
        state = DroneDynamicState.LANDED
        self.logger.info(f"control loop starts")

        setpoint_x = None
        setpoint_y = None
        setpoint_z = None

        position_old = None
        time_old = time.time()
        while do_continue:
            if self.stop_handle_control.is_set():
                self.logger.critical(f"control thread is asked to stop")
                do_continue = False
            time_now = time.time()
            duration = time_now - time_old
            # self.logger.debug(duration)
            if 5.0 < duration:
                self.logger.debug(f"control thread is alive")
                time_old = time_now

            if self.handle_update_finished.is_set():
                self.logger.info(f"control thread recognized that update thread has finished")
                do_continue = False
            # if self.handle_update.is_alive()

            state = self.scf.cf.state
            string = CF_STATE_TO_STRING[state]
            self.logger.info(f"drone state: {string}")

            if do_reset:
                do_reset = False
                if self.no_radio:
                    pass
                else:
                    self.reset_drone()

            # Get current position from motion capture service
            if self.no_motion_capture:
                # We do not care about drone's position
                position = None
            else:
                # We care about drone's position, try to get it no matter what
                position = self.get_current_position()
                if position is None:
                    # Check if we can use old position
                    if position_old is None:
                        # Bad luck, what now?
                        self.logger.warning(f"new and old position not available, no position data usable")
                        self.kill_service()
                        do_continue = False
                    else:
                        self.logger.warning(f"new position not available, using old position")
                        position = position_old
                else:
                    pass

            # Get next command from command buffer
            if self.queue.empty():
                # No commands given, what now?
                pass
            else:
                command = self.queue.get()
                self.logger.debug(f"got command")
                if isinstance(command, CommandQuit):
                    do_continue = False
                    if self.no_radio:
                        pass
                    else:
                        self.disconnect_from_drone()
                if isinstance(command, CommandReset):
                    self.logger.debug(f"got command reset")
                    if self.no_radio:
                        pass
                    else:
                        self.disconnect_from_drone()
                        self.reset_drone()
                if isinstance(command, CommandTakeoff):
                    state = DroneDynamicState.TAKING_OFF
                    setpoint_z = self.takeoff_z
                    self.logger.info(f"drone {self.name} taking off with setpoint z={setpoint_z}")
                if isinstance(command, CommandLand):
                    state = DroneDynamicState.LANDING
                    setpoint_z = self.landing_z
                if isinstance(command, CommandPositionSetpoint):
                    if position is None:
                        pass
                    else:
                        if state == DroneDynamicState.HOVERING:
                            match command.setpoint_type:
                                case PositionSetpointType.ABSOLUTE:
                                    # TODO: validate also z
                                    target_point = Point(command.x, command.y)
                                    if LABORATORY_PHYSICAL_RECTANGLE.contains(target_point):
                                        setpoint_x = command.x
                                        setpoint_y = command.y
                                        if command.z is None:
                                            pass
                                        else:
                                            setpoint_z = command.z
                                    else:
                                        print(f"drone {self.name} target position {target_point} outside limits")
                                        setpoint_x = position.x
                                        setpoint_y = position.y
                                        setpoint_z = position.z
                                case PositionSetpointType.RELATIVE:
                                    setpoint_x = position.x + command.x
                                    setpoint_y = position.y + command.y
                                    if command.z is None:
                                        pass
                                    else:
                                        setpoint_z = position.z + command.z
                                    # TODO: validate also z
                                    target_point = Point(setpoint_x, setpoint_y)
                                    if LABORATORY_PHYSICAL_RECTANGLE.contains(target_point):
                                        pass
                                    else:
                                        print(f"drone {self.name} target position {target_point} outside limits")
                                        setpoint_x = position.x
                                        setpoint_y = position.y
                                        setpoint_z = position.z
                        else:
                            print(f"drone {self.name} can not move until it hovers")
                if isinstance(command, CommandState):
                    if position is None:
                        pass
                    else:
                        print(f"drone {self.name} is in state {state} at ({position.x}, {position.y}, {position.z}, {position.angle}) with setpoint ({setpoint_x}, {setpoint_y}, {setpoint_z})")
                if isinstance(command, CommandTurn):
                    if state == DroneDynamicState.HOVERING:
                        # setpoint_yaw = command.angle
                        print(f"drone {self.name} does not yet support this command")
                    else:
                        print(f"drone {self.name} can not turn until it hovers")

            if setpoint_z is not None:
                self.logger.info(f"hovering {setpoint_x} {setpoint_y} {setpoint_z}")
                self.scf.cf.commander.send_hover_setpoint(vx=0.0, vy=0.0, yawrate=0.0, zdistance=setpoint_z)

            # send_setpoint = True
            # if position is None:
            #     if self.no_motion_capture:
            #         # This is expected
            #         # Since we have no position, we can not give default setpoint
            #         setpoint_x = None
            #         setpoint_y = None
            #         setpoint_z = None
            #     else:
            #         self.logger.warning(f"drone {self.name} position is not available")
            #         if position_old is None:
            #             # Bad luck, we have no position or previous position
            #             setpoint_x = None
            #             setpoint_y = None
            #             setpoint_z = None
            #         else:
            #             # Previous position is the best guess where we are right now
            #             position = position_old
            #             setpoint_x = position.x
            #             setpoint_y = position.y
            #             setpoint_z = None
            # else:
            #     # We have drone's assumed position
            #     send_setpoint = True
            #     # Keep same position by default
            #     setpoint_x = position.x
            #     setpoint_y = position.y
            #     setpoint_z = position.z

            # if send_setpoint:
            #     # Update setpoint based on current state
            #     match state:
            #         case DroneState.LANDED:
            #             # Do nothing, wait for next command
            #             pass
            #         case DroneState.TAKING_OFF:
            #             self.logger.info(f"asd {setpoint_x} {setpoint_y} {setpoint_z}")
            #             self.scf.cf.commander.send_hover_setpoint(vx=0.0, vy=0.0, yawrate=0.0, zdistance=setpoint_z)
            #             if np.abs(position.z - self.takeoff_z) < 0.05:
            #                 # Takeoff height reached, start hovering
            #                 state = DroneState.HOVERING
            #             else:
            #                 # Takeoff height is not yet reached
            #                 state = DroneState.TAKING_OFF
            #         case DroneState.HOVERING:
            #             # Stay at same position
            #             #self.scf.cf.commander.send_position_setpoint(x=setpoint_x, y=setpoint_y, z=setpoint_z, yaw=0.0)
            #             self.scf.cf.commander.send_hover_setpoint(vx=0.0, vy=0.0, yawrate=0.0, zdistance=setpoint_z)
            #             state = DroneState.HOVERING
            #         case DroneState.LANDING:
            #             if position.z < 0.05:
            #                 # We are close enough to ground to stop motors
            #                 self.scf.cf.commander.send_stop_setpoint()
            #                 # Firmware requires this to be zeroed before next flight
            #                 # TODO: where is this documented?
            #                 self.scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
            #                 state = DroneState.LANDED
            #             else:
            #                 # We are not close enough ground to stop motors
            #                 self.scf.cf.commander.send_hover_setpoint(vx=0.0, vy=0.0, yawrate=0.0, zdistance=setpoint_z)
            #                 state = DroneState.LANDING
            #         case DroneState.MOVING:
            #             pass
            # else:
            #     # We do not have setpoint to send to the drone, what now?
            #     match state:
            #         case DroneState.LANDED:
            #             # Lucky, do nothing and wait for next command
            #             pass
            #         case DroneState.TAKING_OFF:
            #             # Lucky, just continue taking off, use default takeoff z
            #             setpoint_z = self.takeoff_z
            #             self.logger.info(f"asd {setpoint_x} {setpoint_y} {setpoint_z}")
            #             self.scf.cf.commander.send_hover_setpoint(vx=0.0, vy=0.0, yawrate=0.0, zdistance=setpoint_z)
            #             if np.abs(position.z - self.takeoff_z) < 0.05:
            #                 # Takeoff height reached, start hovering
            #                 state = DroneState.HOVERING
            #             else:
            #                 # Takeoff height is not yet reached
            #                 state = DroneState.TAKING_OFF
            #         case DroneState.HOVERING:
            #             # Lucky, just continue hovering, use default takeoff z
            #             setpoint_z = self.takeoff_z
            #             # Stay at same position
            #             #self.scf.cf.commander.send_position_setpoint(x=setpoint_x, y=setpoint_y, z=setpoint_z, yaw=0.0)
            #             self.scf.cf.commander.send_hover_setpoint(vx=0.0, vy=0.0, yawrate=0.0, zdistance=setpoint_z)
            #             state = DroneState.HOVERING
            #         case DroneState.LANDING:
            #             # Bad luck, just continue hovering
            #             setpoint_z = self.takeoff_z
            #             # Stay at same position
            #             #self.scf.cf.commander.send_position_setpoint(x=setpoint_x, y=setpoint_y, z=setpoint_z, yaw=0.0)
            #             self.scf.cf.commander.send_hover_setpoint(vx=0.0, vy=0.0, yawrate=0.0, zdistance=setpoint_z)
            #             state = DroneState.HOVERING
            #         case DroneState.MOVING:
            #             # Bad luck, just continue hovering
            #             setpoint_z = self.takeoff_z
            #             # Stay at same position
            #             #self.scf.cf.commander.send_position_setpoint(x=setpoint_x, y=setpoint_y, z=setpoint_z, yaw=0.0)
            #             self.scf.cf.commander.send_hover_setpoint(vx=0.0, vy=0.0, yawrate=0.0, zdistance=setpoint_z)
            #             state = DroneState.HOVERING

            if self.no_motion_capture:
                # Assume that loop iterations are fast
                time.sleep(1.0)

            if position is not None:
                position_old = position

    def control_thread(self) -> None:
        try:
            self.control_thread_logic()
        except Exception:
            traceback.print_exc()
        finally:
            self.handle_control_finished.set()
            self.logger.info(f"control loop finished")

    def monitor_thread_logic(self) -> None:
        self.logger.info(f"monitor loop starts")
        do_continue = True
        time_old = time.time()
        while do_continue:
            if self.stop_handle_monitor.is_set():
                self.logger.critical(f"monitor thread is asked to stop")
                do_continue = False
            time_now = time.time()
            duration = time_now - time_old
            if 5.0 < duration:
                self.logger.debug(f"monitor thread is alive")
                time_old = time_now
            update_finished = self.handle_update_finished.is_set()
            update_died = not self.handle_update.is_alive()
            control_finished = self.handle_control_finished.is_set()
            control_died = not self.handle_control.is_alive()
            if update_finished and control_finished:
                self.logger.info(f"update and control loops finished")
                do_continue = False
            elif update_finished and control_died:
                self.logger.warning(f"update loop finished and control loop died")
                do_continue = False
            elif update_died and control_finished:
                self.logger.warning(f"update loop died and control loop finished")
                do_continue = False
            elif update_died and control_died:
                self.logger.critical(f"update and control loops died")
                do_continue = False
            time.sleep(1.0)
        if self.server is not None:
            self.logger.info(f"closes server")
            self.server.close()
        else:
            self.logger.critical(f"could not close server")

    def monitor_thread(self) -> None:
        try:
            self.monitor_thread_logic()
        except Exception:
            traceback.print_exc()
        finally:
            self.logger.info(f"monitor loop finished")


# FIXME: stupid hack


class Cf1Service(DroneService):
    ALIASES = ["cf1"]


class Cf2Service(DroneService):
    ALIASES = ["cf2"]


class Cf3Service(DroneService):
    ALIASES = ["cf3"]


class Cf4Service(DroneService):
    ALIASES = ["cf4"]


class Cf5Service(DroneService):
    ALIASES = ["cf5"]


class Cf6Service(DroneService):
    ALIASES = ["cf6"]


NAME_TO_SERVICE: Dict[str, DroneService] = {
    "cf1": Cf1Service,
    "cf2": Cf2Service,
    "cf3": Cf3Service,
    "cf4": Cf4Service,
    "cf5": Cf5Service,
    "cf6": Cf6Service,
}
