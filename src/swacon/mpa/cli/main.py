# ==========================================
# REMEMBER: 30 min wait after starting Vicon
# ==========================================

# TODO: use leds to indicate current state
# TODO: study how cachedcffactory works = it might solve slow connection issue
# TODO: use rpyc decorator instead of _exposed

from typing import Optional, Dict, Any
from logging import Logger, getLogger, DEBUG as LOGGING_LEVEL_DEBUG
from argparse import ArgumentParser

import rpyc
from rpyc import Service, ThreadedServer
from rpyc.core.protocol import Connection
from rpyc.utils.factory import discover, DiscoveryError, connect
from rpyc.utils.registry import TCPRegistryClient

from swacon.mpa.constants import DRONE_NAME_TO_URI
from swacon.data_structures.drone.state import DroneState
from swacon.data_structures.drone.command import (
    Command,
    CommandLand,
    CommandPositionSetpoint,
    CommandQuit,
    CommandReset,
    CommandState,
    CommandTakeoff,
    CommandTurn,
    PositionSetpointType,
    TurnDirection,
)


class Application:
    def __init__(self, logger: Logger):
        assert isinstance(logger, Logger)
        self.logger = logger
        self.registrar = TCPRegistryClient("localhost")
        self.motion_capture_connection: Optional[Connection] = None
        self.drone_connections: Dict[str, Connection] = dict()

    def try_parse_drone_name(self, maybe_drone_number: str) -> Optional[str]:
        try:
            number = int(maybe_drone_number)
            name = f"cf{number}"
            if name in DRONE_NAME_TO_URI.keys():
                return name
            else:
                print(f"drone {name} does not exist")
                return None
        except ValueError as error:
            print(f"{error}")
            return None

    def try_get_drone_connection(self, maybe_drone_number: str) -> Optional[Any]:
        match self.try_parse_drone_name(maybe_drone_number):
            case None:
                return None
            case name:
                if name in self.drone_connections.keys():
                    connection = self.drone_connections[name]
                    return connection
                else:
                    self.logger.info(f"not connected to drone {name}")
                    return None

    def connect_to_motion_capture_service(self):
        try:
            service = "motioncapture"
            providers = discover(service, "localhost", self.registrar)
            if len(providers) == 1:
                provider = providers[0]
                host, port = provider
                connection = connect(host, port, keepalive=True)
                assert isinstance(connection, Connection)
                self.motion_capture_connection = connection
            elif 1 < len(providers):
                self.logger.error(f"found too many ({len(providers)}) providers for service {service}")
        except DiscoveryError as error:
            self.logger.error(f"{error}")

    def establish_connection_to_drone(self, name: str) -> Optional[Connection]:
        try:
            service = name
            providers = discover(service, "localhost", self.registrar)
            if len(providers) == 1:
                provider = providers[0]
                host, port = provider
                connection = connect(host, port, keepalive=True)
                assert isinstance(connection, Connection)
                return connection
            elif 1 < len(providers):
                self.logger.error(f"found too many ({len(providers)}) providers for service {service}")
                return None
        except DiscoveryError as error:
            self.logger.error(f"{error}")
            return None
        return None

    def connect_to_drone(self, name: str):
        if name in self.drone_connections.keys():
            # TODO: maybe just update the connection?
            self.logger.info(f"already connected to drone {name}")
        else:
            connection = self.establish_connection_to_drone(name)
            if connection is None:
                self.logger.error(f"failed to establish connection to drone {name}")
            else:
                assert isinstance(connection, Connection)
                self.drone_connections[name] = connection

    def disconnect_from_drone_by_connection(self, connection: Connection):
        connection.root.send_command(CommandQuit())
        # TODO: maybe also stop_service ?
        try:
            connection.root.stop_service()
        except EOFError:
            # This is expected
            pass

    def disconnect_from_drone_by_name(self, name: str):
        connection = self.drone_connections[name]
        self.disconnect_from_drone_by_connection(connection)

    def disconnect_from_all_drones(self):
        if 0 < len(self.drone_connections):
            self.logger.info(f"disconnecting from {len(self.drone_connections)} drones")
            while 0 < len(self.drone_connections):
                _name, connection = self.drone_connections.popitem()
                self.disconnect_from_drone_by_connection(connection)

    def print_help(self):
        print(f"drone controller v0.0.0")
        print(f"  [help] print this message")
        print(f"  [quit] stop operation")
        print(f"  [connections] print established connections")
        print(f"  [connect <list of numbers>] establish connection to drones")
        print(f"  [disconnect <number>] break connection to drone")
        print(f"  [reset <number>]")
        print(f"  [takeoff <number>]")
        print(f"  [land <number>]")
        print(f"  [state <number>] print drone state")
        print(f"  [move <absolute/relative> <number> <x> <y> [<z>]] move drone to absolute or relative position")
        print(f"  [turn <left/right> <number> <angle>] turn drone given degrees left or right")
        print(f"  [parameter <number> <name>] print parameter value")
        print(f"  [positions] print all drone positions detected by motion capture system")

    def print_drone_connections(self):
        for i, (name, connection) in enumerate(self.drone_connections.items()):
            self.logger.info(f"  {i + 1}: {name}, {connection}")

    # TODO: check if connections are valid

    def print_drone_positions(self):
        if self.motion_capture_connection is None:
            self.logger.info(f"could not get drone positions since motion capture connection is not established")
        else:
            positions = self.motion_capture_connection.root.get_positions()
            positions = rpyc.classic.obtain(positions)
            if len(positions) == 0:
                self.logger.info(f"no positions received")
            else:
                for i, (name, position) in enumerate(positions.items()):
                    if isinstance(position, DroneState):
                        x = round(position.x, 4)
                        y = round(position.y, 4)
                        z = round(position.z, 4)
                        angle = round(position.angle, 4)
                        self.logger.info(f"  {i + 1}: {name} {x} {y} {z} {angle}")
                    else:
                        self.logger.warning(f"  {i + 1}: {name}, received {type(position)}, expected DronePosition")

    def print_drone_parameter(self, connection: Connection, parameter: str):
        assert isinstance(connection, Connection)
        assert isinstance(parameter, str)
        try:
            value = connection.root.scf.cf.param.get_value(parameter)
            self.logger.info(f"drone {connection.root.name} parameter {parameter} value is {value}")
        except Exception as error:
            self.logger.info(f"could not get drone {connection.root.name} parameter {parameter} value:\n - {error}")

    def run(self):
        # self.connect_to_motion_capture_service()

        # TODO: placeholder to fix linter warnings
        connected_drones = dict()

        do_continue = True
        do_print_help = True
        while do_continue:
            if do_print_help:
                do_print_help = False
                self.print_help()

            try:
                parts = input("$ ").split(" ")
            except EOFError:
                parts = list()
            if len(parts) == 0:
                do_print_help = True
                continue
            match parts[0]:
                case "help":
                    do_print_help = True
                case "quit":
                    do_continue = False
                case "connections":
                    self.print_drone_connections()
                case "connect":
                    match self.try_parse_drone_name(parts[1]):
                        case None:
                            pass
                        case name:
                            self.connect_to_drone(name)
                case "state":
                    match self.try_get_drone_connection(parts[1]):
                        case None:
                            pass
                        case connection:
                            connection.root.send_command(CommandState())
                case "disconnect":
                    match self.try_get_drone_connection(parts[1]):
                        case None:
                            pass
                        case connection:
                            self.disconnect_from_drone_by_connection(connection)
                case "reset":
                    match self.try_get_drone_connection(parts[1]):
                        case None:
                            pass
                        case connection:
                            connection.root.send_command(CommandReset())
                case "takeoff":
                    match self.try_get_drone_connection(parts[1]):
                        case None:
                            pass
                        case connection:
                            connection.root.send_command(CommandTakeoff())
                case "land":
                    match self.try_get_drone_connection(parts[1]):
                        case None:
                            pass
                        case connection:
                            connection.root.send_command(CommandLand())
                case "parameter":
                    match self.try_get_drone_connection(parts[1]):
                        case None:
                            pass
                        case connection:
                            parameter = parts[2]
                            self.print_drone_parameter(connection, parameter)
                case "turn":
                    match parts[1]:
                        case "left" | "right":
                            match self.try_parse_drone_name(parts[2]):
                                case None:
                                    pass
                                case name:
                                    if name in connected_drones.keys():
                                        drone = connected_drones[name]
                                        try:
                                            angle = float(parts[3])
                                            if parts[1] == "left":
                                                direction = TurnDirection.LEFT
                                            elif parts[1] == "right":
                                                direction = TurnDirection.RIGHT
                                            else:
                                                print(f"invalid turn direction: {parts[1]}")
                                                direction = None
                                            if direction is not None:
                                                drone.queue.put(CommandTurn(direction, angle))
                                        except ValueError as error:
                                            print(f"{error}")
                                    else:
                                        print(f"not connected to drone {name}")
                        case other:
                            print(f"unknown command: {other}")
                case "move":
                    match parts[1]:
                        case "absolute" | "relative":
                            match self.try_parse_drone_name(parts[2]):
                                case None:
                                    pass
                                case name:
                                    if name in connected_drones.keys():
                                        drone = connected_drones[name]
                                        try:
                                            x = float(parts[3])
                                            y = float(parts[4])
                                            continue_command = True
                                            try:
                                                z = float(parts[5])
                                                continue_command = True
                                            except ValueError as error:
                                                # User tried to give z-coordinate, but it is invalid
                                                print(f"{error}")
                                                z = None
                                                continue_command = False
                                            except IndexError:
                                                # User does not want to give z-coordinate
                                                z = None
                                                continue_command = True
                                            if continue_command:
                                                if parts[1] == "absolute":
                                                    setpoint_type = PositionSetpointType.ABSOLUTE
                                                elif parts[1] == "relative":
                                                    setpoint_type = PositionSetpointType.RELATIVE
                                                else:
                                                    print(f"invalid turn direction: {parts[1]}")
                                                    setpoint_type = None
                                                if setpoint_type is not None:
                                                    drone.queue.put(CommandPositionSetpoint(x, y, z, setpoint_type))
                                        except ValueError as error:
                                            print(f"{error}")
                                    else:
                                        print(f"not connected to drone {name}")
                        case other:
                            print(f"unknown command: {other}")
                case "positions":
                    self.print_drone_positions()
                case other:
                    print(f"unknown command: {other}")
        self.disconnect_from_all_drones()
        self.logger.info(f"application finished")


class CliService(Service):
    def __init__(self) -> None:
        super().__init__()
        self.registrar = TCPRegistryClient("localhost")
        self.server: Optional[ThreadedServer] = None
        self.drone_connections: Dict[str, Connection] = dict()

    def establish_connection_to_drone(self, name: str) -> Optional[Connection]:
        # Try to find existing service
        try:
            service = name
            providers = discover(service, "localhost", self.registrar)
            if len(providers) == 1:
                provider = providers[0]
                host, port = provider
                connection = connect(host, port, keepalive=True)
                assert isinstance(connection, Connection)
                return connection
            elif 1 < len(providers):
                # self.logger.error(f"found too many ({len(providers)}) providers for service {service}")
                print(f"found too many ({len(providers)}) providers for service {service}")
                return None
        except DiscoveryError as error:
            print(f"{error}")
            # Try to create new service
            try:
                import subprocess

                # TODO: check success
                subprocess.Popen(["python3.10", "./services/drone/main.py", name, "--no-motion-capture", "--no-radio"])
                providers = discover(service, "localhost", self.registrar)
                if len(providers) == 1:
                    provider = providers[0]
                    host, port = provider
                    connection = connect(host, port, keepalive=True)
                    assert isinstance(connection, Connection)
                    return connection
                elif 1 < len(providers):
                    print(f"found too many ({len(providers)}) providers for service {service}")
                    return None
            except Exception as error:
                print(f"{error}")
                return None
            return None
        return None

    def exposed_connect(self, name: str):
        # assert isinstance(name, str)
        # assert isinstance
        if name in self.drone_connections.keys():
            # TODO: maybe just update the connection?
            # logger.info(f"already connected to drone {name}")
            print(f"already connected to drone {name}")
        else:
            connection = self.establish_connection_to_drone(name)
            if connection is None:
                # logger.error(f"failed to establish connection to drone {name}")
                print(f"failed to establish connection to drone {name}")
            else:
                assert isinstance(connection, Connection)
                self.drone_connections[name] = connection

    def exposed_send_command(self, name: str, command: Command):
        assert isinstance(name, str)
        assert isinstance(command, Command)
        if name in self.drone_connections.keys():
            connection = self.drone_connections[name]
            connection.root.send_command(command)
        else:
            print(f"not connected to drone {name}")

    # def exposed_reset_drone(self, name: str):
    #    if name in self.drone_connections.keys():
    #        connection = self.drone_connections[name]
    #        connection.root.send_command(CommandReset())
    #    else:
    #        print(f"not connected to drone {name}")

    def exposed_disconnect(self, name: str):
        if name in self.drone_connections.keys():
            # Do not kill the service here, keep it open
            connection = self.drone_connections.pop(name)  # noqa: F841
            # connection.root.stop_service()
        else:
            print(f"already disconnected from drone {name}")

    def exposed_help(self):
        print(f"no")


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--service", action="store_true")
    arguments = vars(parser.parse_args())

    main_logger = getLogger("main_logger")
    main_logger.setLevel(LOGGING_LEVEL_DEBUG)
    if arguments["service"]:
        registrar = TCPRegistryClient("localhost")
        service = CliService()
        server = ThreadedServer(service, "localhost", registrar=registrar, auto_register=True)
        service.server = server
        server.start()
    else:
        application = Application(main_logger)
        application.run()
    main_logger.info(f"main thread finished")
