import sys
import socket
import logging
from copy import deepcopy
from pathlib import Path
from argparse import ArgumentParser

from rpyc import Service, ThreadedServer
from rpyc.utils.registry import TCPRegistryClient

# Add top level modules to PYTHONPATH
sys.path.append(str(Path(".").absolute()))
from drone_position import DronePosition
from constants import DRONE_NAME_TO_URI
import services.common.service_configuration

from worker import DRONE_STATES, DRONE_STATES_LOCK, MotionCaptureWorker


class MotionCaptureService(Service):
    def __init__(self, fake: bool) -> None:
        assert isinstance(fake, bool)
        super().__init__()
        self.fake = fake
        if self.fake:
            pass
        else:
            logging.info(f"attempting to start motion capture worker")
            self.worker = MotionCaptureWorker()
            self.worker.start()

    def exposed_get_positions(self):
        if self.fake:
            positions = dict()
            for name in DRONE_NAME_TO_URI.keys():
                positions[name] = DronePosition(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return positions
        else:
            with DRONE_STATES_LOCK:
                positions = deepcopy(DRONE_STATES)
            return positions


class MotionCaptureServer(ThreadedServer):
    def __init__(
        self,
        service: MotionCaptureService,
        hostname=None,
        ipv6=False,
        port=0,
        backlog=socket.SOMAXCONN,
        reuse_addr=True,
        authenticator=None,
        registrar=None,
        auto_register=None,
        protocol_config=None,
        logger=None,
        listener_timeout=0.5,
        socket_path=None,
    ):
        assert isinstance(service, MotionCaptureService)
        super().__init__(service, hostname, ipv6, port, backlog, reuse_addr, authenticator, registrar, auto_register, protocol_config, logger, listener_timeout, socket_path)

    def close(self):
        if self.service.fake:
            pass
        else:
            self.service.worker.close()
        return super().close()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    parser = ArgumentParser()
    parser.add_argument("--fake", action="store_true")
    arguments = vars(parser.parse_args())

    fake = arguments["fake"]
    if fake:
        logging.info(f"motion capture service is configured to produce fake positions")

    registrar = TCPRegistryClient("localhost")
    service = MotionCaptureService(fake)
    server = MotionCaptureServer(service, "localhost", registrar=registrar, auto_register=True)
    server.start()
