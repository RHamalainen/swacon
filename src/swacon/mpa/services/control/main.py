import sys
from pathlib import Path
from typing import Dict
import logging

import rpyc
from rpyc import Service, ThreadedServer
from rpyc.utils.registry import TCPRegistryClient

# Add top level modules to PYTHONPATH
sys.path.append(str(Path(".").absolute()))
from drone_position import DronePosition
from drone_control import DroneControl
import services.common.service_configuration


class ControlService(Service):
    def solve_controls(self, positions: Dict[str, DronePosition]) -> Dict[str, DroneControl]:
        # TODO
        return dict()

    def exposed_get_controls(self, positions):
        if isinstance(positions, dict):
            return self.solve_controls(positions)
        else:
            return None


if __name__ == "__main__":
    registrar = TCPRegistryClient("localhost")
    service = ControlService()
    server = ThreadedServer(service, "localhost", registrar=registrar, auto_register=True)
    server.start()
