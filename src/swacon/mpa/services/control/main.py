from typing import Dict

from rpyc import Service, ThreadedServer
from rpyc.utils.registry import TCPRegistryClient

import swacon.mpa.services.common.service_configuration  # noqa: F401
from swacon.data_structures.drone.state import DroneState
from swacon.data_structures.drone.control import DroneControl


class ControlService(Service):
    def solve_controls(self, positions: Dict[str, DroneState]) -> Dict[str, DroneControl]:
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
