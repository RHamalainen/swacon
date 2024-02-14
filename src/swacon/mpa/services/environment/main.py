"""Environment service

Services provided:
 - get current environment
 - modify current environment
"""

import sys
from pathlib import Path

import xarray as xr
from rpyc import Service, ThreadedServer
from rpyc.utils.registry import TCPRegistryClient

# Add top level modules to PYTHONPATH
sys.path.append(str(Path(".").absolute()))
import services.common.service_configuration


class EnvironmentService(Service):
    def __init__(self) -> None:
        super().__init__()
        self.environment = xr.DataArray()

    def exposed_get_environment(self) -> xr.DataArray:
        return self.environment

    def exposed_set_environment(self, new_environment: xr.DataArray) -> None:
        assert isinstance(new_environment, xr.DataArray)
        self.environment = new_environment


class EnvironmentServer(ThreadedServer):
    def __init__(self, service: EnvironmentService, hostname: str, registrar: TCPRegistryClient) -> None:
        assert isinstance(service, EnvironmentService)
        assert isinstance(hostname, str)
        assert isinstance(registrar, TCPRegistryClient)
        super().__init__(service=service, hostname=hostname, registrar=registrar, auto_register=True)


if __name__ == "__main__":
    registrar = TCPRegistryClient("localhost")
    service = EnvironmentService()
    server = EnvironmentServer(service, "localhost", registrar)
    server.start()
