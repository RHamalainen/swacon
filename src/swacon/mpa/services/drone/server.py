import socket

from rpyc.utils.server import Server

from service import DroneService


class DroneServer(Server):
    def __init__(
        self,
        service: DroneService,
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
        assert isinstance(service, DroneService)
        assert isinstance(hostname, str)
        super().__init__(service, hostname, ipv6, port, backlog, reuse_addr, authenticator, registrar, auto_register, protocol_config, logger, listener_timeout, socket_path)

    def close(self) -> None:
        self.service.stop_handle_update.set()
        self.service.stop_handle_control.set()
        self.service.stop_handle_monitor.set()
        return super().close()
