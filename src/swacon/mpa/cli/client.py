from logging import getLogger, DEBUG as LOGGING_LEVEL_DEBUG

from rpyc import Connection
from rpyc.utils.factory import discover, DiscoveryError, connect
from rpyc.utils.registry import TCPRegistryClient

from swacon.data_structures.drone.command import CommandReset


if __name__ == "__main__":
    logger = getLogger("main_logger")
    logger.setLevel(LOGGING_LEVEL_DEBUG)
    registrar = TCPRegistryClient("localhost")
    try:
        providers = discover("CLI", "localhost", registrar)
        if 1 < len(providers):
            print(f"too many providers: {providers}")
        else:
            provider = providers[0]
            host, port = provider
            connection = connect(host, port)
            assert isinstance(connection, Connection)

            print(connection.root.connect("cf2"))
            connection.root.send_command("cf2", CommandReset())
            # connection.root.send_command("cf4", CommandTakeoff())
            # print(connection.root.reset_drone("cf1"))
            # print(connection.root.help())
            # connection.ping()
    except DiscoveryError as error:
        print(error)
