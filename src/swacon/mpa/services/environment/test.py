# import rpyc
from rpyc.utils.factory import discover, DiscoveryError, connect
from rpyc.utils.registry import TCPRegistryClient


if __name__ == "__main__":
    registrar = TCPRegistryClient("localhost")
    service = "environment"
    try:
        providers = discover(service, "localhost", registrar)
        print(f"found {len(providers)} providers for service {service}:")
        for i, provider in enumerate(providers):
            host, port = provider
            print(f"  {i + 1}: {host} {port}")
        if 1 < len(providers):
            print(f"found too many providers for service {service}")
            quit()
        provider = providers[0]
        host, port = provider
        try:
            connection = connect(host, port)
            # environment = connection.get_environment()
            print(dir(connection))
            # print(environment)

            # positions = connection.root.get_positions()
            # positions = rpyc.classic.obtain(positions)
            # print(f"received positions for {len(positions)} drones")
            # for i, (name, position) in enumerate(positions.items()):
            #    assert isinstance(position, DronePosition)
            #    x = round(position.x, 4)
            #    y = round(position.y, 4)
            #    z = round(position.z, 4)
            #    angle = round(position.angle, 4)
            #    print(f"  {i + 1}: {name} {x} {y} {z} {angle}")
        except ConnectionRefusedError:
            print(f"{service} refused connection")
    except DiscoveryError:
        print(f"found no providers for service {service}")
