from argparse import ArgumentParser

from rpyc.utils.registry import TCPRegistryClient
from cflib.crtp import init_drivers as initialize_drone_drivers

from swacon.mpa.constants import DRONE_NAME_TO_URI
from swacon.mpa.services.drone.server import DroneServer
from swacon.mpa.services.drone.service import NAME_TO_SERVICE

# TODO
# if mww.IN_LABORATORY:
#    from mocap_wrapper import reset_estimator
# else:
#    def reset_estimator():
#        print(f"now executing fake reset_estimator")

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("name", type=str)
    parser.add_argument("--no-motion-capture", action="store_true")
    parser.add_argument("--no-radio", action="store_true")
    arguments = vars(parser.parse_args())
    name = arguments["name"]
    if name not in DRONE_NAME_TO_URI.keys():
        print(f"drone with name {name} does not exist")
        print(f"these drones exist:")
        for name, uri in DRONE_NAME_TO_URI.items():
            print(f"  {name} {uri}")
        exit(-1)
    uri = DRONE_NAME_TO_URI[name]
    no_motion_capture = arguments["no_motion_capture"]
    no_radio = arguments["no_radio"]
    initialize_drone_drivers()

    registrar = TCPRegistryClient("localhost")
    # FIXME: stupid hack
    service = NAME_TO_SERVICE[name](name, uri, no_motion_capture, no_radio)
    server = DroneServer(service, "localhost", registrar=registrar, auto_register=True)
    service.server = server
    # server.service = service
    server.start()

    # TODO: if server is terminated, also the worker threads must be terminated

    # service = DroneService(name, uri, no_motion_capture, no_radio)
    # server = ThreadedServer(service, "localhost", registrar=registrar, auto_register=True)
    # service.server = server
    # server.start()
