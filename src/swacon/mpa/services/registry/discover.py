from pathlib import Path

from rpyc import list_services
from rpyc.utils.factory import discover, DiscoveryError
from rpyc.utils.registry import TCPRegistryClient

IGNORED_FOLDERS = [
    "__pycache__",
    "common",
]


if __name__ == "__main__":
    registrar = TCPRegistryClient("localhost")
    services = list_services(registrar=registrar)
    if 0 < len(services):
        print(f"found {len(services)} services:")
        for i, service in enumerate(services):
            print(f"  {i + 1}: {service}")

        # Assume that service names are equal to folder names
        # services_folder = Path(__file__).parent
        # services = list()
        # for item in services_folder.iterdir():
        #    if item.is_dir():
        #        folder = item.name
        #        if folder not in IGNORED_FOLDERS:
        #            service = folder.replace("_", "")
        #            services.append(service)

        services = [
            "MOTIONCAPTURE",
            "CLI",
            "CF1",
            "CF2",
            "CF3",
            "CF4",
            "CF5",
            "CF6",
        ]

        for service in services:
            try:
                providers = discover(service, "localhost", registrar)
                print(f"found {len(providers)} providers for service {service}")
                for i, provider in enumerate(providers):
                    host, port = provider
                    print(f"  {i + 1}: {host} {port}")
            except DiscoveryError as error:
                print(f"found no providers for service {service}")
    else:
        print(f"found no services")
