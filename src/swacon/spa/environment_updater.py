import time
import hashlib
import datetime
from copy import deepcopy
from pathlib import Path
from threading import Thread

import xarray as xr
from matplotlib.figure import Figure

import swacon.spa.constants as constants
import swacon.spa.environment as environment
from swacon.spa.find_newest_file import find_newest_file


class EnvironmentUpdater(Thread):
    def __init__(self) -> None:
        Thread.__init__(self)
        self.do_continue = True

    def close(self) -> None:
        self.do_continue = False

    def run(self) -> None:
        path_output_folder = constants.PATH_OUTPUT_ENVIRONMENT_FOLDER
        old_checksum = str()
        while self.do_continue:
            most_recent_file = find_newest_file("*.nc", constants.PATH_OUTPUT_ENVIRONMENT_FOLDER, print_output=False)
            if most_recent_file is None:
                print(f"failed to find most recent environment dataset")
            else:
                with most_recent_file.open("rb") as file:
                    data = file.read()
                    new_checksum = hashlib.md5(data).hexdigest()
                    # print(old_checksum, new_checksum)
                    if new_checksum != old_checksum:
                        print(f"detected new environment")
                        new_environment = xr.open_dataarray(file)
                        with environment.ENVIRONMENT_LOCK:
                            environment.ENVIRONMENT = deepcopy(new_environment)
                        # Plot new environment
                        timestamp = datetime.datetime.now().isoformat(timespec="seconds")
                        figure = Figure()
                        axes = figure.add_subplot()
                        axes.imshow(new_environment)
                        path = path_output_folder / Path(f"{timestamp}.png")
                        figure.savefig(path)
                        print(f"updated environment")
                    old_checksum = new_checksum
            time.sleep(1.0)


if __name__ == "__main__":
    print(f"starting environment updater")
    thread = EnvironmentUpdater()
    thread.run()
    print(f"finished environment updater")
