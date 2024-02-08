import pickle
from pathlib import Path
from typing import List, Dict, Optional

import xarray as xr
import matplotlib.pyplot as plt

import swacon.spa.constants as constants
import swacon.algorithms.transforms.coordinate_transforms as ct
from swacon.data_structures.drone.state import DroneState
from swacon.spa.find_newest_file import find_newest_file


def find_most_recent_environment() -> Optional[xr.DataArray]:
    most_recent_path = find_newest_file("*.nc", constants.PATH_OUTPUT_ENVIRONMENT_FOLDER)
    if most_recent_path is None:
        return None
    with most_recent_path.open("rb") as stream:
        data = xr.load_dataarray(stream)
    return data


# TODO: use better return type
def find_most_recent_flight_data() -> Optional[dict]:
    most_recent_path = find_newest_file("*.pickle", constants.PATH_OUTPUT_FLIGHT_DATA_FOLDER)
    if most_recent_path is None:
        return None
    with most_recent_path.open("r+b") as stream:
        data = pickle.load(stream)
    return data


# TODO: create new plot for new environments

if __name__ == "__main__":
    environment = find_most_recent_environment()
    if environment is None:
        print(f"failed to find most recent environment dataset")
        quit()
    flight_data = find_most_recent_flight_data()
    if flight_data is None:
        print(f"failed to find most recent flight data record")
        quit()

    print(f"found data for {len(flight_data.keys())} drones:")
    for i, (name, _record) in enumerate(flight_data.items()):
        print(f" - {i + 1}/{len(flight_data.keys())}: {name}")
    for name, record in flight_data.items():
        timestamps = list()
        states = list()
        xs: List[Optional[float]] = list()
        ys: List[Optional[float]] = list()
        zs: List[Optional[float]] = list()
        for timestamp, state in record:
            # print(f"{timestamp} {state}")
            timestamps.append(timestamp)
            states.append(state)
            if state is None:
                xs.append(None)
                ys.append(None)
                zs.append(None)
            else:
                xs.append(state.x)
                ys.append(state.y)
                zs.append(state.z)
        plt.figure()
        plt.plot(timestamps, xs, label="x")
        plt.plot(timestamps, ys, label="y")
        plt.plot(timestamps, zs, label="z")
        plt.xticks([])
        # TODO: maybe also plot velocities and angles?
        # TODO: maybe also project timepositions into given environment?
        plt.title(f"{name}")
        plt.legend()

    # plt.figure()
    plt.matshow(environment)
    for name, record in flight_data.items():
        timestamps = list()
        states = list()
        xs: List[Optional[float]] = list()
        ys: List[Optional[float]] = list()
        # zs: List[Optional[float]] = list()
        for timestamp, state in record:
            # print(f"{timestamp} {state}")
            timestamps.append(timestamp)
            states.append(state)
            if state is None:
                xs.append(None)
                ys.append(None)
                # zs.append(None)
            else:
                # Physical coordinates
                px = state.x
                py = state.y
                # pz = state.z

                # Raster coordinates
                rx, ry = ct.physical_to_raster((px, py))
                xs.append(rx)
                ys.append(ry)
        plt.scatter(xs, ys, label=f"{name}")
    plt.legend()
    plt.show()
