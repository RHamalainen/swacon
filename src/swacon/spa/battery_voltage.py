from typing import Dict, Optional, List
from functools import partial
from threading import Event, Lock

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import swacon.spa.constants as constants
import swacon.spa.drone_discovery as drone_discovery


DRONE_BATTERY_VOLTAGE_RECORDING_TIMEOUT_SECONDS = 5.0
DRONE_BATTERY_VOLTAGE_RECORDED = Event()
DRONE_NAME_TO_BATTERY_VOLTAGE_LOCK = Lock()
DRONE_NAME_TO_BATTERY_VOLTAGE: Dict[str, float] = dict()


def battery_voltage_callback(name: str, timestamp: int, data: Dict[str, float], logconfig: LogConfig) -> None:
    assert isinstance(name, str)
    assert isinstance(timestamp, int)
    assert isinstance(data, dict)
    assert isinstance(logconfig, LogConfig)
    battery_voltage = data["pm.vbat"]
    with DRONE_NAME_TO_BATTERY_VOLTAGE_LOCK:
        DRONE_NAME_TO_BATTERY_VOLTAGE[name] = battery_voltage
    DRONE_BATTERY_VOLTAGE_RECORDED.set()


def get_battery_voltage(uri: str) -> Optional[float]:
    name = constants.DRONE_URI_TO_NAME[uri]
    with SyncCrazyflie(uri, Crazyflie(rw_cache="./cache")) as scf:
        battery_voltage_cb = partial(battery_voltage_callback, name)
        logconfig = LogConfig(f"{name}-config", 100)
        logconfig.add_variable("pm.vbat", "float")
        logconfig.data_received_cb.add_callback(battery_voltage_cb)
        scf.cf.log.add_config(logconfig)
        logconfig.start()
        if DRONE_BATTERY_VOLTAGE_RECORDED.wait(DRONE_BATTERY_VOLTAGE_RECORDING_TIMEOUT_SECONDS):
            DRONE_BATTERY_VOLTAGE_RECORDED.clear()
            with DRONE_NAME_TO_BATTERY_VOLTAGE_LOCK:
                return DRONE_NAME_TO_BATTERY_VOLTAGE[name]
        else:
            return None


def discover_battery_voltages() -> Dict[str, float]:
    drones = drone_discovery.discover()
    uris: List[str] = list()
    for name in drones:
        uri = constants.DRONE_NAME_TO_URI[name]
        uris.append(uri)
    uris = ["radio://0/100/2M/E7E7E7E701"]
    cflib.crtp.init_drivers()
    voltages: Dict[str, float] = dict()
    for uri in uris:
        name = constants.DRONE_URI_TO_NAME[uri]
        maybe_voltage = get_battery_voltage(uri)
        if maybe_voltage is None:
            print(f"[{name}] failed to record battery voltage")
        else:
            print(f"[{name}] recorded battery voltage")
            voltages[name] = maybe_voltage
    return voltages


if __name__ == "__main__":
    from argparse import ArgumentParser

    parser = ArgumentParser()
    parser.add_argument("cfs")
    arguments = vars(parser.parse_args())
    match arguments["cfs"]:
        case "all":
            # TODO: handle exception
            voltages = discover_battery_voltages()
            for name, battery_voltage in voltages.items():
                print(f"[{name}] battery voltage is {round(battery_voltage, 3)} volts")
        case other:
            # TODO: handle exception
            if other in constants.DRONE_NAME_TO_URI.keys():
                uri = constants.DRONE_NAME_TO_URI[other]
                cflib.crtp.init_drivers()
                voltage = get_battery_voltage("radio://0/100/2M/E7E7E7E701")
                print(voltage)
            else:
                print(f"unknown drone: {other}")
