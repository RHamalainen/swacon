import time
import logging
import traceback
from typing import List, Dict
from functools import partial
from threading import Lock, Event

import cflib.crtp
from cflib.crtp.radiodriver import RadioDriver

import swacon.spa.constants as constants


DRONE_NAME_TO_LINK_ERROR_EVENT_LOCK = Lock()
DRONE_NAME_TO_LINK_ERROR_EVENT = {
    "cf1": Event(),
    "cf2": Event(),
    "cf3": Event(),
    "cf4": Event(),
    "cf5": Event(),
    "cf6": Event(),
}

LINK_LIFETIME_THRESHOLD_SECONDS = 5.0
SLEEP_TIME_SECONDS = 0.5


def link_quality_callback(name: str, link_quality: float) -> None:
    assert isinstance(name, str)
    assert isinstance(link_quality, float)
    # print(f"link quality: {link_quality}")


def link_error_callback(name: str, error: str) -> None:
    assert isinstance(name, str)
    assert isinstance(error, str)
    print(f"[{name}] link error: {error}")
    with DRONE_NAME_TO_LINK_ERROR_EVENT_LOCK:
        DRONE_NAME_TO_LINK_ERROR_EVENT[name].set()


def discover() -> List[str]:
    # Establish links to drones
    drivers: Dict[str, RadioDriver] = dict()
    for name, uri in constants.DRONE_NAME_TO_URI.items():
        driver = cflib.crtp.RadioDriver()
        try:
            link_quality_cb = partial(link_quality_callback, name)
            link_error_cb = partial(link_error_callback, name)
            driver.connect(uri, link_quality_cb, link_error_cb)
            drivers[name] = driver
            print(f"[{name}] connected link")
        except Exception as exception:
            traceback.print_exc()
    # Test that lifetimes of links exceed threshold
    present_drones: List[str] = list()
    try:
        discovery_end = time.time() + LINK_LIFETIME_THRESHOLD_SECONDS
        while time.time() < discovery_end:
            time.sleep(SLEEP_TIME_SECONDS)
            if DRONE_NAME_TO_LINK_ERROR_EVENT_LOCK.acquire(False):
                if 0 < len(DRONE_NAME_TO_LINK_ERROR_EVENT):
                    failed_links = list()
                    for name, event in DRONE_NAME_TO_LINK_ERROR_EVENT.items():
                        if event.is_set():
                            failed_links.append(name)
                            print(f"[{name}] link failed")
                    for name in failed_links:
                        DRONE_NAME_TO_LINK_ERROR_EVENT.pop(name)
                else:
                    # TODO: deprecated?
                    do_continue = False
                DRONE_NAME_TO_LINK_ERROR_EVENT_LOCK.release()
        with DRONE_NAME_TO_LINK_ERROR_EVENT_LOCK:
            for name, event in DRONE_NAME_TO_LINK_ERROR_EVENT.items():
                if not event.is_set():
                    present_drones.append(name)
    except Exception as exception:
        traceback.print_exc()
    finally:
        for name, driver in drivers.items():
            driver.close()
            print(f"[{name}] disconnected link")
    return present_drones


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers()
    present_drones = discover()
    if 0 < len(present_drones):
        print(f"detected {len(present_drones)} drones:")
        for i, name in enumerate(present_drones):
            print(f" - {i + 1}/{len(present_drones)}: {name}")
    else:
        print(f"detected no drones")
