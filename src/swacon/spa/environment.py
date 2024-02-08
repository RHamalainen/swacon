from pathlib import Path
from threading import Lock

import swacon.spa.constants as constants
from swacon.data_structures.environment.environment import Environment

# Environment is saved to this file
PATH_ENVIRONMENT = Path("environment.nc").resolve()
# Environment is protected with this lock
ENVIRONMENT_LOCK = Lock()
# The actual operating environment, also known as the importance field
ENVIRONMENT = None


def initialize() -> None:
    """Initialize importance field

    Must be called before using the importance field"""
    global ENVIRONMENT
    global ENVIRONMENT_LOCK
    with ENVIRONMENT_LOCK:
        ENVIRONMENT = Environment(width=constants.TILES_IN_PHYSICAL_X, height=constants.TILES_IN_PHYSICAL_Y, scenario=0)
        # Save initial environment to a file
        ENVIRONMENT.importance_field.to_netcdf(PATH_ENVIRONMENT)
