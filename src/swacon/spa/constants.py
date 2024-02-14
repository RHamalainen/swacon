from pathlib import Path

from swacon.configuration.radio import *  # noqa: F401, F403
from swacon.configuration.environment import *  # noqa: F401, F403
from swacon.configuration.motion_capture import *  # noqa: F401, F403

PATH_OUTPUT_ENVIRONMENT_FOLDER = Path("./output/environment/").resolve()
PATH_OUTPUT_FLIGHT_DATA_FOLDER = Path("./output/flight-data/").resolve()
