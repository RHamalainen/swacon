# TODO: rename to "configuration.py"

# Global configuration for all services

import logging

# TODO: maybe use path.global_var format
from rpyc.core.protocol import DEFAULT_CONFIG

logging.basicConfig(level=logging.INFO)
DEFAULT_CONFIG["allow_pickle"] = True
