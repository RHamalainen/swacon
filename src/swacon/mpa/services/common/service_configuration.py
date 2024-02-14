# TODO: rename to "configuration.py"

# Global configuration for all services

import logging

logging.basicConfig(level=logging.INFO)

from rpyc.core.protocol import DEFAULT_CONFIG

DEFAULT_CONFIG["allow_pickle"] = True
