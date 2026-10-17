import warnings

warnings.warn(
    "Please update your 'hppfcl' imports to 'coal'", category=DeprecationWarning
)

from coal import Transform3s as Transform3f  # noqa
from coal import *
from coal import __raw_version__, __version__  # noqa
