"""
Thin wrapper around the setuptools build backend which adds the build requirements specific to the math
library backend selected by the MATH_LIBRARY_BACKEND environment variable
"""

import os

from setuptools import build_meta as _setuptools_build_meta
from setuptools.build_meta import *  # noqa: F401, F403


def _math_library_backend_requirements():
    if os.environ.get("MATH_LIBRARY_BACKEND") == "Casadi":
        # biorbd_casadi is compiled against the casadi python package (headers, CMake files and library).
        # casadi >= 3.8 is required as previous versions use an older C++ ABI than the default compilers
        return ["casadi>=3.8"]
    return []


def get_requires_for_build_wheel(config_settings=None):
    return _setuptools_build_meta.get_requires_for_build_wheel(config_settings) + _math_library_backend_requirements()


def get_requires_for_build_editable(config_settings=None):
    return _setuptools_build_meta.get_requires_for_build_editable(config_settings) + _math_library_backend_requirements()
