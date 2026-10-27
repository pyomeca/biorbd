"""
Thin wrapper around the scikit-build-core build backend which selects the math library backend of biorbd.

The math library backend is chosen (by order of priority) with the "backend" config setting
(e.g. pip install . -C backend=casadi), the MATH_LIBRARY_BACKEND environment variable, or defaults to Eigen3.
Depending on that choice, the package is named either biorbd or biorbd_casadi. As the [project] table of
pyproject.toml cannot have a dynamic name (PEP 621), the name is injected into the metadata computed by
scikit-build-core. Everything else is configured in the [tool.scikit-build] table of pyproject.toml
"""

import contextlib
import copy
import os
from pathlib import Path

from scikit_build_core import build as _scikit_build_core
from scikit_build_core.build import sdist as _scikit_build_core_sdist
from scikit_build_core.build import wheel as _scikit_build_core_wheel

_MATH_LIBRARY_BACKENDS = {"eigen": "Eigen3", "eigen3": "Eigen3", "casadi": "Casadi"}
_PACKAGE_NAMES = {"Eigen3": "biorbd", "Casadi": "biorbd_casadi"}


def _math_library_backend(config_settings):
    value = config_settings.pop("backend", None) or os.environ.get("MATH_LIBRARY_BACKEND") or "Eigen3"
    try:
        return _MATH_LIBRARY_BACKENDS[value.lower()]
    except KeyError:
        raise ValueError(f"The math library backend must be either 'Eigen3' or 'Casadi' (got '{value}')") from None


def _prepare(config_settings):
    """
    Return the config settings to send to scikit-build-core, the name of the package and its extra dependencies
    """
    config_settings = dict(config_settings or {})
    math_library_backend = _math_library_backend(config_settings)
    package_name = _PACKAGE_NAMES[math_library_backend]

    config_settings["cmake.define.MATH_LIBRARY_BACKEND"] = math_library_backend
    # Everything is installed in the python package (the python files, the libraries and the dependencies)
    config_settings["wheel.install-dir"] = package_name

    dependencies = []
    if math_library_backend == "Casadi":
        try:
            # casadi is a build requirement of this backend (see _requirements below)
            import casadi
        except ImportError:
            # This happens when the build requirements are being determined, that is before casadi is installed
            casadi = None

        if casadi is not None:
            config_settings["cmake.define.Casadi_DIR"] = str(Path(casadi.__file__).parent / "cmake")
            # The casadi library is not bundled in the wheel, the one of the casadi package is used instead. As casadi
            # does not guarantee binary compatibility between versions, the exact version compiled against is required
            dependencies.append(f"casadi=={casadi.__version__}")

    return config_settings, package_name, dependencies


def _requirements(package_name):
    if package_name == _PACKAGE_NAMES["Casadi"]:
        # biorbd_casadi is compiled against the casadi python package (headers, CMake files and library).
        # casadi >= 3.8 is required as previous versions use an older C++ ABI than the default compilers
        return ["casadi>=3.8"]
    return []


@contextlib.contextmanager
def _project_metadata(package_name, dependencies):
    """
    Inject the name and the extra dependencies of the package in the [project] table read by scikit-build-core
    """
    get_standard_metadata = _scikit_build_core_wheel.get_standard_metadata

    def _get_standard_metadata(pyproject_dict, *args, **kwargs):
        pyproject_dict = copy.deepcopy(dict(pyproject_dict))
        project = pyproject_dict["project"]
        project["name"] = package_name
        project["dependencies"] = [*project.get("dependencies", []), *dependencies]
        return get_standard_metadata(pyproject_dict, *args, **kwargs)

    _scikit_build_core_wheel.get_standard_metadata = _get_standard_metadata
    _scikit_build_core_sdist.get_standard_metadata = _get_standard_metadata
    try:
        yield
    finally:
        _scikit_build_core_wheel.get_standard_metadata = get_standard_metadata
        _scikit_build_core_sdist.get_standard_metadata = get_standard_metadata


def get_requires_for_build_wheel(config_settings=None):
    config_settings, package_name, _ = _prepare(config_settings)
    return _scikit_build_core.get_requires_for_build_wheel(config_settings) + _requirements(package_name)


def get_requires_for_build_editable(config_settings=None):
    config_settings, package_name, _ = _prepare(config_settings)
    return _scikit_build_core.get_requires_for_build_editable(config_settings) + _requirements(package_name)


def get_requires_for_build_sdist(config_settings=None):
    config_settings, _, _ = _prepare(config_settings)
    return _scikit_build_core.get_requires_for_build_sdist(config_settings)


def prepare_metadata_for_build_wheel(metadata_directory, config_settings=None):
    config_settings, package_name, dependencies = _prepare(config_settings)
    with _project_metadata(package_name, dependencies):
        return _scikit_build_core.prepare_metadata_for_build_wheel(metadata_directory, config_settings)


def prepare_metadata_for_build_editable(metadata_directory, config_settings=None):
    config_settings, package_name, dependencies = _prepare(config_settings)
    with _project_metadata(package_name, dependencies):
        return _scikit_build_core.prepare_metadata_for_build_editable(metadata_directory, config_settings)


def build_wheel(wheel_directory, config_settings=None, metadata_directory=None):
    config_settings, package_name, dependencies = _prepare(config_settings)
    with _project_metadata(package_name, dependencies):
        return _scikit_build_core.build_wheel(wheel_directory, config_settings, metadata_directory)


def build_editable(wheel_directory, config_settings=None, metadata_directory=None):
    config_settings, package_name, dependencies = _prepare(config_settings)
    with _project_metadata(package_name, dependencies):
        return _scikit_build_core.build_editable(wheel_directory, config_settings, metadata_directory)


def build_sdist(sdist_directory, config_settings=None):
    config_settings, package_name, dependencies = _prepare(config_settings)
    with _project_metadata(package_name, dependencies):
        return _scikit_build_core.build_sdist(sdist_directory, config_settings)
