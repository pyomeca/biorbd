from skbuild import setup
from skbuild.constants import CMAKE_INSTALL_DIR
from pathlib import Path
import re
import os


dir_path = os.path.dirname(os.path.realpath(__file__))


try:
    import pypandoc

    long_description = pypandoc.convert("README.md", "r")
except (IOError, ImportError):
    long_description = open("README.md").read()

with open(f"{dir_path}/CMakeLists.txt") as file:
    for line in file:
        match = re.search(re.compile("project\\(biorbd VERSION (\\d*\\.\\d*\\.\\d*)\\)"), line)
        if match is not None:
            version = match[1]
            break
    else:
        raise RuntimeError("Version not found")

# Create an empty 'biorbd' folder if it doesn't exist (stub for setup.py)
Path("biorbd").mkdir(exist_ok=True)

# Get the MATH_LIBRARY_BACKEND from environment variable
math_library_backend = os.environ.get("MATH_LIBRARY_BACKEND")
if math_library_backend not in ["Casadi", "Eigen3"]:
    raise RuntimeError("MATH_LIBRARY_BACKEND environment variable must be set to either 'Casadi' or 'Eigen3'")

if math_library_backend == "Casadi":
    raise RuntimeError(
        "Casadi backend is not supported via pip install in this workspace. Use the CMake/dev build instead."
    )

setup(
    # NOTE: Could still add stuff like homepage or author mail, but since this isn't used to redistribute, not important
    name="biorbd",
    version=version,
    author="Michaud, Benjamin and Begon, Mickaël",
    description="Biomechanical add-ons to the RigidBody Dynamics Library",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/pyomeca/biorbd",
    license="MIT",
    packages=["biorbd"],
    cmake_args=[
        "-DBUILD_EXAMPLE:BOOL=OFF",
        "-DBINDER_PYTHON3:BOOL=ON",
        "-DCMAKE_INSTALL_BINDIR=biorbd",
        "-DCMAKE_INSTALL_LIBDIR=biorbd",
        f"-DMATH_LIBRARY_BACKEND={math_library_backend}",
        f"-DINSTALL_DEPENDENCIES_PREFIX={dir_path}/{CMAKE_INSTALL_DIR()}",
        # Install the python package at the root of the cmake-install folder so scikit-build packages it as a
        # regular package (and not as wheel data, which breaks the relative RPATH set by auditwheel/delocate)
        "-DPython3_SITELIB_INSTALL=.",
        f"-DOVERRIDE_LIB_SUFFIX=''",
        # Do not pick up dependencies registered by previous builds (e.g. a deleted _skbuild folder)
        "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF",
    ],
)
