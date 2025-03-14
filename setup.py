from skbuild import setup
from pathlib import Path
import re
import os
import sys
import platform
import pathlib


def get_install_site_packages():
    # Get the local site packages folder for the installation that is used by "pip install ."
    python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
    platform_name = platform.system().lower()  # e.g., 'linux', 'windows', 'darwin'
    
    # Expected path inside _skbuild (before it exists)
    fake_site_packages = pathlib.Path(f"_skbuild/{platform_name}/cmake-install/lib/{python_version}/site-packages")
    
    return fake_site_packages


dir_path = os.path.dirname(os.path.realpath(__file__))


try:
    import pypandoc
    long_description = pypandoc.convert("README.md", "r")
except(IOError, ImportError):
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


setup(
    # NOTE: Could still add stuff like homepage or author mail, but since this isn't used to redistribute, not important
    name="biorbd",
    version=version,
    author="Michaud, Benjamin and Begon, Mickaël",
    description="Biomechanical add-ons to the RigidBody Dynamics Library",
    long_description=long_description,
    long_description_content_type= "text/markdown",
    url = "https://github.com/pyomeca/biorbd",
    license="MIT",
    packages=["biorbd"],
    cmake_args=[
        "-DBUILD_EXAMPLE:BOOL=OFF",
        "-DBINDER_PYTHON3:BOOL=ON",
        "-DCMAKE_INSTALL_BINDIR=biorbd",
        "-DCMAKE_INSTALL_LIBDIR=biorbd",
        "-DMATH_LIBRARY_BACKEND=Casadi",
        "-DINSTALL_DEPENDENCIES_PREFIX=biorbd",
        f"-DPython3_SITELIB_INSTALL={get_install_site_packages()}"
    ],
)