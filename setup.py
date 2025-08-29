from skbuild import setup
from pathlib import Path
import re
import os
import sys
import platform


def get_install_base():
    current_folder = Path(__file__).parent
    platform_name = platform.system().lower()

    if platform_name == "linux":
        architecture = platform.architecture()[0]
        if architecture == "64bit":
            architecture = "x86_64"
        elif architecture == "32bit":
            architecture = "i686"
        else:
            raise RuntimeError(f"Unsupported architecture: {architecture}")

        python_version = f"{sys.version_info.major}.{sys.version_info.minor}"
        platform_name = f"linux-{architecture}-{python_version}"

    elif platform_name == "darwin":
        # Get the number of macos version (e.g. 13.0)
        version = platform.mac_ver()[0].split(".")
        version = f"{version[0]}.0"

        architecture = platform.architecture()[0]
        if architecture == "64bit":
            architecture = "x86_64"
        else:
            raise RuntimeError(f"Unsupported architecture: {architecture}")

        python_version = f"{sys.version_info.major}.{sys.version_info.minor}"
        platform_name = f"macosx-{version}-x86_64-{python_version}"

    elif platform_name == "windows":
        architecture = platform.architecture()[0]
        if architecture == "64bit":
            architecture = "amd64"
        else:
            raise RuntimeError(f"Unsupported architecture: {architecture}")

        python_version = f"{sys.version_info.major}.{sys.version_info.minor}"
        platform_name = f"win-{architecture}-{python_version}"

    return f"{current_folder}/_skbuild/{platform_name}/cmake-install"


def get_install_site_packages():
    # Get the local site packages folder for the installation that is used by "pip install ."
    python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"

    import site

    site_packages = site.getsitepackages()
    if not site_packages:
        raise RuntimeError("Unable to find site-packages directory")

    for site_package in site_packages:
        if "site-packages" in site_package:
            if python_version in site_package:
                return Path(f"{get_install_base()}/lib/{python_version}/site-packages")
            else:
                return Path(f"{get_install_base()}/lib/site-packages")

    return Path(f"{get_install_base()}/lib/{python_version}/site-packages")


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
        f"-DINSTALL_DEPENDENCIES_PREFIX={get_install_base()}",
        f"-DPython3_SITELIB_INSTALL={get_install_site_packages()}",
        f"-DOVERRIDE_LIB_SUFFIX=''",
    ],
)
