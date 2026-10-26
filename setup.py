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

# Get the MATH_LIBRARY_BACKEND from environment variable
math_library_backend = os.environ.get("MATH_LIBRARY_BACKEND")
if math_library_backend not in ["Casadi", "Eigen3"]:
    raise RuntimeError("MATH_LIBRARY_BACKEND environment variable must be set to either 'Casadi' or 'Eigen3'")

install_requires = ["numpy", "scipy"]
backend_cmake_args = []
if math_library_backend == "Eigen3":
    package_name = "biorbd"
else:
    package_name = "biorbd_casadi"
    try:
        # casadi is a build requirement of this backend (see build_backend/biorbd_build_backend.py)
        import casadi
    except ImportError:
        # This happens when the build requirements are being determined, that is before casadi is installed
        casadi = None

    if casadi is not None:
        backend_cmake_args.append(f"-DCasadi_DIR={Path(casadi.__file__).parent / 'cmake'}")
        # The casadi library is not bundled in the wheel, the one of the casadi package is used instead. As casadi
        # does not guarantee binary compatibility between versions, the exact version compiled against is required
        install_requires.append(f"casadi=={casadi.__version__}")

# Create an empty package folder if it doesn't exist (stub for setup.py)
Path(package_name).mkdir(exist_ok=True)

setup(
    name=package_name,
    version=version,
    author="Michaud, Benjamin and Begon, Mickaël",
    description="Biomechanical add-ons to the RigidBody Dynamics Library",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/pyomeca/biorbd",
    license="MIT",
    python_requires=">=3.10",
    install_requires=install_requires,
    packages=[package_name],
    cmake_args=[
        "-DBUILD_EXAMPLE:BOOL=OFF",
        "-DBINDER_PYTHON3:BOOL=ON",
        f"-DCMAKE_INSTALL_BINDIR={package_name}",
        f"-DCMAKE_INSTALL_LIBDIR={package_name}",
        f"-DMATH_LIBRARY_BACKEND={math_library_backend}",
        f"-DINSTALL_DEPENDENCIES_PREFIX={dir_path}/{CMAKE_INSTALL_DIR()}",
        # Install the python package at the root of the cmake-install folder so scikit-build packages it as a
        # regular package (and not as wheel data, which breaks the relative RPATH set by auditwheel/delocate)
        "-DPython3_SITELIB_INSTALL=.",
        f"-DOVERRIDE_LIB_SUFFIX=''",
        # Do not pick up dependencies registered by previous builds (e.g. a deleted _skbuild folder)
        "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF",
        *backend_cmake_args,
    ],
)
