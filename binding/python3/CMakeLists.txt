cmake_minimum_required(VERSION 3.14)
project(_${BIORBD_NAME})

# Windows needs a bigobj option
if(MSVC)
    set(COMPILE_FLAGS "/bigobj")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMPILE_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMPILE_FLAGS}")
endif()

# The python headers of Windows are written for MSVC. With MinGW (used for the casadi backend, as the casadi
# package on PyPI is compiled with it), MS_WIN64 must be defined by hand (otherwise the python types have the
# wrong size) and _hypot (used by pyconfig.h) does not exist in std
if(MINGW)
    add_compile_definitions(MS_WIN64 _hypot=hypot)
endif()

# Sometimes find_package finds one include folder short, so check if "rbdl-casadi" subdirectory exists
set(RBDLCasadi_INCLUDE_DIR_PYTHON ${RBDLCasadi_INCLUDE_DIR})
if(EXISTS "${RBDLCasadi_INCLUDE_DIR_PYTHON}/rbdl-casadi")
    # If it exists, override the include dir to point to it
    set(RBDLCasadi_INCLUDE_DIR_PYTHON "${RBDLCasadi_INCLUDE_DIR_PYTHON}/rbdl-casadi")
endif()

# We need to trick MSVC (and why not all of them) into keeping the rbdl-casadi as the first import. If we
# just add it, it is pushed at the end because BIORBD will also includes it but
# as an external:import. Added the /../ tricks the parser into not recognizing
# that it is in fact the same directory
set(FORCED_FIRST_INCLUDE_DIRS "")
if(${MATH_LIBRARY_BACKEND} STREQUAL "Casadi")
    set(FORCED_FIRST_INCLUDE_DIRS
        "${RBDLCasadi_INCLUDE_DIR_PYTHON}/../rbdl-casadi"
    )
endif()

# Set up swig
if(DEFINED SWIG_EXECUTABLE AND NOT EXISTS "${SWIG_EXECUTABLE}")
    unset(SWIG_EXECUTABLE CACHE)
endif()
if(DEFINED SWIG_DIR AND NOT EXISTS "${SWIG_DIR}")
    unset(SWIG_DIR CACHE)
endif()
find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})

# include python
set(Python3_FIND_VIRTUALENV FIRST)
set(Python3_FIND_STRATEGY LOCATION)
if(DEFINED ENV{CONDA_PREFIX} AND NOT DEFINED Python3_ROOT_DIR)
    set(Python3_ROOT_DIR "$ENV{CONDA_PREFIX}")
endif()
if(DEFINED ENV{CONDA_PREFIX} AND EXISTS "$ENV{CONDA_PREFIX}/bin/python" AND NOT DEFINED Python3_EXECUTABLE)
    set(Python3_EXECUTABLE "$ENV{CONDA_PREFIX}/bin/python")
endif()

# Detect whether the selected interpreter is a free-threaded (no-GIL) build.
execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "import sysconfig; print(1 if sysconfig.get_config_var('Py_GIL_DISABLED') else 0)"
    OUTPUT_VARIABLE PYTHON3_GIL_DISABLED
    OUTPUT_STRIP_TRAILING_WHITESPACE
)


# On some recent CMake + Python toolchains (notably free-threaded builds),
# Development may be reported missing even when NumPy is available.
find_package(Python3 REQUIRED
COMPONENTS Interpreter Development.Module NumPy
)
find_package(Python3 QUIET COMPONENTS Development.Module)

if(Python3_Development.Module_FOUND)
include_directories(${Python3_INCLUDE_DIRS})
else()
execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "import sysconfig; print(sysconfig.get_path('include'))"
    OUTPUT_VARIABLE PYTHON3_INCLUDE_DIR_FROM_INTERPRETER
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(PYTHON3_INCLUDE_DIR_FROM_INTERPRETER)
    include_directories(${PYTHON3_INCLUDE_DIR_FROM_INTERPRETER})
else()
    message(FATAL_ERROR "Could not determine Python include directory. Install Python development headers or use a Python environment with dev files.")
endif()
endif()

include_directories(${Python3_NumPy_INCLUDE_DIRS})


if(WIN32)
    link_directories(${Python3_RUNTIME_LIBRARY_DIRS}/libs)
endif()

# Create the .i file
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${BIORBD_NAME_NO_SUFFIX}_python.i.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${BIORBD_NAME_NO_SUFFIX}_python.i" @ONLY)

# Add swig module
set(CMAKE_SWIG_FLAGS "")
set(I_SOURCE_FILE "${CMAKE_CURRENT_BINARY_DIR}/${BIORBD_NAME_NO_SUFFIX}_python.i")

set_property(SOURCE "${I_SOURCE_FILE}"
    PROPERTY CPLUSPLUS ON
)
SWIG_ADD_LIBRARY(${PROJECT_NAME}
    TYPE MODULE
    LANGUAGE "python"
    SOURCES "${I_SOURCE_FILE}"
)
if(TARGET ${PROJECT_NAME})
    set(PYTHON_PROJECT_NAME ${PROJECT_NAME})
elseif(TARGET _${PROJECT_NAME})
    set(PYTHON_PROJECT_NAME _${PROJECT_NAME})
else()
    message(FATAL_ERROR "Could not find generated SWIG target for ${PROJECT_NAME}")
endif()
set(PYTHON3_EXTENSION_LINK_TARGET "")
if(TARGET Python3::Module)
    set(PYTHON3_EXTENSION_LINK_TARGET Python3::Module)
elseif(TARGET Python3::Python)
    set(PYTHON3_EXTENSION_LINK_TARGET Python3::Python)
elseif(Python3_LIBRARY)
    set(PYTHON3_EXTENSION_LINK_TARGET "${Python3_LIBRARY}")
elseif(Python3_LIBRARIES)
    set(PYTHON3_EXTENSION_LINK_TARGET "${Python3_LIBRARIES}")
endif()

target_link_libraries(${PYTHON_PROJECT_NAME} PRIVATE
    "${BIORBD_NAME}"
)
if(PYTHON3_EXTENSION_LINK_TARGET)
    target_link_libraries(${PYTHON_PROJECT_NAME} PRIVATE ${PYTHON3_EXTENSION_LINK_TARGET})
endif()

if(PYTHON3_GIL_DISABLED STREQUAL "1")
    target_compile_definitions(${PYTHON_PROJECT_NAME} PRIVATE SWIGPYTHON_NOGIL)
endif()

# Add headers
target_include_directories(${PYTHON_PROJECT_NAME} PRIVATE
    "${FORCED_FIRST_INCLUDE_DIRS}"
    "${IPOPT_INCLUDE_DIR}"
    "${Python3_INCLUDE_DIRS}"
    "${Python3_NumPy_INCLUDE_DIRS}"
)

if(APPLE)
    set_target_properties(${PYTHON_PROJECT_NAME} PROPERTIES
        SUFFIX ".so"
        INSTALL_RPATH ${${BIORBD_NAME}_BIN_FOLDER})
    set_property(TARGET ${PYTHON_PROJECT_NAME} APPEND PROPERTY
        LINK_FLAGS "-flat_namespace -undefined suppress")
endif()
set_target_properties(${PYTHON_PROJECT_NAME} PROPERTIES
    PREFIX ""
    INSTALL_RPATH "${${BIORBD_NAME}_BIN_FOLDER}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)
if(DEFINED BIORBD_CASADI_WHEEL_RPATH)
    # See the main CMakeLists.txt
    set_property(TARGET ${PYTHON_PROJECT_NAME} APPEND PROPERTY INSTALL_RPATH ${BIORBD_CASADI_WHEEL_RPATH})
endif()

# Define the python install
# Priority:
# 1) Python3_SITELIB_INSTALL from setup.py (relative path for scikit-build staging)
# 2) Python3_SITELIB from CMake FindPython (native CMake installs, platform-specific)
# 3) Generic relative fallback for environments that don't expose either variable
if(DEFINED Python3_SITELIB_INSTALL AND NOT "${Python3_SITELIB_INSTALL}" STREQUAL "")
    set(PYTHON_INSTALL_PREFIX "${Python3_SITELIB_INSTALL}")
elseif(DEFINED Python3_SITELIB AND NOT "${Python3_SITELIB}" STREQUAL "")
    set(PYTHON_INSTALL_PREFIX "${Python3_SITELIB}")
else()
    set(PYTHON_INSTALL_PREFIX "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
endif()
set(PYTHON_INSTALL_PREFIX "${PYTHON_INSTALL_PREFIX}" CACHE PATH
    "Path where to install the python binder"
)

# Install to the proper directories (defaut eigen
if(${MATH_LIBRARY_BACKEND} STREQUAL "Eigen3")
    set(BIORBD_PYTHON_DIR ${BIORBD_NAME_NO_SUFFIX})
elseif(${MATH_LIBRARY_BACKEND} STREQUAL "Casadi")
    set(BIORBD_PYTHON_DIR ${BIORBD_NAME})
endif()
INSTALL(FILES "${CMAKE_CURRENT_BINARY_DIR}/${BIORBD_NAME_NO_SUFFIX}.py"
    DESTINATION "${PYTHON_INSTALL_PREFIX}/${BIORBD_PYTHON_DIR}"
)

file(GLOB PYTHON_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.py")
INSTALL(FILES ${PYTHON_FILES}
    DESTINATION "${PYTHON_INSTALL_PREFIX}/${BIORBD_PYTHON_DIR}"
)
file(GLOB PYTHON_FILES "${CMAKE_CURRENT_SOURCE_DIR}/wrapper/*.py")
INSTALL(FILES ${PYTHON_FILES}
    DESTINATION "${PYTHON_INSTALL_PREFIX}/${BIORBD_PYTHON_DIR}/wrapper"
)

# Check SWIG behavior to choose where to install the extension module.
# SWIG 4.5+ generates relative imports in the wrapper ("from . import _module"),
# so the compiled module must live inside the Python package directory.
if(${SWIG_VERSION} MATCHES "^3(\.[0-9]*)*$")
    set(PYTHON_PROJECT_INSTALL_DESTINATION "${PYTHON_INSTALL_PREFIX}")
elseif(${SWIG_VERSION} MATCHES "^4(\.[0-9]*)*$")
    set(PYTHON_PROJECT_INSTALL_DESTINATION "${PYTHON_INSTALL_PREFIX}/${BIORBD_PYTHON_DIR}")
else()
    message( FATAL_ERROR "Swig must be version 3 or 4" )
endif()

# biorbd_casadi exchanges casadi objects with the casadi python module. Both SWIG modules must therefore
# share the same SWIG runtime (same runtime version and same type table), otherwise casadi objects are
# not recognized. As this differs between casadi distributions (e.g. conda uses runtime 4 with the default
# table, pip uses runtime 5 with the "casadi" table), ask the installed casadi which one it uses
if(${MATH_LIBRARY_BACKEND} STREQUAL "Casadi")
    execute_process(
        COMMAND "${Python3_EXECUTABLE}" -c
            "import sys, casadi\nfor name in [m for m in sys.modules if m.startswith('swig_runtime_data')]:\n    for attr in [a for a in dir(sys.modules[name]) if a.startswith('type_pointer_capsule')]:\n        print(name[len('swig_runtime_data'):] + ';' + attr[len('type_pointer_capsule'):])"
        OUTPUT_VARIABLE CASADI_SWIG_RUNTIME
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE CASADI_SWIG_RUNTIME_RESULT
    )
    if(NOT CASADI_SWIG_RUNTIME_RESULT EQUAL 0 OR CASADI_SWIG_RUNTIME STREQUAL "")
        message(FATAL_ERROR
            "Could not determine the SWIG runtime of the casadi python module using ${Python3_EXECUTABLE}. "
            "Make sure casadi is installed for that python."
        )
    endif()
    list(GET CASADI_SWIG_RUNTIME 0 CASADI_SWIG_RUNTIME_VERSION)
    list(LENGTH CASADI_SWIG_RUNTIME CASADI_SWIG_RUNTIME_LENGTH)
    if(CASADI_SWIG_RUNTIME_LENGTH GREATER 1)
        list(GET CASADI_SWIG_RUNTIME 1 CASADI_SWIG_TYPE_TABLE)
    else()
        set(CASADI_SWIG_TYPE_TABLE "")
    endif()

    file(STRINGS "${SWIG_DIR}/swigrun.swg" SWIG_RUNTIME_VERSION_LINE
        REGEX "^#define SWIG_RUNTIME_VERSION \"[0-9]+\"")
    string(REGEX MATCH "[0-9]+" SWIG_RUNTIME_VERSION "${SWIG_RUNTIME_VERSION_LINE}")
    if(NOT SWIG_RUNTIME_VERSION STREQUAL CASADI_SWIG_RUNTIME_VERSION)
        message(FATAL_ERROR
            "The casadi python module uses the SWIG runtime version ${CASADI_SWIG_RUNTIME_VERSION}, but SWIG "
            "${SWIG_VERSION} generates runtime version ${SWIG_RUNTIME_VERSION}. Please use a SWIG version "
            "with the same runtime (runtime 4 is SWIG < 4.4.0, runtime 5 is SWIG >= 4.4.0)."
        )
    endif()
    if(NOT CASADI_SWIG_TYPE_TABLE STREQUAL "")
        target_compile_definitions(${PYTHON_PROJECT_NAME} PRIVATE SWIG_TYPE_TABLE=${CASADI_SWIG_TYPE_TABLE})
    endif()
    message(STATUS "Using the SWIG runtime ${CASADI_SWIG_RUNTIME_VERSION} of casadi "
        "(type table: '${CASADI_SWIG_TYPE_TABLE}')")
endif()

# Install the library
INSTALL(TARGETS ${PYTHON_PROJECT_NAME}
    DESTINATION "${PYTHON_PROJECT_INSTALL_DESTINATION}"
)
