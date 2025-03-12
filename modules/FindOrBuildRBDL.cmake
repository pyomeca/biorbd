macro(FindOrBuildRBDL MATH_BACKEND)
    # Check if MATH_BACKEND is EIGEN or CASADI
    if(NOT (${MATH_BACKEND} STREQUAL "EIGEN" OR ${MATH_BACKEND} STREQUAL "CASADI"))
        message(FATAL_ERROR "FindOrBuildRBDL: Invalid option '${MATH_BACKEND}'. Use 'EIGEN' or 'CASADI'.")
    endif()


    # Try to find RBDL first
    if (${MATH_BACKEND} STREQUAL "EIGEN")
        find_package(RBDL QUIET)

    elseif (${MATH_BACKEND} STREQUAL "CASADI")
        find_package(RBDLCasadi QUIET)
        if (RBDLCasadi_FOUND)
            set(RBDL_FOUND TRUE CACHE INTERNAL "RBDL found or built")

            # Define include and library paths to mimic RBDL eigen which is the format expected by biorbd
            set(RBDL_INCLUDE_DIR ${RBDLCasadi_INCLUDE_DIR}/rbdl-casadi ${RBDLCasadi_INCLUDE_DIR})
            set(RBDL_LIBRARY ${RBDLCasadi_LIBRARY})
        endif()
    endif()


    if(NOT RBDL_FOUND)
        message(STATUS "RBDL not found, downloading and installing from GitHub")
        include(ExternalProject)

        set(RBDL_INSTALL_DIR "${CMAKE_BINARY_DIR}/rbdl_install")

        # Give default value for RBDL_BUILD_STATIC to ON
        option(RBDL_BUILD_STATIC "Build RBDL as a static library" ON)

        if (${MATH_BACKEND} STREQUAL "EIGEN")
            set(RBDL_BUILD_CASADI OFF)
        elseif (${MATH_BACKEND} STREQUAL "CASADI")
            set(RBDL_BUILD_CASADI ON)
        endif()

        ExternalProject_Add(rbdl_external
            GIT_REPOSITORY https://github.com/rbdl/rbdl.git
            GIT_TAG master
            CMAKE_ARGS
                -DCMAKE_INSTALL_PREFIX=${RBDL_INSTALL_DIR}
                -DRBDL_BUILD_STATIC=${RBDL_BUILD_STATIC}
                -DRBDL_BUILD_CASADI=${RBDL_BUILD_CASADI}
                -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        )

        # Define include and library paths
        set(RBDL_INCLUDE_DIR "${RBDL_INSTALL_DIR}/include")

        # Detect correct library extension (OS-independent)
        if(WIN32)
            set(RBDL_LIB_NAME "rbdl.lib")  # Static (.lib) for MSVC
            set(RBDL_RUNTIME_DIR "${RBDL_INSTALL_DIR}/bin")  # DLLs go here
        elseif(APPLE)
            set(RBDL_LIB_NAME "librbdl.dylib")  # macOS dynamic library
        else()
            set(RBDL_LIB_NAME "librbdl.a")  # Linux shared library
        endif()

        set(RBDL_LIBRARY "${RBDL_INSTALL_DIR}/lib/${RBDL_LIB_NAME}")

        # Ensure that the library gets built before linking
        add_library(RBDL STATIC IMPORTED)
        set_target_properties(RBDL PROPERTIES
            IMPORTED_LOCATION "${RBDL_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${RBDL_INCLUDE_DIR}"
        )
        add_dependencies(RBDL rbdl_external)

        # Windows DLL runtime setup
        if(WIN32)
            set_target_properties(RBDL PROPERTIES
                IMPORTED_IMPLIB "${RBDL_LIBRARY}"  # Import library for linking
                IMPORTED_LOCATION "${RBDL_RUNTIME_DIR}/rbdl.dll"  # Runtime DLL location
            )
        endif()

        set(RBDL_FOUND TRUE CACHE INTERNAL "RBDL found or built")
    endif()


endmacro()