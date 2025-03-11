include(ExternalProject)

# Try to find RBDL first
find_package(RBDL QUIET)

if(NOT RBDL_FOUND)
    message(STATUS "RBDL not found, using ExternalProject_Add to fetch it...")

    set(RBDL_INSTALL_DIR "${CMAKE_BINARY_DIR}/rbdl_install")

    ExternalProject_Add(rbdl_external
        GIT_REPOSITORY https://github.com/rbdl/rbdl.git
        GIT_TAG master
        CMAKE_ARGS
            -DCMAKE_INSTALL_PREFIX=${RBDL_INSTALL_DIR}
            -DRBDL_BUILD_STATIC=ON
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON  # <-- This forces -fPIC
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
    add_library(RBDL_EXTERNAL STATIC IMPORTED)
    set_target_properties(RBDL_EXTERNAL PROPERTIES
        IMPORTED_LOCATION "${RBDL_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${RBDL_INCLUDE_DIR}"
    )
    add_dependencies(RBDL_EXTERNAL rbdl_external)

    # Windows DLL runtime setup
    if(WIN32)
        set_target_properties(RBDL_EXTERNAL PROPERTIES
            IMPORTED_IMPLIB "${RBDL_LIBRARY}"  # Import library for linking
            IMPORTED_LOCATION "${RBDL_RUNTIME_DIR}/rbdl.dll"  # Runtime DLL location
        )
    endif()

    set(RBDL_FOUND TRUE CACHE INTERNAL "RBDL found or built")
endif()

