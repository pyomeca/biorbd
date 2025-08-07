macro(FindOrBuildEigen3)   
    # Try to find EIGEN3 first
    find_package(Eigen3 QUIET
        PATHS ${INSTALL_DEPENDENCIES_PREFIX}
    )

    if(EIGEN3_FOUND AND NOT EIGEN3_IS_BUILT)
        message (STATUS "EIGEN3 found")

        set(EIGEN3_IS_BUILT FALSE)
        if(NOT DEFINED EIGEN3_INCLUDE_DIR OR EIGEN3_INCLUDE_DIR STREQUAL "")
            # Modern CMake does not set INCLUDE_DIR, so make it retro-compatible
            get_target_property(EIGEN3_INCLUDE_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
        endif()
    else()
        message(STATUS "EIGEN3 not found, using version 3.4 from GitLab")
        set(EIGEN3_IS_BUILT TRUE)
        include(ExternalProject)

        if (NOT INSTALL_DEPENDENCIES_PREFIX OR INSTALL_DEPENDENCIES_PREFIX STREQUAL "")
            set(EIGEN3_INSTALL_DIR "${CMAKE_BINARY_DIR}/Eigen3_install")
        else()
            set(EIGEN3_INSTALL_DIR ${INSTALL_DEPENDENCIES_PREFIX})
        endif()

        ExternalProject_Add(Eigen3_external
            GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
            GIT_TAG 3.4
            CMAKE_ARGS
                -DCMAKE_INSTALL_PREFIX=${EIGEN3_INSTALL_DIR}
                -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                -DCMAKE_POSITION_INDEPENDENT_CODE=ON
            BUILD_BYPRODUCTS "${EIGEN3_INSTALL_DIR}/include/eigen3/Eigen/Core"
        )

        # Define outputs
        add_library(EIGEN3_BUILD INTERFACE IMPORTED)
        add_dependencies(EIGEN3_BUILD Eigen3_external)
        set(EIGEN3_DIR "${EIGEN3_INSTALL_DIR}/share/eigen3/cmake")
        set(EIGEN3_INCLUDE_DIR "${EIGEN3_INSTALL_DIR}/include/eigen3")
        set(EIGEN3_FOUND TRUE)
    endif()


endmacro()