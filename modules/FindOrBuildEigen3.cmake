macro(FindOrBuildEigen3)   
    # Try to find EIGEN3 first
    set(EIGEN3_FOUND FALSE CACHE INTERNAL "EIGEN3 found or built")
    find_package(EIGEN3 QUIET)

    if(EIGEN3_FOUND)
        set(EIGEN3_IS_BUILT FALSE)
        if(NOT DEFINED EIGEN3_INCLUDE_DIR OR EIGEN3_INCLUDE_DIR STREQUAL "")
            # Modern CMake does not set INCLUDE_DIR, so make it retro-compatible
            get_target_property(EIGEN3_INCLUDE_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
        endif()
    else()
        message(STATUS "EIGEN3 not found, downloading version 3.4 and installing from GitLab")
        include(ExternalProject)

        set(EIGEN3_IS_BUILT TRUE)
        set(EIGEN3_INSTALL_DIR "${CMAKE_BINARY_DIR}/Eigen3_install")

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
        set(EIGEN3_DIR "${EIGEN3_INSTALL_DIR}/share/eigen3/cmake")
        set(EIGEN3_INCLUDE_DIR "${EIGEN3_INSTALL_DIR}/include/eigen3")
        set(EIGEN3_FOUND TRUE INTERNAL "EIGEN3 found or built")
    endif()


endmacro()