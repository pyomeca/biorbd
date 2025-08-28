function(FindOrBuildEigen3)
    # -------------------------------
    # Try finding existing install
    # -------------------------------
    find_package(Eigen3 QUIET
        PATHS ${INSTALL_DEPENDENCIES_PREFIX} ${CMAKE_INSTALL_PREFIX}
    )

    if(TARGET Eigen3::Eigen)
        message (STATUS "EIGEN3 found")
        return()
    endif()

    # -------------------------------
    # If not found → build from source
    # -------------------------------
    message(STATUS "EIGEN3 not found, using version 3.4 from GitLab")
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

     # Define outputs (header-only library)
    set(Eigen3_INCLUDE_DIRECTORIES "${EIGEN3_INSTALL_DIR}/include/eigen3")
    file(MAKE_DIRECTORY "${Eigen3_INCLUDE_DIRECTORIES}")
    
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${Eigen3_INCLUDE_DIRECTORIES}"
    )
    set(Eigen3_DIR "${EIGEN3_INSTALL_DIR}/share/eigen3/cmake" PARENT_SCOPE)
    add_dependencies(Eigen3::Eigen Eigen3_external)
endfunction()