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
    # The release archive (rather than a git clone) is downloaded and verified against its hash. The archive can
    # also be provided locally (e.g. from a CI cache) using the BIORBD_EIGEN3_ARCHIVE environment variable (an
    # absolute path, or a path relative to the root of biorbd)
    set(EIGEN3_ARCHIVE_URL "https://gitlab.com/libeigen/eigen/-/archive/5.0.1/eigen-5.0.1.tar.gz")
    if(DEFINED ENV{BIORBD_EIGEN3_ARCHIVE})
        get_filename_component(EIGEN3_LOCAL_ARCHIVE "$ENV{BIORBD_EIGEN3_ARCHIVE}" ABSOLUTE
            BASE_DIR "${CMAKE_SOURCE_DIR}")
        if(EXISTS "${EIGEN3_LOCAL_ARCHIVE}")
            set(EIGEN3_ARCHIVE_URL "${EIGEN3_LOCAL_ARCHIVE}")
        endif()
    endif()
    message(STATUS "EIGEN3 not found, using version 5.0.1 from ${EIGEN3_ARCHIVE_URL}")
    include(ExternalProject)

    if (NOT INSTALL_DEPENDENCIES_PREFIX OR INSTALL_DEPENDENCIES_PREFIX STREQUAL "")
        set(EIGEN3_INSTALL_DIR "${CMAKE_BINARY_DIR}/Eigen3_install")
    else()
        set(EIGEN3_INSTALL_DIR ${INSTALL_DEPENDENCIES_PREFIX})
    endif()

    ExternalProject_Add(Eigen3_external
        URL "${EIGEN3_ARCHIVE_URL}"
        URL_HASH SHA256=e9c326dc8c05cd1e044c71f30f1b2e34a6161a3b6ecf445d56b53ff1669e3dec
        CMAKE_ARGS
            -DCMAKE_INSTALL_PREFIX=${EIGEN3_INSTALL_DIR}
            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON
            # Only the headers are needed. Eigen's tests, blas and lapack folders enable Fortran when a Fortran
            # compiler is found, which fails on the Windows CI where MinGW's gfortran is mixed with the MSVC flags
            -DBUILD_TESTING=OFF
            -DCMAKE_Fortran_COMPILER=CMAKE_Fortran_COMPILER-NOTFOUND
            -DCMAKE_EXPORT_PACKAGE_REGISTRY=OFF
            -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON
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