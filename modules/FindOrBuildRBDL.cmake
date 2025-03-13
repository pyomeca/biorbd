macro(FindOrBuildRBDL MATH_BACKEND)
    # Check if MATH_BACKEND is EIGEN or CASADI
    if(NOT (${MATH_BACKEND} STREQUAL "EIGEN" OR ${MATH_BACKEND} STREQUAL "CASADI"))
        message(FATAL_ERROR "FindOrBuildRBDL: Invalid option '${MATH_BACKEND}'. Use 'EIGEN' or 'CASADI'.")
    endif()

    
    # Try to find RBDL first
    set(RBDL_FOUND FALSE CACHE INTERNAL "RBDL found or built")
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

    if(RBDL_FOUND)
        set(RBDL_IS_BUILT FALSE)
    else()
        message(STATUS "RBDL not found, downloading and installing from GitHub")
        include(ExternalProject)

        set(RBDL_IS_BUILT TRUE)
        set(RBDL_INSTALL_DIR "${CMAKE_BINARY_DIR}/rbdl_install")

        if (${MATH_BACKEND} STREQUAL "EIGEN")
            set(RBDL_BUILD_CASADI OFF)
            set(RBDL_LIBRARY_SUFFIX "")
        elseif (${MATH_BACKEND} STREQUAL "CASADI")
            set(RBDL_BUILD_CASADI ON)
            set(RBDL_LIBRARY_SUFFIX "-casadi")
        endif()
        
        # Detect correct static library extension (OS-independent)
        if(WIN32)
            set(RBDL_LIB_NAME "rbdl${RBDL_LIBRARY_SUFFIX}.lib") 
        else()
            set(RBDL_LIB_NAME "librbdl${RBDL_LIBRARY_SUFFIX}.a")
        endif()
        set(RBDL_LIBRARY "${RBDL_INSTALL_DIR}/lib/${RBDL_LIB_NAME}")

        ExternalProject_Add(rbdl_external
            GIT_REPOSITORY https://github.com/pariterre/rbdl.git
            GIT_TAG master
            CMAKE_ARGS
                -DCMAKE_INSTALL_PREFIX=${RBDL_INSTALL_DIR}
                -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                -DRBDL_BUILD_STATIC=ON
                -DRBDL_BUILD_CASADI=${RBDL_BUILD_CASADI}
                -DEigen3_DIR=${EIGEN3_DIR}
                -DCasadi_INCLUDE_DIR=${Casadi_INCLUDE_DIR}
                -DCasadi_LIBRARY=${Casadi_LIBRARY}
                -DCasadi_FOUND=${Casadi_FOUND}
                -DCMAKE_POSITION_INDEPENDENT_CODE=ON
            BUILD_BYPRODUCTS "${RBDL_INSTALL_DIR}/lib/${RBDL_LIB_NAME}"
        )

        # Define include and library paths
        set(RBDL_INCLUDE_DIR "${RBDL_INSTALL_DIR}/include")
        if (${MATH_BACKEND} STREQUAL "CASADI")
            # Append "rbdl-casadi" to the include path
            set(RBDL_INCLUDE_DIR "${RBDL_INCLUDE_DIR}/rbdl-casadi")
        endif()

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

        set(RBDL_FOUND TRUE INTERNAL "RBDL found or built")
    endif()


endmacro()