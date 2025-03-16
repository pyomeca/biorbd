macro(FinddRBDLWithBackend MATH_BACKEND)
    # Check if MATH_BACKEND is EIGEN or CASADI
    if(NOT (${MATH_BACKEND} STREQUAL "EIGEN" OR ${MATH_BACKEND} STREQUAL "CASADI"))
        message(FATAL_ERROR "FindOrBuildRBDL: Invalid option '${MATH_BACKEND}'. Use 'EIGEN' or 'CASADI'.")
    endif()

    # Try to find RBDL first
    if (${MATH_BACKEND} STREQUAL "EIGEN")
        set (CUSTOM_RBDL_PATH ${INSTALL_DEPENDENCIES_PREFIX})
        find_package(RBDL QUIET)
    
    elseif (${MATH_BACKEND} STREQUAL "CASADI")
        set(RBDL_FOUND FALSE)
        set (CUSTOM_RBDLCasadi_PATH ${INSTALL_DEPENDENCIES_PREFIX})
        find_package(RBDLCasadi QUIET)

        if (RBDLCasadi_FOUND)
            set(RBDL_FOUND TRUE)
            
            # Define include and library paths to mimic RBDL eigen which is the format expected by biorbd
            set(RBDL_INCLUDE_DIR ${RBDLCasadi_INCLUDE_DIR})
            set(RBDL_LIBRARY ${RBDLCasadi_LIBRARY})
        endif()
    endif()

    if(RBDL_FOUND)
        message (STATUS "RBDL found")

    else()
        message(FATAL "RBDL not found")
        
    endif()

endmacro()