function(FindOrBuildRBDL MATH_BACKEND)
    # -------------------------------
    # Validate backend
    # -------------------------------
    if(NOT (${MATH_BACKEND} STREQUAL "EIGEN" OR ${MATH_BACKEND} STREQUAL "CASADI"))
        message(FATAL_ERROR "FindOrBuildRBDL: Invalid option '${MATH_BACKEND}'. Use 'EIGEN' or 'CASADI'.")
    endif()

    # -------------------------------
    # Try finding existing install
    # -------------------------------
    if(NOT INSTALL_DEPENDENCIES_PREFIX OR INSTALL_DEPENDENCIES_PREFIX STREQUAL "")
        set(CUSTOM_RBDL_PATH "${CMAKE_BINARY_DIR}/RBDL_install")
    else()
        set(CUSTOM_RBDL_PATH ${INSTALL_DEPENDENCIES_PREFIX})
    endif()
    # The CUSTOM_RBDL_PATH is automatically used to force the path by find_package which may 
    # cause problem to find an installed version as it set RBDL_FOUND even though it is not. 
    # So we need to unset and retry finding if it prevented from actually finding the package
    set(CUSTOM_RBDL_PATH_COPY ${CUSTOM_RBDL_PATH})
    
    find_package(RBDL QUIET
        PATHS ${CUSTOM_RBDL_PATH} ${INSTALL_DEPENDENCIES_PREFIX} ${CMAKE_INSTALL_PREFIX}
    )
    if (NOT ${RBDL_INCLUDE_DIR})
        unset(CUSTOM_RBDL_PATH)
        find_package(RBDL QUIET
            PATHS ${CUSTOM_RBDL_PATH} ${INSTALL_DEPENDENCIES_PREFIX} ${CMAKE_INSTALL_PREFIX}
        )
        set(CUSTOM_RBDL_PATH ${CUSTOM_RBDL_PATH_COPY})
    endif()
    if(${MATH_BACKEND} STREQUAL "CASADI")
        find_package(RBDLCasadi QUIET
            PATHS ${CUSTOM_RBDL_PATH} ${INSTALL_DEPENDENCIES_PREFIX} ${CMAKE_INSTALL_PREFIX}
        )
        if (NOT ${RBDLCasadi_INCLUDE_DIR})
            unset(CUSTOM_RBDL_PATH)
            find_package(RBDLCasadi QUIET
                PATHS ${CUSTOM_RBDL_PATH} ${INSTALL_DEPENDENCIES_PREFIX} ${CMAKE_INSTALL_PREFIX}
            )
            # Sometimes it finds one include folder short, so check if "rbdl-casadi" subdirectory exists
            if(EXISTS "${RBDLCasadi_INCLUDE_DIR}/rbdl-casadi")
                # If it exists, override the include dir to point to it
                set(RBDLCasadi_INCLUDE_DIR "${RBDLCasadi_INCLUDE_DIR}/rbdl-casadi")
            endif()
            message(${RBDLCasadi_INCLUDE_DIR})
            set(CUSTOM_RBDL_PATH ${CUSTOM_RBDL_PATH_COPY})
        endif()
    endif()

    if(${MATH_BACKEND} STREQUAL "EIGEN")
        if(TARGET RBDL::RBDL)
            message(STATUS "Found RBDL (Eigen backend) with modern target")
            return()
        elseif(RBDL_LIBRARY)
            message(STATUS "Found RBDL (Eigen backend), wrapping legacy variables")
            add_library(RBDL::RBDL UNKNOWN IMPORTED)
            set_target_properties(RBDL::RBDL PROPERTIES
                IMPORTED_LOCATION "${RBDL_LIBRARY}"
                INTERFACE_INCLUDE_DIRECTORIES "${RBDL_INCLUDE_DIR}"
            )
            return()
        endif()

    elseif(${MATH_BACKEND} STREQUAL "CASADI")
        # We need to add "/casadi" as an extra include directory
        get_target_property(Casadi_INCLUDE_DIRS casadi::casadi INTERFACE_INCLUDE_DIRECTORIES)
        set_target_properties(casadi::casadi PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${Casadi_INCLUDE_DIRS}/casadi;${Casadi_INCLUDE_DIRS}"
        )

        if(TARGET RBDL::RBDL)
            message(STATUS "Found RBDL (CasADi backend) with modern target")
            return()
        elseif(RBDLCasadi_LIBRARY)
            message(STATUS "Found RBDL (CasADi backend), wrapping legacy variables")
            add_library(RBDL::RBDL UNKNOWN IMPORTED)
            set_target_properties(RBDL::RBDL PROPERTIES
                IMPORTED_LOCATION "${RBDLCasadi_LIBRARY}"
                INTERFACE_INCLUDE_DIRECTORIES "${RBDLCasadi_INCLUDE_DIR}"
            )
            return()
        endif()
    endif()

    # -------------------------------
    # If not found → build from source
    # -------------------------------
    message(STATUS "RBDL not found, using version master from GitHub")
    include(ExternalProject)

    # Backend config
    if(${MATH_BACKEND} STREQUAL "EIGEN")
        set(RBDL_BUILD_CASADI OFF)
        set(RBDL_LIBRARY_SUFFIX "")
        set(Casadi_DIR "")
        set(Casadi_INCLUDE_DIR "")
        set(Casadi_LIBRARY "")
    elseif (${MATH_BACKEND} STREQUAL "CASADI")
        set(RBDL_BUILD_CASADI ON)
        set(RBDL_LIBRARY_SUFFIX "-casadi")
        # Casadi_DIR is set from casadi::casadi
        # We need to add the Casadi_LIBRARY to the casadi::casadi target
        get_target_property(Casadi_INCLUDE_DIRS casadi::casadi INTERFACE_INCLUDE_DIRECTORIES)
        list(GET Casadi_INCLUDE_DIRS 0 Casadi_INCLUDE_DIR)
        
        get_target_property(Casadi_LIBRARY casadi::casadi IMPORTED_LOCATION_RELEASE)
    else()
		message(FATAL_ERROR "Backend not implemented")
    endif()

    # Detect correct static library extension (OS-independent)
    if(BUILD_SHARED_LIBS)
        set(RBDL_TARGET_STATIC OFF)
    else()
        set(RBDL_TARGET_STATIC ON)
    endif()

    if(WIN32)
        set(RBDL_LIB_NAME "rbdl${RBDL_LIBRARY_SUFFIX}.lib") 
    else()
        if(${RBDL_TARGET_STATIC})
            set(RBDL_LIB_NAME "librbdl${RBDL_LIBRARY_SUFFIX}.a")
        else()
            if(APPLE)
                set(RBDL_LIB_NAME "librbdl${RBDL_LIBRARY_SUFFIX}.dylib")
            elseif(UNIX)
                set(RBDL_LIB_NAME "librbdl${RBDL_LIBRARY_SUFFIX}.so")
            else()
                message(FATAL_ERROR "Unsupported OS")
            endif()
        endif()
    endif()

    set(RBDL_INSTALL_DIR "${CUSTOM_RBDL_PATH}")
    set(RBDL_LIBRARY "${RBDL_INSTALL_DIR}/lib/${RBDL_LIB_NAME}")
    set(RBDL_DEPENDS "")
    if(TARGET Eigen3_external)
        list(APPEND RBDL_DEPENDS Eigen3_external)
    endif()
    if(TARGET Casadi_external)
        list(APPEND RBDL_DEPENDS Casadi_external)
    endif()
    
    ExternalProject_Add(rbdl_external
        GIT_REPOSITORY https://github.com/pariterre/rbdl.git
        GIT_TAG master
        DEPENDS  ${RBDL_DEPENDS}
        CMAKE_ARGS
            -DCMAKE_INSTALL_PREFIX=${RBDL_INSTALL_DIR}
            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
            -DRBDL_BUILD_STATIC=${RBDL_TARGET_STATIC}
            -DRBDL_BUILD_CASADI=${RBDL_BUILD_CASADI}
            -DEigen3_DIR=${Eigen3_DIR}
            -DCasadi_DIR=${Casadi_DIR}
            -DCasadi_INCLUDE_DIR=${Casadi_INCLUDE_DIR}
            -DCasadi_LIBRARY=${Casadi_LIBRARY}
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        BUILD_BYPRODUCTS "${RBDL_LIBRARY}"
    )

    # Define include and library paths
    if(${MATH_BACKEND} STREQUAL "CASADI")
        set(RBDL_INCLUDE_DIR "${RBDL_INSTALL_DIR}/include/rbdl-casadi")
    else()
        set(RBDL_INCLUDE_DIR "${RBDL_INSTALL_DIR}/include")
    endif()

    # Define imported target
    add_library(RBDL::RBDL UNKNOWN IMPORTED)
    set_target_properties(RBDL::RBDL PROPERTIES
        IMPORTED_LOCATION "${RBDL_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${RBDL_INCLUDE_DIR}"
    )
    add_dependencies(RBDL::RBDL rbdl_external)
    
    # Ensure directory exists at configure time
    file(MAKE_DIRECTORY "${RBDL_INCLUDE_DIR}")
    
    # Windows DLL import handling
    if(WIN32)
        set_target_properties(RBDL::RBDL PROPERTIES
            IMPORTED_IMPLIB "${RBDL_LIBRARY}"
            IMPORTED_LOCATION "${RBDL_LIBRARY}"
        )
    endif()

    # Dependencies
    if(RBDL_DEPENDS)
        add_dependencies(rbdl_external ${RBDL_DEPENDS})
    endif()
    
    target_link_libraries(RBDL::RBDL INTERFACE Eigen3::Eigen)
    if(${MATH_BACKEND} STREQUAL "CASADI")
        target_link_libraries(RBDL::RBDL INTERFACE casadi::casadi)
    endif()
endfunction()
