if (${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.13")
    cmake_policy(SET CMP0078 NEW)
endif()
if (${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.14")
    cmake_policy(SET CMP0086 OLD)
endif()

option(BINDER_C "Build C module" OFF)
option(BINDER_PYTHON3 "Build Python SWIG module" OFF)
option(BINDER_MATLAB "Build Matlab module" OFF)

if (BIORBD_USE_CASADI_MATH)
    if(BINDER_C)
        message("Casadi option is not compatible with BINDER_C")
        set(BINDER_C OFF)
    endif(BINDER_C)
    if(BINDER_MATLAB)
        message("Casadi option is not compatible with BINDER_MATLAB")
        set(BINDER_MATLAB OFF)
    endif(BINDER_MATLAB)
endif(BIORBD_USE_CASADI_MATH)

# Common to all swig interface
if (BINDER_PYTHON3)
    # Set up swig
    find_package(SWIG REQUIRED)

    # Create the .i file
    SET(SWIG_MODULE_UTILS_INCLUDE_COMMAND
        "%include \"${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_utils.i\"")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}_utils.i.in"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_utils.i" @ONLY)

    SET(SWIG_MODULE_RIGIDBODY_INCLUDE_COMMAND
        "%include \"${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_rigidbody.i\"")
    if (NOT SKIP_KALMAN)
        SET(SWIG_KALMAN_INCLUDE_COMMAND
            "%include \"${CMAKE_SOURCE_DIR}/include/RigidBody/KalmanRecons.h\"\n
            %include \"${CMAKE_SOURCE_DIR}/include/RigidBody/KalmanReconsMarkers.h\"
            \n%include \"${CMAKE_SOURCE_DIR}/include/RigidBody/KalmanReconsIMU.h\""
        )
    endif()
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}_rigidbody.i.in"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_rigidbody.i" @ONLY)

    if (MODULE_MUSCLES)
        SET(SWIG_MODULE_MUSCLES_INCLUDE_COMMAND
            "%include \"${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_muscles.i\"")
        if (NOT SKIP_STATIC_OPTIM)
            SET(SWIG_STATIC_OPTIMIZATION_INCLUDE_COMMAND
                "%include \"${CMAKE_SOURCE_DIR}/include/Muscles/StaticOptimization.h\"")
        endif()
        configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}_muscles.i.in"
            "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_muscles.i" @ONLY)
    endif()

    if (MODULE_ACTUATORS)
        SET(SWIG_MODULE_ACTUATORS_INCLUDE_COMMAND
            "%include \"${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_actuators.i\"")
        configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}_actuators.i.in"
            "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_actuators.i" @ONLY)
    endif()

    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.i.in"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.i" @ONLY)

    include(${SWIG_USE_FILE})
endif()

# Add subdirectories for each language if desired
if (BINDER_C)
    add_subdirectory(c)
endif()

# Add subdirectories for each language if desired
if (BINDER_PYTHON3)
    add_subdirectory(python3)
endif()

if (BINDER_MATLAB)
    add_subdirectory(matlab)
endif()
