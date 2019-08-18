
option(BINDER_PYTHON3 "Build Python SWIG module" OFF)
option(BINDER_MATLAB "Build Matlab module" OFF)
 
# Common to all swig interface
if (BINDER_PYTHON3)
    # Set up swig
    find_package(SWIG REQUIRED)

    # Create the .i file
    SET(SWIG_MODULE_UTILS_INCLUDE_COMMAND "%include \"${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_utils.i\"")
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}_utils.i.in ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_utils.i @ONLY)

    SET(SWIG_MODULE_RIGIDBODY_INCLUDE_COMMAND "%include \"${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_rigidbody.i\"")
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}_rigidbody.i.in ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_rigidbody.i @ONLY)

    if (MODULE_MUSCLES)
        SET(SWIG_MODULE_MUSCLES_INCLUDE_COMMAND "%include \"${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_muscles.i\"")
        if (IPOPT_FOUND)
            SET(SWIG_STATIC_OPTIMIZATION_INCLUDE_COMMAND "%include \"${CMAKE_SOURCE_DIR}/include/Muscles/StaticOptimization.h\"")
        endif()
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}_muscles.i.in ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_muscles.i @ONLY)
    endif()

    if (MODULE_ACTUATORS)
        SET(SWIG_MODULE_ACTUATORS_INCLUDE_COMMAND "%import \"${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_actuators.i\"")
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}_actuators.i.in ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_actuators.i @ONLY)
    endif()

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.i.in ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.i @ONLY)

    include(${SWIG_USE_FILE})
endif()

# Add subdirectories for each language if desired
if (BINDER_PYTHON3)
    add_subdirectory(python3)
endif()

if (BINDER_MATLAB)
    add_subdirectory(matlab)
endif()
