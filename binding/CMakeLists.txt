
option(BINDER_PYTHON3 "Build Python SWIG module" OFF)
option(BINDER_MATLAB "Build Matlab module" OFF)
 
# If at least one interface
if (BINDER_PYTHON3)
    # Set up swig
    find_package(SWIG REQUIRED)

    # Create the .i file
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
