
option(PYTHON3_WRAPPER "Build Python SWIG module" OFF)

# If at least one interface
if (PYTHON3_WRAPPER)
	# Set up swig
	find_package(SWIG REQUIRED)
	include(${SWIG_USE_FILE})
endif()

# Add subdirectories for each language if desired
if(PYTHON3_WRAPPER)
    add_subdirectory(python3)
endif()
