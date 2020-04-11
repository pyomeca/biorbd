# Include python
find_package(PYTHON3 REQUIRED)
include_directories("${Python3_INCLUDE_DIRS}")

find_package(Numpy REQUIRED)

include_directories(${NUMPY_INCLUDES})
if(WIN32)
    link_directories("${Python3_RUNTIME_LIBRARY_DIRS}/libs")
endif()

# Create the .i file
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}_python.i.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_python.i" @ONLY)

# Add swig module
set(CMAKE_SWIG_FLAGS "")
set(I_SOURCE_FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_python.i")

set_property(SOURCE "${I_SOURCE_FILE}" PROPERTY CPLUSPLUS ON)
SWIG_ADD_LIBRARY(${PROJECT_NAME}_python
    TYPE MODULE
    LANGUAGE "python"
    SOURCES "${I_SOURCE_FILE}"
)
swig_link_libraries(${PROJECT_NAME}_python
    "${PYTHON_LIBRARIES}"
    "${NUMPY_LIBRARIES}"
)

# Add headers
target_include_directories(${PROJECT_NAME}_python PRIVATE
    "${CMAKE_SOURCE_DIR}/include"
    "${BIORBD_BINARY_DIR}/include"
    "${RBDL_INCLUDE_DIR}"
    "${RBDL_INCLUDE_DIR}/.."
    "${MATH_BACKEND_INCLUDE_DIR}"
    "${IPOPT_INCLUDE_DIR}"
    "${PYTHON_INCLUDE_DIRS}"
    "${NUMPY_INCLUDES}"
)

if(APPLE)
    set_target_properties(${PROJECT_NAME}_python
        PROPERTIES SUFFIX ".so" INSTALL_RPATH ${${PROJECT_NAME}_BIN_FOLDER})
    set_property(TARGET ${PROJECT_NAME}_python APPEND PROPERTY
        LINK_FLAGS "-flat_namespace -undefined suppress")
endif()
set_target_properties(${PROJECT_NAME}_python PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${${PROJECT_NAME}_BIN_FOLDER}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)



# Install to the proper directories
INSTALL(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.py"
    DESTINATION "${Python3_SITELIB}/${PROJECT_NAME}")

INSTALL(FILES "${CMAKE_CURRENT_SOURCE_DIR}/__init__.py"
    DESTINATION "${Python3_SITELIB}/${PROJECT_NAME}")

# Create the version file
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/_version.py.i"
    "${CMAKE_CURRENT_BINARY_DIR}/_version.py")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/_version.py"
    DESTINATION "${Python3_SITELIB}/${PROJECT_NAME}")

# Check if swig has its version 3 or 4, to choose the right place to put the library
if(${SWIG_VERSION} MATCHES "^3(\.[0-9]*)*$")
    set(PYTHON_INSTALL_DESTINATION "${Python3_SITELIB}")
elseif(${SWIG_VERSION} MATCHES "^4(\.[0-9]*)*$")
    set(PYTHON_INSTALL_DESTINATION "${Python3_SITELIB}/${PROJECT_NAME}")
else()
    message( FATAL_ERROR "Swig must be version 3 or 4" )
endif()

# Install the library
INSTALL(TARGETS ${PROJECT_NAME}_python
    DESTINATION "${PYTHON_INSTALL_DESTINATION}")
