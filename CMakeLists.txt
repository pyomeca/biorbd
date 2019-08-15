cmake_minimum_required(VERSION 3.8)
project(biorbd VERSION 0.1.3)
set (CMAKE_CXX_STANDARD 11)

# Set some variables
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/modules)

# Set a default build type to 'Release' if none was specified
if (WIN32)
    SET(BUILD_SHARED_LIBS FALSE CACHE BOOL "Choose if the static library should be build" FORCE)
else()
    SET(BUILD_SHARED_LIBS TRUE CACHE BOOL "Choose if the static library should be build" FORCE)
endif()
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	MESSAGE(STATUS "Setting build type to 'Release' as none was specified.")
	SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

# Prepare linkings
find_package(RBDL REQUIRED)
set(BOOST_ROOT ${CMAKE_INSTALL_PREFIX})
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Dlib REQUIRED)
find_package(IPOPT REQUIRED)

# Prepare add library
set(SRC_LIST
    src/BiorbdModel.cpp
    src/ModelReader.cpp
    src/ModelWriter.cpp
)
if (BUILD_SHARED_LIBS)
    add_library(${PROJECT_NAME} SHARED ${SRC_LIST})
else()
    add_library(${PROJECT_NAME} STATIC ${SRC_LIST})
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_debug")

# Create the configuration header file associated with the asked options
configure_file(${CMAKE_SOURCE_DIR}/include/biorbdConfig.h.in ${CMAKE_BINARY_DIR}/include/biorbdConfig.h)

# Add headers
target_include_directories(${PROJECT_NAME} PUBLIC
    ${RBDL_INCLUDE_DIR}
    ${RBDL_INCLUDE_DIR}/..
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${DLIB_INCLUDE_DIR}
    ${IPOPT_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}/include
)

# Installation directories
if(WIN32)
    set(${PROJECT_NAME}_LIB_FOLDER Lib)
    set(${PROJECT_NAME}_BIN_FOLDER bin)
    set(${PROJECT_NAME}_INCLUDE_FOLDER include/${PROJECT_NAME})
else()
    set(${PROJECT_NAME}_LIB_FOLDER ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})
    set(${PROJECT_NAME}_BIN_FOLDER ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})
    set(${PROJECT_NAME}_INCLUDE_FOLDER ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH ${${PROJECT_NAME}_BIN_FOLDER})
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)


# Add the different modules of biorbd
set(BIORBD_MODULE_NAMES)

# Utils
add_subdirectory(src/Utils)
list(APPEND BIORBD_MODULE_NAMES ${UTILS_MODULE_NAME})

# Utils
add_subdirectory(src/RigidBody)
list(APPEND BIORBD_MODULE_NAMES ${RIGIDBODY_MODULE_NAME})

# Actuators
option(MODULE_ACTUATORS "Choose if the module actuators is to be added to biorbd" ON)
if (MODULE_ACTUATORS)
    add_subdirectory(src/Actuators)
    list(APPEND BIORBD_MODULE_NAMES ${ACTUATOR_MODULE_NAME})
endif()

# Muscles
option(MODULE_MUSCLES "Choose if the module actuators is to be added to biorbd" ON)
if (MODULE_MUSCLES)
    add_subdirectory(src/Muscles)
    list(APPEND BIORBD_MODULE_NAMES ${MUSCLE_MODULE_NAME})
endif()

# Add linker
target_link_libraries(${PROJECT_NAME} ${BIORBD_MODULE_NAMES} ${RBDL_LIBRARY} ${DLIB_LIBRARY} ${IPOPT_LIBRARY})

# installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${${PROJECT_NAME}_LIB_FOLDER}
        RUNTIME DESTINATION ${${PROJECT_NAME}_BIN_FOLDER}
        LIBRARY DESTINATION ${${PROJECT_NAME}_LIB_FOLDER}
        )
install(DIRECTORY include DESTINATION ${${PROJECT_NAME}_INCLUDE_FOLDER})
        
# Add binding subdirectory
add_subdirectory(binding)

# Add the example if asked
option(BUILD_EXAMPLE "Build a C++ example" ON)
if (BUILD_EXAMPLE)
    add_subdirectory(example)
endif()

# Testing
option(BUILD_TESTS "Build all tests." OFF) # Makes boolean 'test' available.
if (BUILD_TESTS)
    add_subdirectory(test)
endif()





