project(biorbd)
cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 11)
set (${PROJECT_NAME}_VERSION_MAJOR 0)
set (${PROJECT_NAME}_VERSION_MINOR 1)

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

# Prepare linking with RBDL
find_package(RBDL REQUIRED)
set(BOOST_ROOT ${CMAKE_INSTALL_PREFIX})
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Dlib REQUIRED)

# Prepare add library
aux_source_directory(src SRC_LIST)
if (BUILD_SHARED_LIBS)
	add_library(${PROJECT_NAME} SHARED ${SRC_LIST})
else()
	add_library(${PROJECT_NAME} STATIC ${SRC_LIST})
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_debug")

# Add headers
target_include_directories(${PROJECT_NAME} PRIVATE ${RBDL_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${DLIB_INCLUDE_DIR} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add linker
target_link_libraries(${PROJECT_NAME} ${RBDL_LIBRARY} ${DLIB_LIBRARY})

# Installation
if(WIN32)
    set(${PROJECT_NAME}_LIB_FOLDER Lib)
    set(${PROJECT_NAME}_BIN_FOLDER bin)
    set(${PROJECT_NAME}_INCLUDE_FOLDER include/${PROJECT_NAME})
else()
    set(${PROJECT_NAME}_LIB_FOLDER ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})
    set(${PROJECT_NAME}_BIN_FOLDER ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})
    set(${PROJECT_NAME}_INCLUDE_FOLDER ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})
endif()
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${${PROJECT_NAME}_LIB_FOLDER}
        RUNTIME DESTINATION ${${PROJECT_NAME}_BIN_FOLDER}
        LIBRARY DESTINATION ${${PROJECT_NAME}_LIB_FOLDER}
        )
install(DIRECTORY include DESTINATION ${${PROJECT_NAME}_INCLUDE_FOLDER})
        
# Add binding subdirectory
add_subdirectory(binding)

# Add the example if asked
set(BUILD_EXAMPLE TRUE CACHE BOOL "Build a C++ example")
if (BUILD_EXAMPLE)
    add_subdirectory(example)
endif()







