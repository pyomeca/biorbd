project(biorbd)
cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 11)

# Set some variables
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/modules)

# Set a default build type to 'Release' if none was specified
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

# Compile biorbd
aux_source_directory(src SRC_LIST)
INCLUDE_DIRECTORIES(${RBDL_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${DLIB_INCLUDE_DIR} include)
add_library(${PROJECT_NAME} SHARED ${SRC_LIST})

# Add linker to biorbd
target_link_libraries(${PROJECT_NAME} ${RBDL_LIBRARY} ${DLIB_LIBRARY})

# Installation
if(WIN32)
	set(${PROJECT_NAME}_LIB_FOLDER Lib)
	set(${PROJECT_NAME}_BIN_FOLDER bin)
else()
	set(${PROJECT_NAME}_LIB_FOLDER ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})
	set(${PROJECT_NAME}_BIN_FOLDER ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})
endif()
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${${PROJECT_NAME}_LIB_FOLDER}
        RUNTIME DESTINATION ${${PROJECT_NAME}_BIN_FOLDER}
        LIBRARY DESTINATION ${${PROJECT_NAME}_LIB_FOLDER}
        INCLUDES DESTINATION include}
        )


# Add swig subdirectory
add_subdirectory(wrapper)









