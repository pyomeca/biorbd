cmake_minimum_required(VERSION 3.8)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/modules")

set(FILE "forwardDynamicsExample.cpp")
get_filename_component(FILE_NAME ${FILE} NAME_WE)
project(${FILE_NAME})

# Set a default build type to 'Release' if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build.")
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
        STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo" "Coverage")
endif()

find_package(biorbd_eigen REQUIRED)
include(FindRBDLWithBackend)
FindRBDLWithBackend(EIGEN)
find_package(Eigen3 REQUIRED)
find_package(TinyXML2 REQUIRED)

add_executable(${PROJECT_NAME} ${FILE})

# Headers
target_include_directories(${PROJECT_NAME} PUBLIC 
    ${RBDL_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${biorbd_eigen_INCLUDE_DIR}
)
find_package(IPOPT)
if (IPOPT_FOUND)
    target_include_directories(${PROJECT_NAME} PUBLIC 
        ${IPOPT_INCLUDE_DIR}
    )
endif()

# Linker and instalation
target_link_libraries(${PROJECT_NAME} 
    ${biorbd_eigen_LIBRARIES}
    ${RBDL_LIBRARY}
    tinyxml2::tinyxml2
)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/pyomecaman.bioMod 
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

