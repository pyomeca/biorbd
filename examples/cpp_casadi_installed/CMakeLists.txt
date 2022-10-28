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

find_package(biorbd_casadi REQUIRED)
find_package(RBDLCasadi REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TinyXML REQUIRED)
find_package(Casadi REQUIRED)

add_executable(${PROJECT_NAME} ${FILE})

# Headers
if(WIN32)
    message(coucou)
    message(${RBDLCasadi_INCLUDE_DIR})
    message(coucou)
    set(RBDL_REAL_INCLUDE_DIR ${RBDLCasadi_INCLUDE_DIR}/rbdl-casadi)
else()
    set(RBDL_REAL_INCLUDE_DIR ${RBDLCasadi_INCLUDE_DIR})
endif()
target_include_directories(${PROJECT_NAME} PUBLIC 
    ${biorbd_casadi_INCLUDE_DIR}
    ${RBDL_REAL_INCLUDE_DIR}
    ${Casadi_INCLUDE_DIR}
    ${Casadi_INCLUDE_DIR}/..
	${TinyXML_INCLUDE_DIR}
)

find_package(IPOPT)
if (IPOPT_FOUND)
    target_include_directories(${PROJECT_NAME} PUBLIC 
        ${IPOPT_INCLUDE_DIR}
    )
endif()

# Linker and instalation
target_link_libraries(${PROJECT_NAME} 
    ${biorbd_casadi_LIBRARIES}
    ${RBDLCasadi_LIBRARY}
    ${Casadi_LIBRARY}
	${TinyXML_LIBRARY}
)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/pyomecaman.bioMod 
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

