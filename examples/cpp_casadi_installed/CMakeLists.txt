cmake_minimum_required(VERSION 3.8)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

set(FILE "forwardDynamicsExample.cpp")
get_filename_component(FILE_NAME ${FILE} NAME_WE)
project(${FILE_NAME})


find_package(biorbd_casadi REQUIRED)
find_package(RBDLCasadi REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TinyXML REQUIRED)
find_package(Casadi REQUIRED)

add_executable(${PROJECT_NAME} ${FILE})

# Headers
target_include_directories(${PROJECT_NAME} PUBLIC 
    ${biorbd_casadi_INCLUDE_DIR}
    ${RBDLCasadi_INCLUDE_DIR}
    ${RBDLCasadi_INCLUDE_DIR}/..
    ${Casadi_INCLUDE_DIR}
    ${Casadi_INCLUDE_DIR}/..
	${TinyXML_INCLUDE_DIR}
)
if(NOT WIN32)
    find_package(IPOPT REQUIRED)
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

