cmake_minimum_required(VERSION 3.8)
set(FILE "forwardDynamicsExample.cpp")
get_filename_component(FILE_NAME ${FILE} NAME_WE)
project(${FILE_NAME})
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/modules")


find_package(biorbd REQUIRED)
find_package(RBDL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(IPOPT REQUIRED)

add_executable(${PROJECT_NAME} ${FILE})

# Headers
target_include_directories(${PROJECT_NAME} PUBLIC 
    ${biorbd_INCLUDE_DIR}
    ${RBDL_INCLUDE_DIR}
    ${RBDL_INCLUDE_DIR}/..
    ${EIGEN3_INCLUDE_DIR}
    ${IPOPT_INCLUDE_DIR}
)

# Linker and instalation
target_link_libraries(${PROJECT_NAME} 
    ${biorbd_LIBRARIES}
    ${RBDL_LIBRARY}
)

