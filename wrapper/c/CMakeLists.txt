project(S2M_rbdl_c)
cmake_minimum_required(VERSION 2.8)

set(C_WRAPPER_PATH ${CMAKE_CURRENT_SOURCE_DIR}/wrapper/c)

# Get the file to compile
set(C_FILE ${C_WRAPPER_PATH}/c_wrapper.cpp)

# Compile S2M_rbdl
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(${PROJECT_NAME} SHARED ${C_FILE})

target_link_libraries(${PROJECT_NAME} S2M_rbdl)







