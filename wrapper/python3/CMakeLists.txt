project(pyorbdl)
cmake_minimum_required(VERSION 2.8)

# Find Python 3
find_package(PYTHON3 REQUIRED)
set(PYTHON3_WRAPPER_PATH ${CMAKE_CURRENT_SOURCE_DIR}/wrapper/python3)

# Get the file to compile
set(CPP_FILE ${PYTHON3_WRAPPER_PATH}/pyorbdl.cpp)

# Compile S2M_rbdl
include_directories(${Python3_INCLUDE_DIRS} ${Python3_SITELIB}/numpy/core/include ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(${PROJECT_NAME} SHARED ${CPP_FILE})

if(WIN32)
else(WIN32)
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES PREFIX "")
endif(WIN32)


target_link_libraries(${PROJECT_NAME} S2M_rbdl)

