set(MASTER_PROJECT_NAME ${PROJECT_NAME})
project(${MASTER_PROJECT_NAME}_actuators)
set(ACTUATOR_MODULE_NAME ${MASTER_PROJECT_NAME}_actuators PARENT_SCOPE)

# Add the relevant files
set(SRC_LIST_MODULE
    ${CMAKE_CURRENT_SOURCE_DIR}/Actuator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorConstant.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorGauss3p.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorGauss6p.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorLinear.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Actuators.cpp
)

# Create the library
if (BUILD_SHARED_LIBS)
    add_library(${PROJECT_NAME} SHARED ${SRC_LIST_MODULE})
else()
    add_library(${PROJECT_NAME} STATIC ${SRC_LIST_MODULE})
endif()

# Add the dependencies for insuring build order
add_dependencies(${PROJECT_NAME} ${MASTER_PROJECT_NAME}_rigidbody ${MASTER_PROJECT_NAME}_utils)

# Add the include
target_include_directories(${PROJECT_NAME} PUBLIC
    ${RBDL_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${DLIB_INCLUDE_DIR}
    ${IPOPT_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}/include
)

# Installation
install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${${MASTER_PROJECT_NAME}_LIB_FOLDER}
    RUNTIME DESTINATION ${${MASTER_PROJECT_NAME}_BIN_FOLDER}
    LIBRARY DESTINATION ${${MASTER_PROJECT_NAME}_LIB_FOLDER}
)
