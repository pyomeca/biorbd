set(MASTER_PROJECT_NAME ${PROJECT_NAME})
project(${MASTER_PROJECT_NAME}_muscles)
set(MUSCLE_MODULE_NAME ${MASTER_PROJECT_NAME}_muscles PARENT_SCOPE)

# Add the relevant files
set(SRC_LIST_MODULE
    ${CMAKE_CURRENT_SOURCE_DIR}/Caracteristics.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Compound.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Fatigable.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/FatigueDynamicState.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/FatigueDynamicStateXia.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/FatigueParameters.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/FatigueState.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Force.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ForceFromInsertion.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ForceFromOrigin.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/HillType.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/IdealizedActuator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/HillTypeThelen.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/HillTypeThelenFatigable.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Muscle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MuscleGroup.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Muscles.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/PathChangers.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/State.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/StateDynamics.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/StateDynamicsBuchanan.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ViaPoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/WrappingCylinder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/WrappingObject.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/WrappingSphere.cpp
)

if (IPOPT_FOUND)
    list(APPEND SRC_LIST_MODULE
        ${CMAKE_CURRENT_SOURCE_DIR}/StaticOptimization.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/StaticOptimizationIpopt.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/StaticOptimizationIpoptLinearized.cpp
    )
endif()

# Create the library
if (BUILD_SHARED_LIBS)
    add_library(${PROJECT_NAME} SHARED ${SRC_LIST_MODULE})
else()
    add_library(${PROJECT_NAME} STATIC ${SRC_LIST_MODULE})
endif()

# Add the dependencies for insuring build order
target_link_libraries(${PROJECT_NAME} ${RBDL_LIBRARY} ${IPOPT_LIBRARY} ${MASTER_PROJECT_NAME}_utils ${MASTER_PROJECT_NAME}_rigidbody)
add_dependencies(${PROJECT_NAME} ${MASTER_PROJECT_NAME}_utils ${MASTER_PROJECT_NAME}_rigidbody)

# Add the include
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

# Installation
install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${${MASTER_PROJECT_NAME}_LIB_FOLDER}
    RUNTIME DESTINATION ${${MASTER_PROJECT_NAME}_BIN_FOLDER}
    LIBRARY DESTINATION ${${MASTER_PROJECT_NAME}_LIB_FOLDER}
)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH ${${MASTER_PROJECT_NAME}_BIN_FOLDER})
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
