project(${BIORBD_NAME}_muscles)
set(MUSCLE_MODULE_NAME ${BIORBD_NAME}_muscles PARENT_SCOPE)

# Add the relevant files
set(SRC_LIST_MODULE
    "${CMAKE_CURRENT_SOURCE_DIR}/Characteristics.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Compound.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FatigueModel.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FatigueDynamicState.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FatigueDynamicStateXia.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FatigueParameters.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FatigueState.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Geometry.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/HillType.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/IdealizedActuator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/HillThelenType.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/HillThelenTypeFatigable.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Muscle.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/MuscleGroup.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Muscles.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/PathModifiers.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/State.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/StateDynamics.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/StateDynamicsBuchanan.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ViaPoint.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/WrappingCylinder.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/WrappingObject.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/WrappingSphere.cpp"
)

if (NOT SKIP_STATIC_OPTIM)
    list(APPEND SRC_LIST_MODULE
        "${CMAKE_CURRENT_SOURCE_DIR}/StaticOptimization.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/StaticOptimizationIpopt.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/StaticOptimizationIpoptLinearized.cpp"
    )
endif()

# Create the library
if (WIN32)
    add_library(${PROJECT_NAME} STATIC "${SRC_LIST_MODULE}")
else()
    if (BUILD_SHARED_LIBS)
        add_library(${PROJECT_NAME} SHARED "${SRC_LIST_MODULE}")
    else()
        add_library(${PROJECT_NAME} STATIC "${SRC_LIST_MODULE}")
    endif()
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_debug")

# Add the include
target_include_directories(${PROJECT_NAME} PRIVATE
    "${CMAKE_SOURCE_DIR}/include"
    "${BIORBD_BINARY_DIR}/include"
    "${RBDL_INCLUDE_DIR}"
    "${RBDL_INCLUDE_DIR}/.."
    "${MATH_BACKEND_INCLUDE_DIR}"
    "${IPOPT_INCLUDE_DIR}"
)

# Add the dependencies for insuring build order
target_link_libraries(${PROJECT_NAME}
    "${RBDL_LIBRARY}"
    "${MATH_BACKEND_LIBRARIES}"
    "${IPOPT_LIBRARY}"
    "${BIORBD_NAME}_utils"
    "${BIORBD_NAME}_rigidbody"
)
add_dependencies(${PROJECT_NAME}
    "${BIORBD_NAME}_utils"
    "${BIORBD_NAME}_rigidbody"
)

# Installation
install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${${BIORBD_NAME}_LIB_FOLDER}
    RUNTIME DESTINATION ${${BIORBD_NAME}_BIN_FOLDER}
    LIBRARY DESTINATION ${${BIORBD_NAME}_LIB_FOLDER}
)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH ${${BIORBD_NAME}_BIN_FOLDER})
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
