project(${BIORBD_NAME}_muscles)
set(MUSCLES_MODULE_NAME ${BIORBD_NAME}_muscles PARENT_SCOPE)

# Add the relevant files
set(SRC_LIST_MODULE
    "${CMAKE_CURRENT_SOURCE_DIR}/Characteristics.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/MuscleGeometry.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FatigueModel.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FatigueDynamicState.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FatigueDynamicStateXia.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FatigueParameters.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FatigueState.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/HillType.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/IdealizedActuator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/HillThelenType.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/HillDeGrooteType.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/HillThelenActiveOnlyType.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/HillThelenTypeFatigable.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/HillDeGrooteActiveOnlyType.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/HillDeGrooteTypeFatigable.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Muscle.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/MuscleGroup.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Muscles.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/State.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/StateDynamics.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/StateDynamicsDeGroote.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/StateDynamicsBuchanan.cpp"
)

if (MODULE_STATIC_OPTIM)
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
    "${MATH_BACKEND_INCLUDE_DIR}"
    "${IPOPT_INCLUDE_DIR}"
    "${CMAKE_SOURCE_DIR}/include"
    "${BIORBD_BINARY_DIR}/include"
    "${INSTALL_DEPENDENCIES_PREFIX}/include"
)

# Add the dependencies for insuring build order
target_link_libraries(${PROJECT_NAME}
    RBDL::RBDL
    "${IPOPT_LIBRARY}"
    "${BIORBD_NAME}_utils"
    "${BIORBD_NAME}_rigidbody"
    "${BIORBD_NAME}_internal_forces"
)
add_dependencies(${PROJECT_NAME}
    "${BIORBD_NAME}_utils"
    "${BIORBD_NAME}_rigidbody"
    "${BIORBD_NAME}_internal_forces"
)

# Installation
install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION "${${BIORBD_NAME}_LIB_FOLDER}"
    RUNTIME DESTINATION "${${BIORBD_NAME}_BIN_FOLDER}"
    LIBRARY DESTINATION "${${BIORBD_NAME}_LIB_FOLDER}"
)
set_target_properties(${PROJECT_NAME} PROPERTIES
    INSTALL_RPATH ${${BIORBD_NAME}_BIN_FOLDER}
    INSTALL_RPATH_USE_LINK_PATH TRUE
)
