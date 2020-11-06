project(${BIORBD_NAME}_actuators)
set(ACTUATOR_MODULE_NAME ${BIORBD_NAME}_actuators PARENT_SCOPE)

# Add the relevant files
set(SRC_LIST_MODULE
    "${CMAKE_CURRENT_SOURCE_DIR}/Actuator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ActuatorConstant.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ActuatorGauss3p.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ActuatorGauss6p.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ActuatorLinear.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ActuatorSigmoidGauss3p.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Actuators.cpp"
)

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
    "${RBDL_INCLUDE_DIR}"
    "${RBDL_INCLUDE_DIR}/.."
    "${MATH_BACKEND_INCLUDE_DIR}"
    "${CMAKE_SOURCE_DIR}/include"
    "${BIORBD_BINARY_DIR}/include"
)

# Add the dependencies for insuring build order
target_link_libraries(${PROJECT_NAME}
    "${RBDL_LIBRARY}"
    "${MATH_BACKEND_LIBRARIES}"
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
    ARCHIVE DESTINATION "${${BIORBD_NAME}_LIB_FOLDER}"
    RUNTIME DESTINATION "${${BIORBD_NAME}_BIN_FOLDER}"
    LIBRARY DESTINATION "${${BIORBD_NAME}_LIB_FOLDER}"
)
set_target_properties(${PROJECT_NAME} PROPERTIES
    INSTALL_RPATH "${${BIORBD_NAME}_BIN_FOLDER}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)
