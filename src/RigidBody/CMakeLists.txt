project(${BIORBD_NAME}_rigidbody)
set(RIGIDBODY_MODULE_NAME ${BIORBD_NAME}_rigidbody PARENT_SCOPE)

# Add the relevant files
set(SRC_LIST_MODULE
    "${CMAKE_CURRENT_SOURCE_DIR}/Segment.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SegmentCharacteristics.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Mesh.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Contacts.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/GeneralizedCoordinates.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/GeneralizedVelocity.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/GeneralizedAcceleration.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/GeneralizedTorque.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/IMU.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/IMUs.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Joints.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Markers.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/NodeSegment.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/RotoTransNodes.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/MeshFace.cpp"
)
if (BIORBD_USE_CASADI_MATH AND MODULE_KALMAN)
    message(WARNING "Casadi and Kalman cannot be used alongside, MODULE_KALMAN is automatically set to OFF")
    set(MODULE_KALMAN OFF CACHE BOOL "" FORCE)
endif()
if (MODULE_KALMAN)
    list(APPEND SRC_LIST_MODULE
        "${CMAKE_CURRENT_SOURCE_DIR}/KalmanRecons.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/KalmanReconsIMU.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/KalmanReconsMarkers.cpp"
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
)

# Add the dependencies for insuring build order
target_link_libraries(${PROJECT_NAME}
    "${RBDL_LIBRARY}"
    "${MATH_BACKEND_LIBRARIES}"
    "${BIORBD_NAME}_utils"
)
add_dependencies(${PROJECT_NAME} "${BIORBD_NAME}_utils")

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
