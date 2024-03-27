project(${BIORBD_NAME}_utils)
set(UTILS_MODULE_NAME ${BIORBD_NAME}_utils PARENT_SCOPE)

# Add the relevant files
set(SRC_LIST_MODULE
    "${CMAKE_CURRENT_SOURCE_DIR}/RotoTrans.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Benchmark.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Equation.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Error.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/IfStream.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Path.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Matrix.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Matrix2d.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Matrix3d.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Node.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Scalar.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Vector2d.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Vector3d.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SpatialTransform.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SpatialVector.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Range.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Rotation.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/RotoTransNode.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Quaternion.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/String.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Timer.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Vector.cpp"
)

if(${MATH_LIBRARY_BACKEND} STREQUAL "Casadi")
    list(APPEND SRC_LIST_MODULE "${CMAKE_CURRENT_SOURCE_DIR}/CasadiExpand.cpp")
    option(USE_SMOOTH_IF_ELSE "If biorbd should be compiled with branching if_else (from CasADi) or using the tanh approximation" OFF)
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
    "${MATH_BACKEND_INCLUDE_DIR}"
)

target_link_libraries(${PROJECT_NAME}
    "${RBDL_LIBRARY}"
    "${MATH_BACKEND_LIBRARIES}"
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
