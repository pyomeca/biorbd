project(${BIORBD_NAME}_example)
add_executable(${PROJECT_NAME} "main.cpp")
add_dependencies(${PROJECT_NAME} ${BIORBD_NAME})

# Headers
target_include_directories(${PROJECT_NAME} PRIVATE
    "${CMAKE_SOURCE_DIR}/include"
    "${BIORBD_BINARY_DIR}/include"
    "${RBDL_INCLUDE_DIR}"
    "${RBDL_INCLUDE_DIR}/.."
    "${MATH_BACKEND_INCLUDE_DIR}"
    "${IPOPT_INCLUDE_DIR}"
)

# Libraries linker
get_target_property(DEBUG_POSTFIX ${BIORBD_NAME} DEBUG_POSTFIX)
IF(CMAKE_BUILD_TYPE MATCHES "Debug")
    SET(${BIORBD_NAME}_LIB_NAME ${BIORBD_NAME}${DEBUG_POSTFIX})
else()
    SET(${BIORBD_NAME}_LIB_NAME ${BIORBD_NAME})
endif()
if (APPLE)
    string(CONCAT ${BIORBD_NAME}_LIB_NAME "lib" ${${BIORBD_NAME}_LIB_NAME} ".dylib")
elseif (UNIX)
    string(CONCAT ${BIORBD_NAME}_LIB_NAME "lib" ${${BIORBD_NAME}_LIB_NAME} ".so")
elseif(_WIN32)
    string(CONCAT ${BIORBD_NAME}_LIB_NAME ${${BIORBD_NAME}_LIB_NAME} ".dll")
endif()
set(${BIORBD_NAME}_LIBRARY ${CMAKE_BINARY_DIR}/${${BIORBD_NAME}_LIB_NAME})
target_link_libraries(${PROJECT_NAME}
    ${BIORBD_NAME}
)

# Copy the c3d of the example
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/pyomecaman.bioMod
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
