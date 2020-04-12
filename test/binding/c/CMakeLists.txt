project(${BIORBD_NAME}_tests_binding_c)
set(C_BINDER_TESTS_NAME ${PROJECT_NAME} PARENT_SCOPE)

file(GLOB TEST_SRC_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_executable(${PROJECT_NAME} "${TEST_SRC_FILES}")

# headers for the project
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/binding/c
    "${CMAKE_SOURCE_DIR}/include"
    "${BIORBD_BINARY_DIR}/include"
    "${RBDL_INCLUDE_DIR}"
    "${RBDL_INCLUDE_DIR}/.."
    "${MATH_BACKEND_INCLUDE_DIR}"
)

# Standard linking to gtest stuff.
target_link_libraries(${PROJECT_NAME} "gtest_main")

# Extra linking for the project.
target_link_libraries(${PROJECT_NAME} "${BIORBD_NAME}_c")

# Copy the necessary file for the tests
file(GLOB BIORBD_TEST_FILES
    "${CMAKE_SOURCE_DIR}/test/models/*.bioMod")
file(COPY ${BIORBD_TEST_FILES}
     DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/models/")
