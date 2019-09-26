project(${MASTER_PROJECT_NAME}_tests_binding_c)
set(C_BINDER_TESTS_NAME ${PROJECT_NAME} PARENT_SCOPE)

file(GLOB TEST_SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_executable(${PROJECT_NAME} ${TEST_SRC_FILES})

# headers for the project
target_include_directories(${PROJECT_NAME} PUBLIC
    ${RBDL_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${DLIB_INCLUDE_DIR}
    ${IPOPT_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/binding/c
)

# Standard linking to gtest stuff.
target_link_libraries(${PROJECT_NAME} gtest_main)

# Extra linking for the project.
target_link_libraries(${PROJECT_NAME} ${MASTER_PROJECT_NAME}_c)
