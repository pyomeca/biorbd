project(${BIORBD_NAME}_tests)

# Set a define to skip or not long tests
option(SKIP_LONG_TESTS
    "Skip some longer tests, mostly for debug purposes" FALSE)

# Download gtest if necessary
set(RELATIVE_PATH_TO_GTEST "external/googletest")
find_path(GTEST_INCLUDE_DIR "gtest.h" HINTS
    "${CMAKE_SOURCE_DIR}/${RELATIVE_PATH_TO_GTEST}/googletest/include/gtest"
)

if((NOT GTEST_INCLUDE_DIR) OR (NOT EXISTS "${GTEST_INCLUDE_DIR}"))
    # we couldn't find the header files for gtest or they don't exist
    message("Unable to find gtest")

    # we have a submodule setup for gtest, assume it is under external/gtest
    # now we need to clone this submodule
    execute_process(COMMAND "git submodule update --init -- external/googletest"
                    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

    find_path(GTEST_INCLUDE_DIR "gtest.h" HINTS
        "${CMAKE_SOURCE_DIR}/${RELATIVE_PATH_TO_GTEST}/googletest/include/gtest"
    )
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
set(BUILD_GMOCK OFF)
set(INSTALL_GTEST OFF)
add_subdirectory(
    "${CMAKE_SOURCE_DIR}/${RELATIVE_PATH_TO_GTEST}"
    "${CMAKE_BINARY_DIR}/gtest"
    EXCLUDE_FROM_ALL
)

##############
# Unit Tests
##############
enable_testing()

set(TEST_SRC_FILES
    "${CMAKE_SOURCE_DIR}/test/test_biorbd.cpp"
    "${CMAKE_SOURCE_DIR}/test/test_rigidbody.cpp"
    "${CMAKE_SOURCE_DIR}/test/test_utils.cpp"
    "${CMAKE_SOURCE_DIR}/test/test_actuators.cpp"
)
if(MODULE_MUSCLES)
    list(APPEND TEST_SRC_FILES "${CMAKE_SOURCE_DIR}/test/test_muscles.cpp")
endif()
if(MODULE_ACTUATORS)
    list(APPEND TEST_SRC_FILES "${CMAKE_SOURCE_DIR}/test/test_actuators.cpp")
endif()
add_executable(${PROJECT_NAME} "${TEST_SRC_FILES}")
add_dependencies(${PROJECT_NAME} ${BIORBD_NAME})

# headers for the project
target_include_directories(${PROJECT_NAME} PRIVATE
    "${CMAKE_SOURCE_DIR}/include"
    "${BIORBD_BINARY_DIR}/include"
    "${RBDL_INCLUDE_DIR}"
    "${RBDL_INCLUDE_DIR}/.."
    "${MATH_BACKEND_INCLUDE_DIR}"
    "${IPOPT_INCLUDE_DIR}"
)

# Standard linking to gtest stuff.
target_link_libraries(${PROJECT_NAME}
    "gtest_main")

# Extra linking for the project.
target_link_libraries(${PROJECT_NAME}
    "${BIORBD_NAME}")

if (CMAKE_BUILD_TYPE STREQUAL "Coverage")
    set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.travis/cmake")

    include(CodeCoverage)
    setup_target_for_coverage(${BIORBD_NAME}_coverage
        "${PROJECT_NAME}" "coverage")

    SET(CMAKE_CXX_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")
    SET(CMAKE_C_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")
endif() #CMAKE_BUILD_TYPE STREQUAL "Coverage"

# Copy the necessary file for the tests
file(GLOB BIORBD_TEST_FILES
    "${CMAKE_SOURCE_DIR}/test/models/*.bioMod")
file(COPY ${BIORBD_TEST_FILES}
     DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/models/")
file(GLOB BIORBD_TEST_FILES
    "${CMAKE_SOURCE_DIR}/test/models/meshFiles/*.bioMesh")
file(COPY ${BIORBD_TEST_FILES}
  DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/models/meshFiles/")
file(GLOB BIORBD_TEST_FILES
    "${CMAKE_SOURCE_DIR}/test/models/meshFiles/vtp/*.vtp")
file(COPY ${BIORBD_TEST_FILES}
  DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/models/meshFiles/vtp/")
file(GLOB BIORBD_TEST_FILES
    "${CMAKE_SOURCE_DIR}/test/models/meshFiles/*.obj")
file(COPY ${BIORBD_TEST_FILES}
  DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/models/meshFiles/")

set(ALL_TESTS "${PROJECT_NAME}")
if (BINDER_C)
    add_subdirectory("binding/c")
    list(APPEND ALL_TESTS "${ALL_TESTS}" "${C_BINDER_TESTS_NAME}")
endif()

if (BINDER_PYTHON3)
    file(GLOB BIORBD_PYTHON3_TEST_FILES
        "${CMAKE_SOURCE_DIR}/test/binding/Python3/*.py")
    file(COPY ${BIORBD_PYTHON3_TEST_FILES}
        DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/binding/Python3")
endif()

# This is so you can do 'make test' to see all your tests run, instead of
# manually running the executable runUnitTests to see those specific tests.
add_test(UnitTests "${ALL_TESTS}")
