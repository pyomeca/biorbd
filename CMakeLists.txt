project(biorbd)
cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 11)
set (${PROJECT_NAME}_VERSION_MAJOR 0)
set (${PROJECT_NAME}_VERSION_MINOR 1)

# Set some variables
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/modules)

# Set a default build type to 'Release' if none was specified
if (WIN32)
    SET(BUILD_SHARED_LIBS FALSE CACHE BOOL "Choose if the static library should be build" FORCE)
else()
    SET(BUILD_SHARED_LIBS TRUE CACHE BOOL "Choose if the static library should be build" FORCE)
endif()
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	MESSAGE(STATUS "Setting build type to 'Release' as none was specified.")
	SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

# Prepare linking with RBDL
find_package(RBDL REQUIRED)
set(BOOST_ROOT ${CMAKE_INSTALL_PREFIX})
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(Dlib REQUIRED)

# Prepare add library
aux_source_directory(src SRC_LIST)
if (BUILD_SHARED_LIBS)
    add_library(${PROJECT_NAME} SHARED ${SRC_LIST})
else()
    add_library(${PROJECT_NAME} STATIC ${SRC_LIST})
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_debug")

# Add headers
target_include_directories(${PROJECT_NAME} PUBLIC
    ${RBDL_INCLUDE_DIR}
    ${RBDL_INCLUDE_DIR}/..
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    # ${DLIB_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add linker
target_link_libraries(${PROJECT_NAME} ${RBDL_LIBRARY} ${DLIB_LIBRARY})

# Installation
if(WIN32)
    set(${PROJECT_NAME}_LIB_FOLDER Lib)
    set(${PROJECT_NAME}_BIN_FOLDER bin)
    set(${PROJECT_NAME}_INCLUDE_FOLDER include/${PROJECT_NAME})
else()
    set(${PROJECT_NAME}_LIB_FOLDER ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})
    set(${PROJECT_NAME}_BIN_FOLDER ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})
    set(${PROJECT_NAME}_INCLUDE_FOLDER ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})
endif()
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${${PROJECT_NAME}_LIB_FOLDER}
        RUNTIME DESTINATION ${${PROJECT_NAME}_BIN_FOLDER}
        LIBRARY DESTINATION ${${PROJECT_NAME}_LIB_FOLDER}
        )
install(DIRECTORY include DESTINATION ${${PROJECT_NAME}_INCLUDE_FOLDER})
        
# Add binding subdirectory
add_subdirectory(binding)

# Add the example if asked
set(BUILD_EXAMPLE TRUE CACHE BOOL "Build a C++ example")
if (BUILD_EXAMPLE)
    add_subdirectory(example)
endif()



################################
# Testing
################################

# Options. Turn on with 'cmake -DBUILD_TESTS=ON'.
option(BUILD_TESTS "Build all tests." OFF) # Makes boolean 'test' available.

if (BUILD_TESTS)
    # Download gtest if necessary
    set(RELATIVE_PATH_TO_GTEST external/googletest)
    find_path(GTEST_INCLUDE_DIR gtest.h HINTS ${CMAKE_CURRENT_SOURCE_DIR}/${RELATIVE_PATH_TO_GTEST}/googletest/include/gtest)

    if((NOT GTEST_INCLUDE_DIR) OR (NOT EXISTS ${GTEST_INCLUDE_DIR}))
        # we couldn't find the header files for gtest or they don't exist
        message("Unable to find gtest")

        # we have a submodule setup for gtest, assume it is under external/gtest
        # now we need to clone this submodule
        execute_process(COMMAND git submodule update --init -- external/googletest
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

        find_path(GTEST_INCLUDE_DIR gtest.h HINTS ${CMAKE_CURRENT_SOURCE_DIR}/${RELATIVE_PATH_TO_GTEST}/googletest/include/gtest)
    endif()

    # Prevent overriding the parent project's compiler/linker
    # settings on Windows
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    # Add googletest directly to our build. This defines
    # the gtest and gtest_main targets.
    set(BUILD_GMOCK OFF)
    set(INSTALL_GTEST OFF)
    add_subdirectory(${RELATIVE_PATH_TO_GTEST}
                     ${CMAKE_BINARY_DIR}/gtest
                     EXCLUDE_FROM_ALL)

    ##############
    # Unit Tests
    ##############
    enable_testing()
    file(GLOB TEST_SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)
    add_executable(runUnitTests ${TEST_SRC_FILES})

    # headers for the project
    target_include_directories(runUnitTests PUBLIC
        ${RBDL_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        # ${DLIB_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    # Standard linking to gtest stuff.
    target_link_libraries(runUnitTests gtest_main)

    # Extra linking for the project.
    target_link_libraries(runUnitTests ${PROJECT_NAME})

    # This is so you can do 'make test' to see all your tests run, instead of
    # manually running the executable runUnitTests to see those specific tests.
    add_test(UnitTests runUnitTests)

    if (CMAKE_BUILD_TYPE STREQUAL "Coverage")
        set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/.travis/cmake)

        include(CodeCoverage)
        setup_target_for_coverage(${PROJECT_NAME}_coverage runUnitTests coverage)

        SET(CMAKE_CXX_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")
        SET(CMAKE_C_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")
    endif() #CMAKE_BUILD_TYPE STREQUAL "Coverage"

    # Copy the c3d of the example
    file(GLOB C3D_TEST_FILES ${CMAKE_CURRENT_SOURCE_DIR}/test/models/*.bioMod)
    file(COPY ${C3D_TEST_FILES}
         DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/models/)
endif()





