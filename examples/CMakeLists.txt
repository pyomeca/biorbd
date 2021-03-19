project(${BIORBD_NAME}_example)

set(EXAMPLE_FILES
    "forwardKinematicsExample.cpp"
    "forwardDynamicsExample.cpp"
    "inverseDynamicsExample.cpp"
)
if (MODULE_MUSCLES)
    list(APPEND EXAMPLE_FILES "forwardDynamicsFromMusclesExample.cpp")
endif()
if (MODULE_STATIC_OPTIM)
    list(APPEND EXAMPLE_FILES "staticOptimizationExample.cpp")
endif()
if (MODULE_KALMAN)
    list(APPEND EXAMPLE_FILES "inverseKinematicsKalmanExample.cpp")
endif()
if (MODULE_MUSCLES)
    list(APPEND EXAMPLE_FILES "WrappingObjectsExample.cpp")
endif()

foreach(FILE ${EXAMPLE_FILES})
    # Get the name of the current file
    get_filename_component(FILE_NAME ${FILE} NAME_WE)
    set(PROJECT_NAME ${FILE_NAME})

    add_executable(${PROJECT_NAME} ${FILE})
    add_dependencies(${PROJECT_NAME} ${BIORBD_NAME})

    # Headers
    target_include_directories(${PROJECT_NAME} PRIVATE
        "${CMAKE_SOURCE_DIR}/include"
        "${BIORBD_BINARY_DIR}/include"
        "${RBDL_INCLUDE_DIR}"
        "${RBDL_INCLUDE_DIR}/.."
        "${IPOPT_INCLUDE_DIR}"
        "${MATH_BACKEND_INCLUDE_DIR}"
    )

    # Linker and instalation
    target_link_libraries(${PROJECT_NAME}
        "${BIORBD_NAME}"
    )
endforeach()

# Copy the c3d of the example
file(COPY
    ${CMAKE_CURRENT_SOURCE_DIR}/pyomecaman.bioMod
    ${CMAKE_CURRENT_SOURCE_DIR}/arm26.bioMod
    ${CMAKE_CURRENT_SOURCE_DIR}/WrappingObjectExample.bioMod
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
