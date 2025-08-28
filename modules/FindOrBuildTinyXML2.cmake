function(FindOrBuildTinyXML2)
    # -------------------------------
    # Try finding existing install
    # -------------------------------
    find_package(TinyXML2 QUIET
        PATHS ${INSTALL_DEPENDENCIES_PREFIX} ${CMAKE_INSTALL_PREFIX}
    )

    if (TARGET tinyxml2::tinyxml2)
        message (STATUS "TinyXML2 found")
        return()
    endif()

    # -------------------------------
    # If not found → build from source
    # -------------------------------
    message(STATUS "TinyXML2 not found, using version 10.1.0 from GitHub")
    include(ExternalProject)

    if (NOT INSTALL_DEPENDENCIES_PREFIX OR INSTALL_DEPENDENCIES_PREFIX STREQUAL "")
        set(TinyXML2_INSTALL_DIR "${CMAKE_BINARY_DIR}/TinyXML2_install")
    else()
        set(TinyXML2_INSTALL_DIR ${INSTALL_DEPENDENCIES_PREFIX})
    endif()

    # Detect correct static library extension (OS-independent)
    set(TynyXML2_BUILD_SHARED_LIBS OFF)
    if(WIN32)
        set(TinyXML2_LIB_NAME "tinyxml2.lib")
    elseif(LINUX)
        set(TinyXML2_LIB_NAME "libtinyxml2.a")
    elseif(APPLE)
        set(TinyXML2_LIB_NAME "libtinyxml2.a")
    else()
        message(FATAL_ERROR "Unsupported OS")
    endif()

    ExternalProject_Add(TinyXML2_external
        GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
        GIT_TAG 10.1.0
        CMAKE_ARGS
            -DCMAKE_INSTALL_PREFIX=${TinyXML2_INSTALL_DIR}
            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
            -DBUILD_SHARED_LIBS=${TynyXML2_BUILD_SHARED_LIBS}
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        BUILD_BYPRODUCTS "${TinyXML2_INSTALL_DIR}/lib/${TinyXML2_LIB_NAME}"
    )

    # Ensure directory exists at configure time
    file(MAKE_DIRECTORY "${TinyXML2_INSTALL_DIR}/include")

    # Define imported target
    add_library(tinyxml2::tinyxml2 UNKNOWN IMPORTED)
    set_target_properties(tinyxml2::tinyxml2 PROPERTIES
        IMPORTED_LOCATION "${TinyXML2_INSTALL_DIR}/lib/${TinyXML2_LIB_NAME}"
        INTERFACE_INCLUDE_DIRECTORIES "${TinyXML2_INSTALL_DIR}/include"
    )

    # Ensure all targets depending on TinyXML2 wait for it to build
    add_dependencies(tinyxml2::tinyxml2 TinyXML2_external)
endfunction()