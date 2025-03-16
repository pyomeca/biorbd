macro(FindOrBuildTinyXML2)   
    include(ExternalProject)

    find_package(TinyXML2 QUIET
        PATHS ${INSTALL_DEPENDENCIES_PREFIX}
    )

    if (TinyXML2_FOUND AND NOT TinyXML2e_IS_BUILT)
        set(TinyXML2_IS_BUILT FALSE)
        if (NOT DEFINED TinyXML2_INCLUDE_DIR OR TinyXML2_INCLUDE_DIR STREQUAL "")
            # Modern CMake does not set INCLUDE_DIR, so make it retro-compatible
            get_target_property(TinyXML2_INCLUDE_DIR tinyxml2::tinyxml2 INTERFACE_INCLUDE_DIRECTORIES)
            
            # We once tried to get it from INTERFACE_LINK_LIBRARIES but it was not working
            if(WIN32)
                file(GLOB_RECURSE TinyXML2_LIBRARY "${TinyXML2_INCLUDE_DIR}/../tinyxml2.dll")
            else()
                file(GLOB_RECURSE TinyXML2_LIBRARY "${TinyXML2_INCLUDE_DIR}/../libtinyxml2.so")
            endif()
        endif()

    else()
        message(STATUS "TinyXML2 not found, downloading version 10.1.0 and installing from GitHub")

        set(TinyXML2_IS_BUILT TRUE)
        if (NOT INSTALL_DEPENDENCIES_PREFIX OR INSTALL_DEPENDENCIES_PREFIX STREQUAL "")
            set(TinyXML2_INSTALL_DIR "${CMAKE_BINARY_DIR}/TinyXML2_install")
        else()
            set(TinyXML2_INSTALL_DIR ${INSTALL_DEPENDENCIES_PREFIX})
        endif()
        set(TynyXML2_BUILD_SHARED_LIBS OFF)

        # Detect correct static library extension (OS-independent)
        if(WIN32)
            set(TinyXML2_LIB_NAME "tinyxml2.lib")
        else()
            set(TinyXML2_LIB_NAME "libtinyxml2.a")
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

        # Define outputs
        set(TinyXML2_DIR "${TinyXML2_INSTALL_DIR}/share/tinyxml2/cmake")
        set(TinyXML2_INCLUDE_DIR "${TinyXML2_INSTALL_DIR}/include")
        set(TinyXML2_LIBRARY "${TinyXML2_INSTALL_DIR}/lib/${TinyXML2_LIB_NAME}")
        set(TinyXML2_FOUND TRUE)

        # Ensure all targets depending on TinyXML2 wait for it to build
        add_library(TinyXML2_BUILD INTERFACE IMPORTED)
        add_dependencies(TinyXML2_BUILD TinyXML2_external)

    endif()

endmacro()