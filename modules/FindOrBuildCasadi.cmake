macro(FindOrBuildCasadi)   
    # Try to find Casadi first
    
    # If python is found with casadi install and was installed using pip, the library will be installed in the site-packages directory
    # So we should add the site-package folder to the searching for the library
    find_package(Python3 COMPONENTS Interpreter)
    if (Python3_FOUND)
        execute_process(
            COMMAND ${Python3_EXECUTABLE} -c "import casadi; print(casadi.__file__)"
            OUTPUT_VARIABLE Casadi_FROM_PIP
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
        get_filename_component(Casadi_FROM_PIP "${Casadi_FROM_PIP}" DIRECTORY)
        set(Casadi_FROM_PIP ${Casadi_FROM_PIP}/cmake)
    endif()

    find_package(Casadi QUIET
        PATHS ${Casadi_FROM_PIP}
    )

    if(Casadi_FOUND AND NOT Casadi_IS_BUILT)
        set(Casadi_IS_BUILT FALSE)
        if(NOT DEFINED Casadi_INCLUDE_DIR OR Casadi_INCLUDE_DIR STREQUAL "")
            # Modern CMake does not set INCLUDE_DIR, so make it retro-compatible
            get_target_property(Casadi_INCLUDE_DIR casadi INTERFACE_INCLUDE_DIRECTORIES)
            
            # We once tried to get it from INTERFACE_LINK_LIBRARIES but it was not working
            if(WIN32)
                file(GLOB_RECURSE Casadi_LIBRARY "${Casadi_INCLUDE_DIR}/../casadi.dll")
            else()
                file(GLOB_RECURSE Casadi_LIBRARY "${Casadi_INCLUDE_DIR}/../libcasadi.so")
            endif()
            
            set(Casadi_DIR_ARE_ADJUSTED FALSE)
        endif()

    else()
        message(STATUS "Casadi not found, downloading version 3.6.7 and installing from GitHub")
        include(ExternalProject)

        set(Casadi_IS_BUILT TRUE)
        if (INSTALL_DEPENDENCIES_ON_SYSTEM)
            set(Casadi_INSTALL_DIR ${CMAKE_INSTALL_PREFIX})
        else()
            set(Casadi_INSTALL_DIR "${CMAKE_BINARY_DIR}/Casadi_install")
        endif()
        
        # Detect correct library extension (OS-independent)
        if(WIN32)
            set(Casadi_LIB_NAME "casadi.lib")
        else()
            set(Casadi_LIB_NAME "libcasadi.so")
        endif()
        set(Casadi_LIBRARY "${Casadi_INSTALL_DIR}/lib/${Casadi_LIB_NAME}")

        ExternalProject_Add(Casadi_external
            GIT_REPOSITORY https://github.com/casadi/casadi.git
            GIT_TAG 3.6.7
            CMAKE_ARGS
                -DCMAKE_INSTALL_PREFIX=${Casadi_INSTALL_DIR}
                -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                -DCMAKE_POSITION_INDEPENDENT_CODE=ON
            BUILD_BYPRODUCTS "${Casadi_LIBRARY}"
        )

        # Define outputs
        set(Casadi_DIR "${Casadi_INSTALL_DIR}/share/casadi/cmake")
        set(Casadi_INCLUDE_DIR "${Casadi_INSTALL_DIR}/include/casadi")
        set(Casadi_FOUND TRUE)
        
        # Ensure that the library gets built before linking
        add_library(CASADI_BUILD INTERFACE IMPORTED)
        set_target_properties(CASADI_BUILD PROPERTIES
            IMPORTED_LOCATION "${Casadi_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${Casadi_INCLUDE_DIR}"
        )        
        
        add_dependencies(CASADI_BUILD Casadi_external)
    endif()

    # For some reason, the include directory is sometimes one level too high or too low
    if (NOT Casadi_DIR_ARE_ADJUSTED)
        # Find the casadi.hpp file from Casadi_INCLUDE_DIR, CasaDI_INCLUDE_DIR/../ or Casadi_INCLUDE_DIR/casadi
        if (EXISTS "${Casadi_INCLUDE_DIR}/casadi.hpp")
            # Do nothing
        elseif (EXISTS "${Casadi_INCLUDE_DIR}/casadi/casadi.hpp")
            set(Casadi_INCLUDE_DIR "${Casadi_INCLUDE_DIR}/casadi")
        elseif (EXISTS "${Casadi_INCLUDE_DIR}/../casadi.hpp")
            set(Casadi_INCLUDE_DIR "${Casadi_INCLUDE_DIR}/..")
        endif()
        set(Casadi_DIR_ARE_ADJUSTED TRUE)
    endif()

    
endmacro()