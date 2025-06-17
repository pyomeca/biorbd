macro(FindOrBuildCasadi)      
    # # If python is found with casadi install and was installed using pip, the library will be installed in the site-packages directory
    # # So we should add the site-package folder to the searching for the library
    # find_package(Python3 COMPONENTS Interpreter)
    # if (Python3_FOUND)
    #     execute_process(
    #         COMMAND ${Python3_EXECUTABLE} -c "import casadi; print(casadi.__file__)"
    #         OUTPUT_VARIABLE Casadi_FROM_PIP
    #         OUTPUT_STRIP_TRAILING_WHITESPACE
    #         ERROR_QUIET
    #     )
    #     get_filename_component(Casadi_FROM_PIP "${Casadi_FROM_PIP}" DIRECTORY)
    #     set(Casadi_FROM_PIP ${Casadi_FROM_PIP}/cmake)
    # endif()

    # Try to find Casadi first
    find_package(Casadi QUIET
        # The linker does not work it the pip version of casadi. When this works, uncomment the following line and the Python section above
        # PATHS ${Casadi_FROM_PIP}
        PATHS ${INSTALL_DEPENDENCIES_PREFIX}
    )

    if(Casadi_FOUND AND NOT Casadi_IS_BUILT)
        message (STATUS "Casadi found")
        set(Casadi_IS_BUILT FALSE)

        if(NOT DEFINED Casadi_INCLUDE_DIR OR Casadi_INCLUDE_DIR STREQUAL "")
            # Modern CMake does not set INCLUDE_DIR, so make it retro-compatible
            get_target_property(Casadi_INCLUDE_DIR casadi INTERFACE_INCLUDE_DIRECTORIES)

            # We once tried to get it from INTERFACE_LINK_LIBRARIES but it was not working
            if(WIN32)
                file(GLOB_RECURSE Casadi_LIBRARY "${Casadi_INCLUDE_DIR}/../casadi.lib")
            elseif(LINUX)
                file(GLOB_RECURSE Casadi_LIBRARY "${Casadi_INCLUDE_DIR}/../libcasadi.so")
            elseif(APPLE)
                file(GLOB_RECURSE Casadi_LIBRARY "${Casadi_INCLUDE_DIR}/../libcasadi.dylib")
            else()
                message(FATAL_ERROR "Unsupported OS")
            endif()
            if (NOT Casadi_LIBRARY)
                # This should not happen. It probably means Casadi changed their building procedure
                message(FATAL_ERROR "Casadi library not found")
            endif()
            
            # For some reason, the include directory is sometimes one level too high or too low
            if (EXISTS "${Casadi_INCLUDE_DIR}/casadi.hpp")
                # Do nothing
            elseif (EXISTS "${Casadi_INCLUDE_DIR}/casadi/casadi.hpp")
                set(Casadi_INCLUDE_DIR "${Casadi_INCLUDE_DIR}/casadi")
            endif()
            
        endif()

    else()
        message(STATUS "Casadi not found, using version 3.7.0 from GitHub")
        set(Casadi_IS_BUILT TRUE)
        include(ExternalProject)
        
        if (NOT INSTALL_DEPENDENCIES_PREFIX OR INSTALL_DEPENDENCIES_PREFIX STREQUAL "")
            set(Casadi_INSTALL_DIR "${CMAKE_BINARY_DIR}/Casadi_install")
        else()
            set(Casadi_INSTALL_DIR ${INSTALL_DEPENDENCIES_PREFIX})
        endif()
        
        if (BUILD_SHARED_LIBS)
            set(Casadi_BUILD_SHARED_LIBS ON)
            set(Casadi_BUILD_STATIC_LIBS OFF)
        else()
            set(Casadi_BUILD_SHARED_LIBS OFF)
            set(Casadi_BUILD_STATIC_LIBS ON)
        endif()
        

        # Detect correct library extension (OS-independent)
        if(WIN32)
            set(Casadi_LIB_NAME "casadi.lib")
        else()
            if(${Casadi_BUILD_SHARED_LIBS})
                if(LINUX)
                    set(Casadi_LIB_NAME "libcasadi.so")
                elseif(APPLE)
                    set(Casadi_LIB_NAME "libcasadi.dylib")
                else()
                    message(FATAL_ERROR "Unsupported OS")
                endif() 
            else()
                set(Casadi_LIB_NAME "libcasadi.a")
            endif()
        endif()
        set(Casadi_LIBRARY "${Casadi_INSTALL_DIR}/lib/${Casadi_LIB_NAME}")

        if (BINDER_PYTHON3)
            find_package(Python3 COMPONENTS Interpreter Development)
            if (NOT Python3_FOUND)
                message(FATAL_ERROR "Python3 not found. Please install it before building the Python interface.")
            endif()
        endif()

        ExternalProject_Add(Casadi_external
            GIT_REPOSITORY https://github.com/casadi/casadi.git
            GIT_TAG 3.7.0
            CMAKE_ARGS
                -DCMAKE_INSTALL_PREFIX=${Casadi_INSTALL_DIR}
                -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                -DENABLE_SHARED=${Casadi_BUILD_SHARED_LIBS}
                -DENABLE_STATIC=${Casadi_BUILD_STATIC_LIBS}
                -DCMAKE_POSITION_INDEPENDENT_CODE=ON
                -DWITH_PYTHON=${BINDER_PYTHON3}
                -DWITH_PYTHON3=${BINDER_PYTHON3}
                -DPYTHON_PREFIX=${Python3_SITELIB}
                -DWITH_IPOPT=ON
                -DWITH_BUILD_IPOPT=ON
                -DWITH_THREAD=ON
                -DWITH_BUILD_MUMPS=ON
                -DWITH_BUILD_METIS=ON
                -DWITH_BUILD_LAPACK=ON
            BUILD_BYPRODUCTS "${Casadi_LIBRARY}"
        )

        # Define outputs
        set(Casadi_DIR "${Casadi_INSTALL_DIR}/share/casadi/cmake")
        set(Casadi_INCLUDE_DIR "${Casadi_INSTALL_DIR}/include/;${Casadi_INSTALL_DIR}/include/casadi")
        set(Casadi_FOUND TRUE)
        
        # Ensure that the library gets built before linking
        add_library(CASADI_BUILD INTERFACE IMPORTED)
        set_target_properties(CASADI_BUILD PROPERTIES
            IMPORTED_LOCATION "${Casadi_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${Casadi_INCLUDE_DIR}"
        )        
        
        add_dependencies(CASADI_BUILD Casadi_external)
    endif()

endmacro()