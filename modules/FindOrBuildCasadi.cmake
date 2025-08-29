function(FindOrBuildCasadi)      
    # If python is found with casadi install and was installed using pip, the library is fully installed in the site-packages directory
    # So we can add the site-package folder to the searching for the library.
    # This is however not working since the toolchain used to build on pip is incompatible with the one used to build the library
    # If this ever changes, we can uncomment the following lines
    # find_package(Python3 COMPONENTS Interpreter)
    # if (Python3_FOUND)
    #     execute_process(
    #         COMMAND ${Python3_EXECUTABLE} -c "import casadi; print(casadi.__file__)"
    #         OUTPUT_VARIABLE Casadi_FROM_PIP
    #         OUTPUT_STRIP_TRAILING_WHITESPACE
    #         ERROR_QUIET
    #     )
    #     if(Casadi_FROM_PIP)
    #         # There is a bug in the config file which necessitate the casadi interface to be defined
    #         # We create a dummy interface library to avoid issues
    #         add_library(casadi INTERFACE IMPORTED GLOBAL)
    #     endif()

    #     get_filename_component(Casadi_FROM_PIP "${Casadi_FROM_PIP}" DIRECTORY)
    #     set(Casadi_FROM_PIP ${Casadi_FROM_PIP}/cmake)
    # endif()

    # -------------------------------
    # Try finding existing install
    # -------------------------------
    find_package(Casadi QUIET
        # The linker does not work it the pip version of casadi. When this works, uncomment the following line and the Python section above
        # PATHS ${Casadi_FROM_PIP}
        PATHS ${INSTALL_DEPENDENCIES_PREFIX} ${CMAKE_INSTALL_PREFIX}
    )

    if(TARGET casadi::casadi)
        message(STATUS "Found Casadi (modern target)")
        # # We need to add the Casadi_LIBRARY to the casadi::casadi target
        # find_library(Casadi_LIBRARY NAMES casadi
        #     PATHS "${INSTALL_DEPENDENCIES_PREFIX}/lib" "${Casadi_INSTALL_DIR}/lib"
        # )
        # if (Casadi_LIBRARY)
        #     set_target_properties(casadi::casadi PROPERTIES
        #         Casadi_LIBRARY "${Casadi_LIBRARY}"
        #     )
        # endif()

        # # We also need to add /casadi as an extra include directory
        # get_target_property(Casadi_INCLUDE_DIR casadi::casadi INTERFACE_INCLUDE_DIRECTORIES)
        # set_target_properties(casadi::casadi PROPERTIES
        #     INTERFACE_INCLUDE_DIRECTORIES "${Casadi_INCLUDE_DIR};${Casadi_INCLUDE_DIR}/casadi"
        # )
        return()
    endif()

    # -------------------------------
    # If not found → build from source
    # -------------------------------
    # OpenBLAS drastically lags behind CMake and cannot be compiled anymore with modern CMake
    # We therefore cannot auto-build Casadi. Because of that the full auto-build process was
    # not fully tested and may or may not work when reintegrated (removing the next FATAL_ERROR)
    message(FATAL_ERROR 
        "Auto-build is not currently supported for Casadi (until OpenBLAS support CMake > 3.5). "
        "Please install it manually using (e.g. using conda)"
    )
    message(STATUS "Casadi not found, using version 3.7.0 from GitHub")
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
    set(Casadi_INCLUDE_DIR "${Casadi_INSTALL_DIR}/include/;${Casadi_INSTALL_DIR}/include/casadi")
    file(MAKE_DIRECTORY "${Casadi_INSTALL_DIR}/include/casadi")

    add_library(casadi::casadi INTERFACE IMPORTED)
    set_target_properties(casadi::casadi PROPERTIES
        IMPORTED_LOCATION_RELEASE "${Casadi_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${Casadi_INCLUDE_DIR}"
    )        
    set(Casadi_DIR "${Casadi_INSTALL_DIR}/share/casadi/cmake" PARENT_SCOPE)
    add_dependencies(casadi::casadi Casadi_external)
endfunction()