set(MASTER_PROJECT_NAME ${PROJECT_NAME})

project(${MASTER_PROJECT_NAME}_matlab)
cmake_minimum_required(VERSION 3.1)

find_package(Matlab REQUIRED COMPONENTS MX_LIBRARY)


# Get the file to compile
set(CPP_FILES ${CMAKE_CURRENT_SOURCE_DIR}/biorbd.cpp ${CMAKE_CURRENT_SOURCE_DIR}/S2M_rbdl.cpp)

foreach(CPP_FILE ${CPP_FILES})
    # Get the name of the current file
    get_filename_component(FILE_NAME ${CPP_FILE} NAME_WE)
    set(MATLAB_PROJECT_NAME ${FILE_NAME}_matlab)

    # Add project
    add_library(${MATLAB_PROJECT_NAME} SHARED ${CPP_FILE} ${CMAKE_CURRENT_SOURCE_DIR}/Matlabdef.def)
    # Reset the name of the output
    if(WIN32)
        SET_TARGET_PROPERTIES(${MATLAB_PROJECT_NAME} PROPERTIES OUTPUT_NAME ${FILE_NAME})
    else(WIN32)
        SET_TARGET_PROPERTIES(${MATLAB_PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_NAME ${FILE_NAME})
    endif(WIN32)

    # Add headers
    message(${Matlab_INCLUDE_DIRS})
    target_include_directories(${MATLAB_PROJECT_NAME} PUBLIC
        ${Matlab_INCLUDE_DIRS}
        ${RBDL_INCLUDE_DIR}
        ${RBDL_INCLUDE_DIR}/..
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${DLIB_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    )

    # Set the name of the library
    if(WIN32)
      if (CMAKE_CL_64)
          SET_TARGET_PROPERTIES(${MATLAB_PROJECT_NAME} PROPERTIES SUFFIX .mexw64)
      else(CMAKE_CL_64)
          SET_TARGET_PROPERTIES(${MATLAB_PROJECT_NAME} PROPERTIES SUFFIX .mexw32)
      endif(CMAKE_CL_64)
    elseif(APPLE)
      if (CMAKE_SIZEOF_VOID_P MATCHES "8")
          SET_TARGET_PROPERTIES(${MATLAB_PROJECT_NAME} PROPERTIES SUFFIX .mexmaci64 PREFIX "")
      else(CMAKE_SIZEOF_VOID_P MATCHES "8")
          SET_TARGET_PROPERTIES(${MATLAB_PROJECT_NAME} PROPERTIES SUFFIX .mexmaci64 PREFIX "")
      endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
    else()
      if (CMAKE_SIZEOF_VOID_P MATCHES "8")
          SET_TARGET_PROPERTIES(${MATLAB_PROJECT_NAME} PROPERTIES SUFFIX .mexa64 PREFIX "")
      else(CMAKE_SIZEOF_VOID_P MATCHES "8")
          SET_TARGET_PROPERTIES(${MATLAB_PROJECT_NAME} PROPERTIES SUFFIX .mexglx PREFIX "")
      endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
    endif(WIN32)

    # Set runtime path for linux
    SET_TARGET_PROPERTIES(${MATLAB_PROJECT_NAME} PROPERTIES INSTALL_RPATH ${${MASTER_PROJECT_NAME}_BIN_FOLDER})
    SET_TARGET_PROPERTIES(${MATLAB_PROJECT_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

    # Linker and instalation
    target_link_libraries(${MATLAB_PROJECT_NAME} ${MASTER_PROJECT_NAME} ${Matlab_MX_LIBRARY} ${Matlab_MEX_LIBRARY})

    # install target
    install(TARGETS ${MATLAB_PROJECT_NAME} DESTINATION ${Matlab_${MASTER_PROJECT_NAME}_INSTALL_DIR}/${PROJECT_NAME})
endforeach(CPP_FILE)

# Installation path
set(Matlab_${MASTER_PROJECT_NAME}_INSTALL_DIR ${Matlab_ROOT_DIR}/toolbox CACHE PATH "Toolbox path to install, please note that the default directory may require administrator access")
