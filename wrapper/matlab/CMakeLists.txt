project(S2M_rbdl_matlab)
cmake_minimum_required(VERSION 3.1)

find_package(MATLAB REQUIRED)
set(MATLAB_WRAPPER_PATH ${CMAKE_CURRENT_SOURCE_DIR}/wrapper/matlab)

# Get the file to compile
set(CPP_FILE ${MATLAB_WRAPPER_PATH}/MexS2M_rbdl.cpp)

# Compile S2M_rbdl
INCLUDE_DIRECTORIES(${Matlab_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(${PROJECT_NAME} SHARED ${CPP_FILE} ${MATLAB_WRAPPER_PATH}/Matlabdef.def)

# Set the name of the library
if(WIN32)
  if (CMAKE_CL_64)
      SET_TARGET_PROPERTIES(S2M_rbdl_matlab PROPERTIES SUFFIX .mexw64)
  else(CMAKE_CL_64)
      SET_TARGET_PROPERTIES(S2M_rbdl_matlab PROPERTIES SUFFIX .mexw32)
  endif(CMAKE_CL_64)
else(WIN32)
  if (CMAKE_SIZEOF_VOID_P MATCHES "8")
      SET_TARGET_PROPERTIES(S2M_rbdl_matlab PROPERTIES SUFFIX .mexa64 PREFIX "")
  else(CMAKE_SIZEOF_VOID_P MATCHES "8")
      SET_TARGET_PROPERTIES(S2M_rbdl_matlab PROPERTIES SUFFIX .mexglx PREFIX "")
  endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
endif(WIN32)

target_link_libraries(${PROJECT_NAME} S2M_rbdl)
install(TARGETS ${PROJECT_NAME} DESTINATION ${Matlab_ROOT_DIR}/toolbox/S2M_rbdl)





