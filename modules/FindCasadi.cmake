# - Find Casadi
# Find the native Casadi includes and libraries
#
#  Casadi_INCLUDE_DIR - where to find casadi.hpp, etc.
#  Casadi_LIBRARIES   - List of libraries when using Casadi.
#  Casadi_FOUND       - True if Casadi is found.

if (Casadi_INCLUDE_DIR)
  # Already in cache, be silent
  set (Casadi_FIND_QUIETLY TRUE)
endif (Casadi_INCLUDE_DIR)

# Check if Casadi_ROOT_DIR is provided, otherwise add it to the available variables to the user
if (NOT Casadi_ROOT_DIR)
    set (Casadi_ROOT_DIR "" CACHE PATH "Root directory of Casadi")
endif ()

find_path (Casadi_INCLUDE_DIR "casadi.hpp" 
    PATHS 
    ${CMAKE_INSTALL_PREFIX}/include/casadi 
    ${CMAKE_INSTALL_PREFIX}/Library/include/casadi
    ${Casadi_ROOT_DIR}/include
)

# Find the library
find_library (Casadi_LIBRARY NAMES casadi 
    PATHS 
    ${CMAKE_INSTALL_PREFIX}/lib 
    ${CMAKE_INSTALL_PREFIX}/Library/lib
    
)

# handle the QUIETLY and REQUIRED arguments and set Casadi_FOUND to TRUE if
# all listed variables are TRUE
include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (Casadi DEFAULT_MSG 
    Casadi_LIBRARY
    Casadi_INCLUDE_DIR
)

# Add DLL to the CasaDi library
if (WIN32)
    set (Casadi_LIBRARY ${Casadi_LIBRARY} ${Casadi_ROOT_DIR}/casadi/libcasadi.dll)
endif ()
