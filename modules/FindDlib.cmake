# - Find Dlib
# Find the native Dlib includes and library
#
#  DLIB_INCLUDE_DIR - where to find zlib.h, etc.
#  DLIB_LIBRARIES   - List of libraries when using zlib.
#  DLIB_FOUND       - True if zlib found.

if (DLIB_INCLUDE_DIR)
  # Already in cache, be silent
  set (Dlib_FIND_QUIETLY TRUE)
endif (DLIB_INCLUDE_DIR)

find_path (DLIB_INCLUDE_DIR "dlib/algs.h"
  PATHS "${DLIB_DIR}")

if(WIN32)
	# For some reason find_library won't find dlib with only "dlib" name, it must have the full previously unknown name
	FILE(GLOB LibPath ${CMAKE_INSTALL_PREFIX}/lib/dlib*)
	get_filename_component(LibNames ${LibPath} NAME)
	find_library (DLIB_LIBRARY 
			NAMES ${LibNames}
			HINTS ${CMAKE_INSTALL_PREFIX}/bin)
else()
	set (DLIB_NAMES dlib)
	find_library (DLIB_LIBRARY NAMES ${DLIB_NAMES})
endif()


# handle the QUIETLY and REQUIRED arguments and set DLIB_FOUND to TRUE if 
# all listed variables are TRUE
include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (DLIB DEFAULT_MSG 
  DLIB_LIBRARY 
  DLIB_INCLUDE_DIR)

if (DLIB_FOUND)
  set (DLIB_LIBRARIES ${DLIB_LIBRARY})
else (DLIB_FOUND)
  set (DLIB_LIBRARIES)
endif (DLIB_FOUND)
mark_as_advanced (DLIB_LIBRARY DLIB_INCLUDE_DIR)
