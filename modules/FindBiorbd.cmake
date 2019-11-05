# Searches for biorbd includes and library files
#
# Sets the variables
#   BIORBD_FOUND
#   BIORBD_INCLUDE_DIR
#   BIORBD_LIBRARY

SET (BIORBD_FOUND FALSE)

FIND_PATH (BIORBD_INCLUDE_DIR biorbd.h
	/usr/include
  /usr/include/biorbd
	/usr/local/include
  /usr/local/include/biorbd
  ${CMAKE_INSTALL_PREFIX}/include
  ${CMAKE_INSTALL_PREFIX}/include/biorbd
	$ENV{HOME}/local/include
  $ENV{HOME}/local/include/biorbd
  $ENV{BIORBD_PATH}/include
  $ENV{BIORBD_PATH}/include/biorbd
  $ENV{BIORBD_INCLUDE_PATH}
)
FIND_LIBRARY (BIORBD_LIBRARY NAMES biorbd	PATHS
  /usr/lib
  /usr/lib/biorbd
  /usr/local/lib
  /usr/local/lib/biorbd
  ${CMAKE_INSTALL_PREFIX}/lib
  ${CMAKE_INSTALL_PREFIX}/lib/biorbd
  $ENV{HOME}/local/lib
  $ENV{HOME}/local/lib/biorbd
  $ENV{BIORBD_PATH}
  $ENV{BIORBD_LIBRARY_PATH}
)

IF (BIORBD_INCLUDE_DIR AND BIORBD_LIBRARY)
        SET (BIORBD_FOUND TRUE)
ENDIF (BIORBD_INCLUDE_DIR AND BIORBD_LIBRARY)

IF (BIORBD_FOUND)
   IF (NOT BIORBD_FIND_QUIETLY)
      MESSAGE(STATUS "Found biorbd: ${BIORBD_LIBRARY}")
   ENDIF (NOT BIORBD_FIND_QUIETLY)
ELSE (BIORBD_FOUND)
   IF (BIORBD_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find biorbd")
   ENDIF (BIORBD_FIND_REQUIRED)
ENDIF (BIORBD_FOUND)

MARK_AS_ADVANCED (
        BIORBD_FOUND
        BIORBD_INCLUDE_DIR
        BIORBD_LIBRARY
	)
