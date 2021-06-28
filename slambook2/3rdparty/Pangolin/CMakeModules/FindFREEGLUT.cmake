# Try to find the FREEGLUT library
#
# FREEGLUT_INCLUDE_DIR
# FREEGLUT_LIBRARY
# FREEGLUT_FOUND

FIND_PATH(
  FREEGLUT_INCLUDE_DIR GL/freeglut.h
  ${CMAKE_INCLUDE_PATH}
  $ENV{include}
  ${OPENGL_INCLUDE_DIR}
  /usr/include
  /usr/local/include
)

SET(STORE_CMAKE_FIND_FRAMEWORK ${CMAKE_FIND_FRAMEWORK})
SET(CMAKE_FIND_FRAMEWORK NEVER)

FIND_LIBRARY(
  FREEGLUT_LIBRARY
  NAMES freeglut_static freeglut glut
  PATH
    /opt/local/lib
    ${CMAKE_LIBRARY_PATH}
    $ENV{lib}
    /usr/lib
    /usr/local/lib
)

SET(CMAKE_FIND_FRAMEWORK ${STORE_CMAKE_FIND_FRAMEWORK})

IF (FREEGLUT_INCLUDE_DIR AND FREEGLUT_LIBRARY)
   SET(FREEGLUT_FOUND TRUE)
ENDIF (FREEGLUT_INCLUDE_DIR AND FREEGLUT_LIBRARY)

IF (FREEGLUT_FOUND)
   IF (NOT FREEGLUT_FIND_QUIETLY)
      MESSAGE(STATUS "Found FREEGLUT: ${FREEGLUT_LIBRARY}")
   ENDIF (NOT FREEGLUT_FIND_QUIETLY)
ELSE (FREEGLUT_FOUND)
   IF (FREEGLUT_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find FREEGLUT")
   ENDIF (FREEGLUT_FIND_REQUIRED)
ENDIF (FREEGLUT_FOUND)
