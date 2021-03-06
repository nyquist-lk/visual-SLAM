# Try to find the GLUES lib and include files
#
# GLUES_INCLUDE_DIR
# GLUES_LIBRARIES
# GLUES_FOUND

FIND_PATH( GLUES_INCLUDE_DIR glues/glues.h
  /usr/include
  /usr/local/include
  /opt/include
  /opt/local/include
  ${CMAKE_INSTALL_PREFIX}/include
)

FIND_LIBRARY( GLUES_LIBRARY glues
  /usr/lib64
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  /opt/local/lib64
  ${CMAKE_INSTALL_PREFIX}/lib
)

IF(GLUES_INCLUDE_DIR AND GLUES_LIBRARY)
  SET( GLUES_FOUND TRUE )
  SET( GLUES_LIBRARIES ${GLUES_LIBRARY} )
ENDIF(GLUES_INCLUDE_DIR AND GLUES_LIBRARY)

IF(GLUES_FOUND)
   IF(NOT GLUES_FIND_QUIETLY)
      MESSAGE(STATUS "Found GLUES: ${GLUES_LIBRARY}")
   ENDIF(NOT GLUES_FIND_QUIETLY)
ELSE(GLUES_FOUND)
   IF(GLUES_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find GLUES")
   ENDIF(GLUES_FIND_REQUIRED)
ENDIF(GLUES_FOUND)

