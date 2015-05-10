# Find the header files

MESSAGE(STATUS "NICP_ROOT: " $ENV{NICP_ROOT})

FIND_PATH(NICP_INCLUDE_DIR nicp/aligner.h
  $ENV{NICP_ROOT}/nicp
  /usr/local/include
  /usr/include
  /opt/local/include
  /sw/local/include
  /sw/include
  NO_DEFAULT_PATH
 )

# Macro to unify finding both the debug and release versions of the
# libraries; this is adapted from the OpenSceneGraph FIND_LIBRARY
# macro.
MACRO(FIND_NICP_LIBRARY MYLIBRARY MYLIBRARYNAME)
  FIND_LIBRARY("${MYLIBRARY}_DEBUG"
    NAMES "${MYLIBRARYNAME}_d"
    PATHS
    $ENV{NICP_ROOT}/lib/Debug
    $ENV{NICP_ROOT}/lib
    NO_DEFAULT_PATH
  )

  FIND_LIBRARY("${MYLIBRARY}_DEBUG"
    NAMES "${MYLIBRARYNAME}_d"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /opt/local/lib
    /sw/local/lib
    /sw/lib
  )
  
  FIND_LIBRARY(${MYLIBRARY}
    NAMES "${MYLIBRARYNAME}"
    PATHS
    $ENV{NICP_ROOT}/lib/Release
    $ENV{NICP_ROOT}/lib
    NO_DEFAULT_PATH
  )

  FIND_LIBRARY(${MYLIBRARY}
    NAMES "${MYLIBRARYNAME}"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /opt/local/lib
    /sw/local/lib
    /sw/lib
  )
  
  IF(NOT ${MYLIBRARY}_DEBUG)
    IF(MYLIBRARY)
      SET(${MYLIBRARY}_DEBUG ${MYLIBRARY})
    ENDIF(MYLIBRARY)
  ENDIF(NOT ${MYLIBRARY}_DEBUG)
ENDMACRO(FIND_NICP_LIBRARY LIBRARY LIBRARYNAME)

# Find the core elements
FIND_NICP_LIBRARY(NICP_LIBRARY nicp)
FIND_NICP_LIBRARY(NICP_VIEWER_LIBRARY nicp_viewer)
