# Find the header files


FIND_PATH(PWN_INCLUDE_DIR pwn/aligner.h
  ${PROJECT_SOURCE_DIR}/../pwn
  ${PROJECT_SOURCE_DIR}/../../pwn
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
MACRO(FIND_PWN_LIBRARY MYLIBRARY MYLIBRARYNAME)
  FIND_LIBRARY("${MYLIBRARY}_DEBUG"
    NAMES "${MYLIBRARYNAME}_d"
    PATHS
    ${PROJECT_SOURCE_DIR}/../pwn/lib/Debug
    ${PROJECT_SOURCE_DIR}/../pwn/lib
    ${PROJECT_SOURCE_DIR}/../../pwn/lib/Debug
    ${PROJECT_SOURCE_DIR}/../../pwn/lib
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
    ${PROJECT_SOURCE_DIR}/../pwn/lib/Release
    ${PROJECT_SOURCE_DIR}/../pwn/lib
    ${PROJECT_SOURCE_DIR}/../../pwn/lib/Release
    ${PROJECT_SOURCE_DIR}/../../pwn/lib
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
ENDMACRO(FIND_PWN_LIBRARY LIBRARY LIBRARYNAME)

# Find the core elements
FIND_PWN_LIBRARY(PWN_LIBRARY pwn)
FIND_PWN_LIBRARY(PWN_BOSS_LIBRARY pwn_boss)
FIND_PWN_LIBRARY(PWN_BOSS_LIBRARY pwn_viewer)
