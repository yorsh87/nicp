CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(nicp)

# Optionally compiling part
OPTION(BUILD_NICP_VIEWER "enables NICP viewer" true)
OPTION(BUILD_NICP_TEST "enables NICP test" false)

# The library prefix
SET(LIB_PREFIX nicp)

SET(nicp_C_FLAGS)
SET(nicp_CXX_FLAGS)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

# postfix, based on type
SET(CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "postfix applied to debug build of libraries")
SET(CMAKE_RELEASE_POSTFIX "" CACHE STRING "postfix applied to release build of libraries")
SET(CMAKE_RELWITHDEBINFO_POSTFIX "_rd" CACHE STRING "postfix applied to release-with-debug-information libraries")
SET(CMAKE_MINSIZEREL_POSTFIX "_s" CACHE STRING "postfix applied to minimium-size-build libraries")

# work out the postfix; required where we use OUTPUT_NAME
IF(CMAKE_BUILD_TYPE MATCHES Release)
  SET(EXE_POSTFIX)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Debug)
  SET(EXE_POSTFIX ${CMAKE_DEBUG_POSTFIX})
ELSEIF(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
  SET(EXE_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
ELSEIF(CMAKE_BUILD_TYPE MATCHES MinSizeRel)
  SET(EXE_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

# Set the output directory for the build executables and libraries
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${nicp_SOURCE_DIR}/lib)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${nicp_SOURCE_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${nicp_SOURCE_DIR}/bin)

# Set search directory for looking for our custom CMake scripts to
# look for Eigen3
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${nicp_SOURCE_DIR}/cmake_modules)

ADD_DEFINITIONS(-DUNIX)
MESSAGE(STATUS "Compiling on Unix")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -msse4.2")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -msse4.2")
SET(nicp_C_FLAGS "${nicp_C_FLAGS} -Wall -W")
SET(nicp_CXX_FLAGS "${nicp_CXX_FLAGS} -Wall -W")

# Specifying compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${nicp_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${nicp_C_FLAGS}")

# Find Eigen3
FIND_PACKAGE(Eigen3 3.2.0 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

# Find Flann
FIND_PACKAGE(Flann REQUIRED)
INCLUDE_DIRECTORIES(${FLANN_INCLUDE_DIR})
LINK_DIRECTORIES(${FLANN_LIBRARY} )

# Find OpenMP
FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
 SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -D_NICP_USE_OPENMP_ -DEIGEN_DONT_PARALLELIZE")
 SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
ENDIF(OPENMP_FOUND)

# Find Opencv
FIND_PACKAGE(OpenCV 2.4.8 REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# Set up the top-level include directories
INCLUDE_DIRECTORIES(${nicp_SOURCE_DIR}/nicp)

IF(BUILD_NICP_VIEWER)
  # OpenGL is used in the draw actions for the different types, as well
  # as for creating the GUI itself
  FIND_PACKAGE(OpenGL REQUIRED)
  INCLUDE_DIRECTORIES(${OPENGL_INCLUDE})
  MESSAGE(STATUS "Compiling with OpenGL support")

  # Find Qt4
  FIND_PACKAGE(Qt4 REQUIRED)
  INCLUDE_DIRECTORIES(${QT_INCLUDES})

  # For building the GUI
  FIND_PACKAGE(QGLViewer REQUIRED)
  INCLUDE_DIRECTORIES(${QGLVIEWER_INCLUDE_DIR})
ENDIF(BUILD_NICP_VIEWER)

# Include the subdirectories
ADD_SUBDIRECTORY(nicp)
