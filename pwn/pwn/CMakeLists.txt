ADD_LIBRARY(pwn SHARED
  definitions.h
  imageutils.cpp imageutils.h	
  aligner.cpp aligner.h
  correspondencefinder.cpp correspondencefinder.h
  cylindricalpointprojector.cpp cylindricalpointprojector.h
  depthimageconverter.cpp depthimageconverter.h
  depthimageconverterintegralimage.cpp depthimageconverterintegralimage.h
  cloud.cpp cloud.h
  gaussian.h
  gaussian3.cpp gaussian3.h
  homogeneousvector4f.h
  informationmatrix.h
  informationmatrixcalculator.h informationmatrixcalculator.cpp
  linearizer.cpp linearizer.h
  merger.cpp merger.h
  multipointprojector.cpp multipointprojector.h 
  pinholepointprojector.cpp pinholepointprojector.h
  pointaccumulator.h
  pointintegralimage.cpp pointintegralimage.h
  pointprojector.cpp pointprojector.h
  se3_prior.cpp se3_prior.h
  parallelcylindricalpointprojector.cpp parallelcylindricalpointprojector.h
  stats.h
  statscalculator.cpp statscalculator.h
  statscalculatorintegralimage.cpp statscalculatorintegralimage.h
  sphericalpointprojector.cpp sphericalpointprojector.h
  transformable_vector.h
  voxelcalculator.cpp voxelcalculator.h
)
SET_TARGET_PROPERTIES(pwn PROPERTIES OUTPUT_NAME ${LIB_PREFIX})
TARGET_LINK_LIBRARIES(pwn ${OpenCV_LIBS})

ADD_EXECUTABLE(pwn_simple_aligner pwn_simple_aligner.cpp )
SET_TARGET_PROPERTIES(pwn_simple_aligner PROPERTIES OUTPUT_NAME pwn_simple_aligner)
TARGET_LINK_LIBRARIES(pwn_simple_aligner pwn ${OpenCV_LIBS})

ADD_EXECUTABLE(pwn_aligner pwn_aligner.cpp )
SET_TARGET_PROPERTIES(pwn_aligner PROPERTIES OUTPUT_NAME pwn_aligner)
TARGET_LINK_LIBRARIES(pwn_aligner pwn ${OpenCV_LIBS})
