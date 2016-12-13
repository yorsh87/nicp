ADD_LIBRARY(nicp SHARED
  definitions.h
  imageutils.cpp imageutils.h
  aligner.cpp aligner.h
  alignernn.cpp alignernn.h
  alignerprojective.cpp alignerprojective.h
  correspondencefinder.cpp correspondencefinder.h
  correspondencefindernn.cpp correspondencefindernn.h
  correspondencefinderprojective.cpp correspondencefinderprojective.h
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
SET_TARGET_PROPERTIES(nicp PROPERTIES OUTPUT_NAME ${LIB_PREFIX})
TARGET_LINK_LIBRARIES(nicp ${OpenCV_LIBS} ${FLANN_LIBRARY})

ADD_EXECUTABLE(nicp_simple_aligner nicp_simple_aligner.cpp )
SET_TARGET_PROPERTIES(nicp_simple_aligner PROPERTIES OUTPUT_NAME nicp_simple_aligner)
TARGET_LINK_LIBRARIES(nicp_simple_aligner nicp ${OpenCV_LIBS})

ADD_EXECUTABLE(nicp_aligner nicp_aligner.cpp )
SET_TARGET_PROPERTIES(nicp_aligner PROPERTIES OUTPUT_NAME nicp_aligner)
TARGET_LINK_LIBRARIES(nicp_aligner nicp ${OpenCV_LIBS})
