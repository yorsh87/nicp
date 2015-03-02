ADD_EXECUTABLE(pwn_generate_cropped_associations pwn_generate_cropped_associations.cpp)
SET_TARGET_PROPERTIES(pwn_generate_cropped_associations PROPERTIES OUTPUT_NAME pwn_generate_cropped_associations)
TARGET_LINK_LIBRARIES(pwn_generate_cropped_associations)

ADD_EXECUTABLE(pwn_pomerleau2munich pwn_pomerleau2munich.cpp)
SET_TARGET_PROPERTIES(pwn_pomerleau2munich PROPERTIES OUTPUT_NAME pwn_pomerleau2munich)
TARGET_LINK_LIBRARIES(pwn_pomerleau2munich ${OpenCV_LIBS} pwn)

CUDA_ADD_EXECUTABLE(kinfu_eth_kinect_evaluate kinfu_eth_kinect_evaluate.cpp
  pwn_kinfu_tracker.h
  image_view.cpp
)
TARGET_LINK_LIBRARIES(kinfu_eth_kinect_evaluate 
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  pwn
) 

ADD_EXECUTABLE(nicp_eth_kinect_evaluate nicp_eth_kinect_evaluate.cpp)
SET_TARGET_PROPERTIES(nicp_eth_kinect_evaluate PROPERTIES OUTPUT_NAME nicp_eth_kinect_evaluate)
TARGET_LINK_LIBRARIES(nicp_eth_kinect_evaluate 
  ${OpenCV_LIBS} 
  pwn pwn_viewer
  ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} 
)

ADD_EXECUTABLE(nicp_incremental_eth_kinect_evaluate nicp_incremental_eth_kinect_evaluate.cpp)
SET_TARGET_PROPERTIES(nicp_incremental_eth_kinect_evaluate PROPERTIES OUTPUT_NAME nicp_incremental_eth_kinect_evaluate)
TARGET_LINK_LIBRARIES(nicp_incremental_eth_kinect_evaluate 
  ${OpenCV_LIBS} 
  pwn pwn_viewer
  ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} 
)

ADD_EXECUTABLE(nicp_eth_laser_evaluate nicp_eth_laser_evaluate.cpp)
SET_TARGET_PROPERTIES(nicp_eth_laser_evaluate PROPERTIES OUTPUT_NAME nicp_eth_laser_evaluate)
TARGET_LINK_LIBRARIES(nicp_eth_laser_evaluate 
  ${OpenCV_LIBS} 
  pwn pwn_viewer
  ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} 
)