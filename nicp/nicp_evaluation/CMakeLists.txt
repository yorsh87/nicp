ADD_EXECUTABLE(nicp_incremental_eth_kinect_evaluate nicp_incremental_eth_kinect_evaluate.cpp)
SET_TARGET_PROPERTIES(nicp_incremental_eth_kinect_evaluate PROPERTIES OUTPUT_NAME nicp_incremental_eth_kinect_evaluate)
TARGET_LINK_LIBRARIES(nicp_incremental_eth_kinect_evaluate
  ${OpenCV_LIBS}
  nicp nicp_viewer
  ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY}
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)
