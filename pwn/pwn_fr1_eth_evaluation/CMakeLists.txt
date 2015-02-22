ADD_EXECUTABLE(pwn_munich_evaluate pwn_munich_evaluate.cpp)
SET_TARGET_PROPERTIES(pwn_munich_evaluate PROPERTIES OUTPUT_NAME pwn_munich_evaluate)
TARGET_LINK_LIBRARIES(pwn_munich_evaluate ${OpenCV_LIBS} pwn)

ADD_EXECUTABLE(gicp_munich_evaluate gicp_munich_evaluate.cpp)
SET_TARGET_PROPERTIES(gicp_munich_evaluate PROPERTIES OUTPUT_NAME gicp_munich_evaluate)
TARGET_LINK_LIBRARIES(gicp_munich_evaluate ${OpenCV_LIBS} pwn)

ADD_EXECUTABLE(pwn_generate_munich_associations pwn_generate_munich_associations.cpp)
SET_TARGET_PROPERTIES(pwn_generate_munich_associations PROPERTIES OUTPUT_NAME pwn_generate_munich_associations)
TARGET_LINK_LIBRARIES(pwn_generate_munich_associations)

ADD_EXECUTABLE(pwn_pomerleau2munich pwn_pomerleau2munich.cpp)
SET_TARGET_PROPERTIES(pwn_pomerleau2munich PROPERTIES OUTPUT_NAME pwn_pomerleau2munich)
TARGET_LINK_LIBRARIES(pwn_pomerleau2munich ${OpenCV_LIBS} pwn)

ADD_EXECUTABLE(pwn_eth_evaluate pwn_eth_evaluate.cpp)
SET_TARGET_PROPERTIES(pwn_eth_evaluate PROPERTIES OUTPUT_NAME pwn_eth_evaluate)
TARGET_LINK_LIBRARIES(pwn_eth_evaluate ${OpenCV_LIBS} pwn)

CUDA_ADD_EXECUTABLE(kinfu_munich_evaluate kinfu_munich_evaluate.cpp
  pwn_kinfu_tracker.h
  image_view.cpp
)
TARGET_LINK_LIBRARIES(kinfu_munich_evaluate 
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  pwn
) 

ADD_EXECUTABLE(ndt_munich_evaluate ndt_munich_evaluate.cpp)
TARGET_LINK_LIBRARIES(ndt_munich_evaluate 
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  pwn
) 

ADD_EXECUTABLE(nicp_tracker_app nicp_tracker_app.cpp)
SET_TARGET_PROPERTIES(nicp_tracker_app PROPERTIES OUTPUT_NAME nicp_tracker_app)
TARGET_LINK_LIBRARIES(nicp_tracker_app 
  ${OpenCV_LIBS} 
  pwn pwn_viewer
  ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} 
)