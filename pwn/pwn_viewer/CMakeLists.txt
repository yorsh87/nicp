INCLUDE(${QT_USE_FILE})

SET(UI_HEADERS gl_parameter.h)

QT4_WRAP_UI(UI_HEADERS pwn_aligner_gui_ui_main_window.ui)
QT4_WRAP_CPP(UI_SOURCES ${UI_HEADERS} pwn_aligner_gui_main_window.h)

ADD_LIBRARY(pwn_viewer SHARED
  drawable.cpp drawable.h
  drawable_correspondences.cpp drawable_correspondences.h
  drawable_covariances.cpp drawable_covariances.h
  drawable_cloud.cpp drawable_cloud.h
  drawable_normals.cpp drawable_normals.h
  drawable_points.cpp drawable_points.h
  drawable_trajectory.cpp drawable_trajectory.h
  drawable_transform_covariance.cpp drawable_transform_covariance.h
  drawable_uncertainty.cpp drawable_uncertainty.h
  gl_parameter.cpp gl_parameter.h
  gl_parameter_correspondences.cpp gl_parameter_correspondences.h
  gl_parameter_covariances.cpp gl_parameter_covariances.h
  gl_parameter_cloud.cpp gl_parameter_cloud.h 
  gl_parameter_normals.cpp gl_parameter_normals.h
  gl_parameter_points.cpp gl_parameter_points.h
  gl_parameter_trajectory.cpp gl_parameter_trajectory.h 
  gl_parameter_transform_covariance.cpp gl_parameter_transform_covariance.h
  gl_parameter_uncertainty.cpp gl_parameter_uncertainty.h
  imageview.cpp imageview.h
  pwn_qglviewer.cpp pwn_qglviewer.h
  opengl_primitives.cpp opengl_primitives.h
  pwn_aligner_gui_ui_main_window.h
  pwn_aligner_gui_main_window.cpp pwn_aligner_gui_main_window.h
  ${UI_HEADERS}
  ${UI_SOURCES}
)
SET_TARGET_PROPERTIES(pwn_viewer PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_viewer)
TARGET_LINK_LIBRARIES(pwn_viewer pwn ${OpenCV_LIBS} ${QGLVIEWER_LIBRARY})

ADD_EXECUTABLE(pwn_cloud_prop_viewer pwn_cloud_prop_viewer.cpp)
SET_TARGET_PROPERTIES(pwn_cloud_prop_viewer PROPERTIES OUTPUT_NAME pwn_cloud_prop_viewer)
TARGET_LINK_LIBRARIES(pwn_cloud_prop_viewer pwn_viewer pwn 
  ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} 
  ${OpenCV_LIBS}
)

ADD_EXECUTABLE(pwn_aligner_gui pwn_aligner_gui.cpp)
SET_TARGET_PROPERTIES(pwn_aligner_gui PROPERTIES OUTPUT_NAME pwn_aligner_gui)
TARGET_LINK_LIBRARIES(pwn_aligner_gui pwn pwn_viewer
  ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} 
  ${OpenCV_LIBS}
)

ADD_EXECUTABLE(pwn_simple_viewer pwn_simple_viewer.cpp )
INCLUDE_DIRECTORIES(
  ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} 
  ${QGLVIEWER_INCLUDE_DIR} ${QT_QTCORE_INCLUDE_DIR} ${QT_QTCORE_INCLUDE_DIR}/..)
TARGET_LINK_LIBRARIES(pwn_simple_viewer pwn_viewer pwn 
  ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} 
)