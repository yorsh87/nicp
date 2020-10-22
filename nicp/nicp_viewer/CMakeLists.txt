SET(UI_HEADERS gl_parameter.h)

QT5_WRAP_UI(UI_HEADERS nicp_aligner_gui_ui_main_window.ui)
QT5_WRAP_CPP(UI_SOURCES ${UI_HEADERS} nicp_aligner_gui_main_window.h)

ADD_LIBRARY(nicp_viewer SHARED
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
  nicp_qglviewer.cpp nicp_qglviewer.h
  opengl_primitives.cpp opengl_primitives.h
  nicp_aligner_gui_ui_main_window.h
  nicp_aligner_gui_main_window.cpp nicp_aligner_gui_main_window.h
  ${UI_HEADERS}
  ${UI_SOURCES}
)
SET_TARGET_PROPERTIES(nicp_viewer PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_viewer)
TARGET_LINK_LIBRARIES(nicp_viewer nicp ${OpenCV_LIBS} ${QGLVIEWER_LIBRARY})

ADD_EXECUTABLE(nicp_cloud_prop_viewer nicp_cloud_prop_viewer.cpp)
SET_TARGET_PROPERTIES(nicp_cloud_prop_viewer PROPERTIES OUTPUT_NAME nicp_cloud_prop_viewer)
TARGET_LINK_LIBRARIES(nicp_cloud_prop_viewer nicp_viewer nicp 
  ${QGLVIEWER_LIBRARY} Qt5::Xml Qt5::OpenGL Qt5::Gui Qt5::Core Qt5::Widgets 
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} 
  ${OpenCV_LIBS}
)

ADD_EXECUTABLE(nicp_aligner_gui nicp_aligner_gui.cpp)
SET_TARGET_PROPERTIES(nicp_aligner_gui PROPERTIES OUTPUT_NAME nicp_aligner_gui)
TARGET_LINK_LIBRARIES(nicp_aligner_gui nicp nicp_viewer
  ${QGLVIEWER_LIBRARY} Qt5::Xml Qt5::OpenGL Qt5::Gui Qt5::Core Qt5::Widgets
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} 
  ${OpenCV_LIBS}
)

ADD_EXECUTABLE(nicp_simple_viewer nicp_simple_viewer.cpp )
INCLUDE_DIRECTORIES(
  ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} 
  ${QGLVIEWER_INCLUDE_DIR} ${QT_QTCORE_INCLUDE_DIR} ${QT_QTCORE_INCLUDE_DIR}/..)
TARGET_LINK_LIBRARIES(nicp_simple_viewer nicp_viewer nicp 
  ${QGLVIEWER_LIBRARY} Qt5::Xml Qt5::OpenGL Qt5::Gui Qt5::Core Qt5::Widgets
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} 
)
