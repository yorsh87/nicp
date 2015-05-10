#include "gl_parameter_normals.h"

namespace nicp_viewer {

  GLParameterNormals::GLParameterNormals() : GLParameter() {
    _pointSize = 0.1f;
    _color = Eigen::Vector4f(0.0f, 0.0f, 1.0f, 0.5f);
    _normalLength = 0.02f;
    _lineWidth = 1.0;
  }

  GLParameterNormals::GLParameterNormals(float pointSize_, Eigen::Vector4f color_, float normalLength_, float lineWidth_) : GLParameter() {
    _pointSize = pointSize_;
    _color = color_;
    _normalLength = normalLength_;
    _lineWidth = lineWidth_;
  }

  void GLParameterNormals::applyGLParameter() {
    glColor4f(_color[0], _color[1], _color[2], _color[3]);
    glPointSize(_pointSize);
    glLineWidth(_lineWidth);
  }

}
