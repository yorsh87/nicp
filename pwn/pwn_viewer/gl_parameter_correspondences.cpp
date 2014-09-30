#include "gl_parameter_correspondences.h"

namespace pwn_viewer {

  GLParameterCorrespondences::GLParameterCorrespondences() : GLParameter() {
    _pointSize = 0.1f;
    _color = Eigen::Vector4f(0.0f, 0.0f, 1.0f, 0.5f);
    _lineWidth = 3.0f;
  }

  GLParameterCorrespondences::GLParameterCorrespondences(float pointSize_, Eigen::Vector4f color_, float lineWidth_) : GLParameter() {
    _pointSize = pointSize_;
    _color = color_;
    _lineWidth = lineWidth_;
  }

  void GLParameterCorrespondences::applyGLParameter() {
    glColor4f(_color[0], _color[1], _color[2], _color[3]);
    glPointSize(_pointSize);
    glLineWidth(_lineWidth);
  }

}
