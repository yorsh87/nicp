#include "gl_parameter_uncertainty.h"

namespace pwn_viewer {

  GLParameterUncertainty::GLParameterUncertainty() : GLParameter() {
    _pointSize = 1.0f;
    _color = Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f);
    _ellipsoidScale = 0.05f;
  }

  GLParameterUncertainty::GLParameterUncertainty(float pointSize_, 
						 Eigen::Vector4f color_, 
						 float ellipsoidScale_) : GLParameter() {
    _pointSize = pointSize_;
    _color = color_;
    _ellipsoidScale = ellipsoidScale_;
  }

  void GLParameterUncertainty::applyGLParameter() {
    glColor4f(_color[0], _color[1], _color[2], _color[3]);
    glPointSize(_pointSize);
  }

}
