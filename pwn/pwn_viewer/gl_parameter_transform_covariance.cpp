#include "gl_parameter_transform_covariance.h"

namespace pwn_viewer {

  GLParameterTransformCovariance::GLParameterTransformCovariance() : GLParameter() {
    _color = Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f);
    _scale = 0.05f;
  }

  GLParameterTransformCovariance::GLParameterTransformCovariance(Eigen::Vector4f color_, float scale_) : GLParameter() {
    _color = color_;
    _scale = scale_;
  }

  void GLParameterTransformCovariance::applyGLParameter() {}

}
