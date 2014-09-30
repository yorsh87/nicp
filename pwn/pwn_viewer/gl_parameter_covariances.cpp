#include "gl_parameter_covariances.h"

namespace pwn_viewer {

  GLParameterCovariances::GLParameterCovariances() : GLParameter() {
    _pointSize = 1.0f;
    _colorLowCurvature = Eigen::Vector4f(0.0f, 1.0f, 0.0f, 0.5f);
    _colorHighCurvature = Eigen::Vector4f(1.0f, 0.0f, 0.0f, 0.5f);
    _curvatureThreshold = 0.02f;
    _ellipsoidScale = 0.05f;
  }

  GLParameterCovariances::GLParameterCovariances(float pointSize_, 
						 Eigen::Vector4f colorLowCurvature_, Eigen::Vector4f colorHighCurvature_, 
						 float curvatureThreshold_, float ellipsoidScale_) : GLParameter() {
    _pointSize = pointSize_;
    _colorLowCurvature = colorLowCurvature_;
    _colorHighCurvature = colorHighCurvature_;
    _curvatureThreshold = curvatureThreshold_;
    _ellipsoidScale = ellipsoidScale_;
  }

  void GLParameterCovariances::applyGLParameter() {
    glPointSize(_pointSize);
  }

}
