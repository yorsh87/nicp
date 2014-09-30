#include "gl_parameter_trajectory.h"

namespace pwn_viewer {

  GLParameterTrajectory::GLParameterTrajectory() : GLParameter() {
    _pyramidScale = 1.0f;
    _color = Eigen::Vector4f(1.0f, 1.0f, 0.0f, 0.5f);
  }

  GLParameterTrajectory::GLParameterTrajectory(float pyramidScale_, const Eigen::Vector4f color_) : GLParameter() {
    _pyramidScale = pyramidScale_;
    _color = color_;
  }

}
