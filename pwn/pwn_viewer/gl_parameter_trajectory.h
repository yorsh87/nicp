#pragma once

#include <Eigen/Geometry>
#include <GL/gl.h>
#include "gl_parameter.h"

namespace pwn_viewer {

  class GLParameterTrajectory : public GLParameter {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    GLParameterTrajectory();
    GLParameterTrajectory(float pyramidScale_, const Eigen::Vector4f color_);
    virtual ~GLParameterTrajectory() {}

    virtual void applyGLParameter() { glColor4f(_color[0], _color[1], _color[2], _color[3]); }

    float pyramidScale() { return _pyramidScale; }
    void setPyramidScale(float pyramidScale_) { _pyramidScale = pyramidScale_; }

    Eigen::Vector4f color() { return _color; }
    void setColor(Eigen::Vector4f color_) { _color = color_; }

  protected:
    float _pyramidScale;
    Eigen::Vector4f _color;
  };

}
