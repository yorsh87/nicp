#pragma once

#include <Eigen/Geometry>
#include <GL/gl.h>
#include "gl_parameter.h"

namespace pwn_viewer {

  class GLParameterPoints : public GLParameter {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    GLParameterPoints();
    GLParameterPoints(float pointSize_, const Eigen::Vector4f color_);
    virtual ~GLParameterPoints() {}

    virtual void applyGLParameter();

    float pointSize() { return _pointSize; }
    void setPointSize(float pointSize_) { _pointSize = pointSize_; }
    
    Eigen::Vector4f color() { return _color; }
    void setColor(Eigen::Vector4f color_) { _color = color_; }

  protected:
    float _pointSize;
    Eigen::Vector4f _color;
  };

}
