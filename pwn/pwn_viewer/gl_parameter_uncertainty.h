#pragma once

#include <Eigen/Geometry>
#include <GL/gl.h>
#include "gl_parameter.h"

namespace pwn_viewer {

  class GLParameterUncertainty : public GLParameter {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    GLParameterUncertainty();
    GLParameterUncertainty(float pointSize_, 
			   Eigen::Vector4f color_, 
			   float ellipsoidScale_);
    virtual ~GLParameterUncertainty() {}
  
    virtual void applyGLParameter();

    float pointSize() { return _pointSize; }
    void setPointSize(float pointSize_) { _pointSize = pointSize_; }

    Eigen::Vector4f color() { return _color; }
    void setColor(Eigen::Vector4f color_) { _color = color_; }

    float ellipsoidScale() { return _ellipsoidScale; }
    void setEllipsoidScale(float ellipsoidScale_) { _ellipsoidScale = ellipsoidScale_; }
  
  protected:
    float _pointSize;
    Eigen::Vector4f _color;
    float _ellipsoidScale;
  };

}
