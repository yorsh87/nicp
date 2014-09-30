#pragma once

#include <Eigen/Geometry>
#include <GL/gl.h>
#include "gl_parameter.h"

namespace pwn_viewer {

  class GLParameterTransformCovariance : public GLParameter{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    GLParameterTransformCovariance();
    GLParameterTransformCovariance(Eigen::Vector4f color_, float scale_);
    virtual ~GLParameterTransformCovariance() {}
  
    virtual void applyGLParameter();

    Eigen::Vector4f color() { return _color; }
    void setColor(Eigen::Vector4f color_) { _color = color_; }

    float scale() { return _scale; }  
    void setScale(float scale_) { _scale = scale_; }
  
  protected:
    Eigen::Vector4f _color;
    float _scale;
  };

}
