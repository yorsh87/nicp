#pragma once

#include <Eigen/Geometry>
#include <GL/gl.h>
#include "gl_parameter.h"

namespace pwn_viewer {

  class GLParameterCorrespondences : public GLParameter {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    GLParameterCorrespondences();
    GLParameterCorrespondences(float pointSize_, Eigen::Vector4f color_, float lineWidth_);
    virtual ~GLParameterCorrespondences() {}

    virtual void applyGLParameter();

    float pointSize() { return _pointSize; }
    void setPointSize(float pointSize_) { _pointSize = pointSize_; }
    
    Eigen::Vector4f color() { return _color; }
    void setColor(Eigen::Vector4f color_) { _color = color_; }
    
    float lineWidth() { return _lineWidth; }
    void setLineWidth(float lineWidth_) { _lineWidth = lineWidth_; }

  protected:
    float _pointSize;
    Eigen::Vector4f _color;
    float _lineWidth;
  };

}
