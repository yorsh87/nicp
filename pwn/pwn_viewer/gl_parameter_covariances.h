#pragma once

#include <Eigen/Geometry>
#include <GL/gl.h>
#include "gl_parameter.h"

namespace pwn_viewer {

  class GLParameterCovariances : public GLParameter {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    GLParameterCovariances();
    GLParameterCovariances(float pointSize_, 
			   Eigen::Vector4f colorLowCurvature_, Eigen::Vector4f colorHighCurvature_, 
			   float curvatureThreshold_, float ellipsoidScale_);
    virtual ~GLParameterCovariances() {}
  
    virtual void applyGLParameter();

    float pointSize() { return _pointSize; }
    void setPointSize(float pointSize_) { _pointSize = pointSize_; }

    Eigen::Vector4f colorLowCurvature() { return _colorLowCurvature; }
    void setColorLowCurvature(Eigen::Vector4f colorLowCurvature_) { _colorLowCurvature = colorLowCurvature_; }

    Eigen::Vector4f colorHighCurvature() { return _colorHighCurvature; }
    void setColorHighCurvature(Eigen::Vector4f colorHighCurvature_) { _colorHighCurvature = colorHighCurvature_; }

    float curvatureThreshold() { return _curvatureThreshold; }
    void setCurvatureThreshold(float curvatureThreshold_) { _curvatureThreshold = curvatureThreshold_; }

    float ellipsoidScale() { return _ellipsoidScale; }
    void setEllipsoidScale(float ellipsoidScale_) { _ellipsoidScale = ellipsoidScale_; }
  
  protected:
    float _pointSize;
    Eigen::Vector4f _colorLowCurvature;
    Eigen::Vector4f _colorHighCurvature;
    float _curvatureThreshold;
    float _ellipsoidScale;
  };

}
