#pragma once

#include <Eigen/Geometry>
#include <GL/gl.h>
#include "gl_parameter.h"

namespace pwn_viewer {

  class PWNQGLViewer;

  class Drawable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Drawable();
    Drawable(Eigen::Isometry3f transformation_);
    virtual ~Drawable() {}
  
    virtual void draw() {}

    Eigen::Isometry3f transformation() { return _transformation; }
    virtual void setTransformation(Eigen::Isometry3f transformation_) { _transformation = transformation_; }

    PWNQGLViewer* viewer() { return _viewer; }
    virtual void setViewer(PWNQGLViewer *viewer_) { _viewer = viewer_; }

    virtual GLParameter* parameter() = 0;  
    virtual bool setParameter(GLParameter *parameter_) = 0;

  protected:
    Eigen::Isometry3f _transformation;
    PWNQGLViewer *_viewer;
  };

}
