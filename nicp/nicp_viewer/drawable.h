#pragma once

#include <Eigen/Geometry>
#include <GL/gl.h>
#include "gl_parameter.h"

namespace nicp_viewer {

  class NICPQGLViewer;

  class Drawable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Drawable();
    Drawable(Eigen::Isometry3f transformation_);
    virtual ~Drawable() {}
  
    virtual void draw() {}

    Eigen::Isometry3f transformation() { return _transformation; }
    virtual void setTransformation(Eigen::Isometry3f transformation_) { _transformation = transformation_; }

    NICPQGLViewer* viewer() { return _viewer; }
    virtual void setViewer(NICPQGLViewer *viewer_) { _viewer = viewer_; }

    virtual GLParameter* parameter() = 0;  
    virtual bool setParameter(GLParameter *parameter_) = 0;

  protected:
    Eigen::Isometry3f _transformation;
    NICPQGLViewer *_viewer;
  };

}
