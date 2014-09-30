#pragma once

#include "pwn/homogeneousvector4f.h"
#include "gl_parameter_normals.h"
#include "drawable.h"

using namespace pwn;

namespace pwn_viewer {

  class DrawableNormals : public Drawable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    DrawableNormals();
    DrawableNormals(const Eigen::Isometry3f &transformation_, GLParameter *parameter_, PointVector *points_, NormalVector *normals_);
    virtual ~DrawableNormals() { glDeleteLists(_normalDrawList, 1); }

    virtual GLParameter* parameter() { return _parameter; };
    virtual bool setParameter(GLParameter *parameter_);

    virtual PointVector* points() { return _points; }
    virtual void setPoints(PointVector *points_) { 
      _points = points_; 
      updateNormalDrawList();
    }

    virtual NormalVector* normals() { return _normals; }    
    virtual void setNormals(NormalVector *normals_) { 
      _normals = normals_; 
      updateNormalDrawList();
    }

    void setStep(int step_) {
      _parameter->setStep(step_);
      updateNormalDrawList();
    }
    void setNormalLength(float normalLength_) {
      _parameter->setNormalLength(normalLength_);
      updateNormalDrawList();
    }
    void setNormalWidth(float lineWidth_) {
      _parameter->setLineWidth(lineWidth_);
      updateNormalDrawList();
    }

    inline GLuint normalDrawList() { return _normalDrawList; }

    virtual void draw();
    void updateNormalDrawList();

  protected:
    GLParameterNormals *_parameter;
    PointVector *_points;
    NormalVector *_normals;
    GLuint _normalDrawList; 
  };  

}
