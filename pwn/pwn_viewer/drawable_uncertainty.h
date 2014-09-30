#pragma once

#include "pwn/gaussian3.h"
#include "gl_parameter_uncertainty.h"
#include "drawable.h"

using namespace pwn;

namespace pwn_viewer {

  class DrawableUncertainty : public Drawable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    DrawableUncertainty();
    DrawableUncertainty(Eigen::Isometry3f transformation_, GLParameter *parameter_, Gaussian3fVector *covariances_);
    virtual ~DrawableUncertainty() { 
      glDeleteLists(_covarianceDrawList, 1); 
      glDeleteLists(_sphereDrawList, 1); 
    }

    virtual GLParameter* parameter() { return _parameter; }
    virtual bool setParameter(GLParameter *parameter_);
    
    virtual Gaussian3fVector* covariances() { return _covariances; }
    virtual void setCovariances(Gaussian3fVector *covariances_) { 
      _covariances = covariances_; 
      updateCovarianceDrawList();
    }

    inline GLuint covarianceDrawList() { return _covarianceDrawList; }    
    inline GLuint sphereDrawList() { return _sphereDrawList; }

    void setStep(int step_) {
      _parameter->setStep(step_);
      updateCovarianceDrawList();
    }
    void setEllipsoidScale(float ellipsoidScale_) {
      _parameter->setEllipsoidScale(ellipsoidScale_);
      updateCovarianceDrawList();
    }

    virtual void draw();
    void updateCovarianceDrawList();

  protected:
    GLParameterUncertainty *_parameter;
    Gaussian3fVector *_covariances;
    GLuint _covarianceDrawList; 
    GLuint _sphereDrawList; 
  };  

}
