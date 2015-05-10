#pragma once

#include "nicp/stats.h"
#include "gl_parameter_covariances.h"
#include "drawable.h"

using namespace nicp;

namespace nicp_viewer {

  class DrawableCovariances : public Drawable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    DrawableCovariances();
    DrawableCovariances(Eigen::Isometry3f transformation_, GLParameter *parameter_, StatsVector *covariances_);
    virtual ~DrawableCovariances() { 
      glDeleteLists(_covarianceDrawList, 1); 
      glDeleteLists(_sphereDrawList, 1); 
    }

    virtual GLParameter* parameter() { return _parameter; }
    virtual bool setParameter(GLParameter *parameter_);
    
    virtual StatsVector* covariances() { return _covariances; }
    virtual void setCovariances(StatsVector *covariances_) { 
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
    GLParameterCovariances *_parameter;
    StatsVector *_covariances;
    GLuint _covarianceDrawList; 
    GLuint _sphereDrawList; 
  };  

}
