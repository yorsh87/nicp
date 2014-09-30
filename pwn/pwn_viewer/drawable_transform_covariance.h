#pragma once

#include "gl_parameter_transform_covariance.h"
#include "drawable.h"

namespace pwn_viewer {

  class DrawableTransformCovariance : public Drawable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    DrawableTransformCovariance();
    DrawableTransformCovariance(Eigen::Isometry3f transformation_, GLParameter *parameter_, Eigen::Matrix3f covariance_, Eigen::Vector3f mean_);
    virtual ~DrawableTransformCovariance() { 
      glDeleteLists(_covarianceDrawList, 1); 
      glDeleteLists(_sphereDrawList, 1); 
    }

    virtual GLParameter* parameter() { return _parameter; }
    virtual bool setParameter(GLParameter *parameter_);

    virtual Eigen::Matrix3f covariance() { return _covariance; }
    virtual void setCovariances(Eigen::Matrix3f covariance_) { 
      _covariance = covariance_; 
      updateCovarianceDrawList();
    }

    virtual Eigen::Vector3f mean() { return _mean; }
    virtual void setMean(Eigen::Vector3f mean_) { 
      _mean = mean_; 
      updateCovarianceDrawList();
    }

    inline GLuint covarianceDrawList() { return _covarianceDrawList; }
    inline GLuint sphereDrawList() { return _sphereDrawList; }

    virtual void draw();
    void updateCovarianceDrawList();

  protected:
    GLParameterTransformCovariance *_parameter;
    Eigen::Matrix3f _covariance;
    Eigen::Vector3f _mean;
    GLuint _covarianceDrawList; 
    GLuint _sphereDrawList; 
  };  

}
