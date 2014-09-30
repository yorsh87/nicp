#pragma once

#include "bm_se3.h"

/*
   Base class that implements a simple prior on te transformation to be used in SE2 pose.
   The prior is defined as a gaussian distribution centered in a certain value (priorMean),
   and having a certain information matrix;
   
   The prior has to affect the transform between the current and the treference vertex.
   So it is relative by nature. However we can also enforce absolute priors,
   by remapping a global constraint as a relative one.
   

   Since we are operating on a manifold measurement space, the infromation matrix in the error
   space depends on the linearization point.
   
   To use this class:
   0) derive it and  overwrite the error(...) function in the desired classes to reflect the prior type.
   1) set the priorMean and the priorInformation to the desired values
   2) get the information of the error (omega), through the
      errorInformation(invT) function
   3) get the error and the jacobian to compute H and b;
*/

namespace pwn {

  class SE3Prior {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
    SE3Prior(const Eigen::Isometry3f &priorMean = Eigen::Isometry3f::Identity(), 
	     const Matrix6f &information = Matrix6f::Zero());
    virtual ~SE3Prior() {}

    inline void setMean(const Eigen::Isometry3f &priorMean_) { _priorMean = priorMean_; }
    inline const Eigen::Isometry3f& mean() const { return _priorMean; }

    inline void setInformation(const Matrix6f &priorInformation_) { _priorInformation = priorInformation_; }
    inline const Matrix6f& information() const { return _priorInformation; }

    // Computes the error of the prior at the inverse transform
    virtual Vector6f error(const Eigen::Isometry3f &invT) const = 0;

    // Computes the jacobian of the error 
    Matrix6f jacobian(const Eigen::Isometry3f &invT) const;

    // Projects the information matrix of the prior in the error space
    Matrix6f errorInformation(const Eigen::Isometry3f &invT) const;

  protected:
    // Computes the jacobian of the error w.r.t. the priorMean
    // Used internally to project the information matrix in the measurement space
    Matrix6f jacobianZ(const Eigen::Isometry3f &invT) const;
  
    mutable Eigen::Isometry3f  _priorMean;
    Matrix6f _priorInformation;
  };

  /*
     The error induced by the prior is defined as 
     t2v(invT * priorMean),
     where invT s the *inverse* of the transformation we are looking for.
  */

  class SE3RelativePrior: public SE3Prior{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    SE3RelativePrior(const Eigen::Isometry3f &priorMean = Eigen::Isometry3f::Identity(), 
		     const Matrix6f &information = Matrix6f::Zero());
    virtual ~SE3RelativePrior() {}

    // Computes the error of the prior at the inverse transform
    virtual Vector6f error(const Eigen::Isometry3f &invT) const;
  };


  /*
   
     The error induced by the prior is defined as 
     t2v(invT * _referenceTransform.inverse()*priorMean),
     where invT s the *inverse* of the transformation we are looking for.
  */

  class SE3AbsolutePrior : public SE3Prior{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    SE3AbsolutePrior(const Eigen::Isometry3f &referenceTransform_ = Eigen::Isometry3f::Identity(), 
		     const Eigen::Isometry3f &priorMean = Eigen::Isometry3f::Identity(), 
		     const Matrix6f &information = Matrix6f::Zero());
    virtual ~SE3AbsolutePrior() {}

    // Computes the error of the prior at the inverse transform
    virtual Vector6f error(const Eigen::Isometry3f &invT) const;

    inline const Eigen::Isometry3f& referenceTransform() const { return _referenceTransform; }
    inline void setReferenceTransform(const Eigen::Isometry3f &referenceTransform_)  {
      _referenceTransform = referenceTransform_;
      _inverseReferenceTransform = referenceTransform_.inverse();
    }

  protected:
    Eigen::Isometry3f _referenceTransform;
    Eigen::Isometry3f _inverseReferenceTransform;
  };

}
