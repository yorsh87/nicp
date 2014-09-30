#include "se3_prior.h"

namespace pwn {

  SE3Prior::SE3Prior(const Eigen::Isometry3f &priorMean_, const Matrix6f &priorInformation_) :
    _priorMean(priorMean_), _priorInformation(priorInformation_) {}

  Matrix6f SE3Prior::jacobian(const Eigen::Isometry3f &invT) const {
    Matrix6f J;
    float epsilon = 1e-3;
    float iEpsilon = 0.5f / epsilon;
    Vector6f incrementsUp = Vector6f::Zero();
    Vector6f incrementsDown = Vector6f::Zero();
    
    for(int i = 0; i < J.cols(); i++) {
      incrementsUp(i) = epsilon;
      incrementsDown(i) = -epsilon;
      J.col(i) = iEpsilon * (error(v2t(incrementsUp) * invT) - error(v2t(incrementsDown) * invT));
      incrementsUp(i) = 0;
      incrementsDown(i) = 0;
    }
    return J;
  }

  Matrix6f SE3Prior::jacobianZ(const Eigen::Isometry3f &invT) const {
    Matrix6f J;
    float epsilon = 1e-3;
    float iEpsilon = 0.5f / epsilon;
    Vector6f incrementsUp = Vector6f::Zero();
    Vector6f incrementsDown = Vector6f::Zero();
    Eigen::Isometry3f savedPrior = _priorMean;
    for(int i = 0; i < J.cols(); i++) {
      incrementsUp(i) = epsilon;
      _priorMean = savedPrior * v2t(incrementsUp);
      Vector6f eUp = error(invT);
    
      incrementsDown(i) = -epsilon;
      _priorMean = savedPrior * v2t(incrementsDown);
      Vector6f eDown = error(invT);
      J.col(i) = iEpsilon * (eUp - eDown);
      incrementsUp(i) = 0;
      incrementsDown(i) = 0;
    }
    _priorMean = savedPrior;
    return J;
  }

  Matrix6f SE3Prior::errorInformation(const Eigen::Isometry3f &invT) const {
    Matrix6f Jz = jacobianZ(invT);
    Matrix6f iJz = Jz.inverse();
    return iJz.transpose() * _priorInformation * iJz;
  }

  SE3RelativePrior::SE3RelativePrior(const Eigen::Isometry3f &priorMean_, 
				     const Matrix6f &information_): 
    SE3Prior(priorMean_, information_) {}

  Vector6f SE3RelativePrior::error(const Eigen::Isometry3f &invT) const {
    return t2v(invT * _priorMean);
  }

  SE3AbsolutePrior::SE3AbsolutePrior(const Eigen::Isometry3f &referenceTransform_,
				     const Eigen::Isometry3f &priorMean_, 
				     const Matrix6f &information_):
    SE3Prior(priorMean_, information_) {
    setReferenceTransform(referenceTransform_);
  }

  Vector6f SE3AbsolutePrior::error(const Eigen::Isometry3f &invT) const {
    //Vector6f r;
    //r.setZero();
    //return r;
    return t2v(invT * _inverseReferenceTransform * _priorMean);
  }

}
