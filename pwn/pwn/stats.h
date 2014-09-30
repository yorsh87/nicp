#pragma once

#include "homogeneousvector4f.h"

namespace pwn {

  /** \struct Stats stats.h "stats.h"
   *  \brief Class to store additional point properties.
   *  
   *  This class provides structures to store additional point properties like 
   *  covariance eigenvector and eignevalues, number of points used for normal computation and so on.
   */
  struct Stats : public Eigen::Matrix4f {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Empty constructor.
     *  This empty constructor creates a Stats with default values for all
     *  its attributes.
     */
    inline Stats() {
      _n = 0;
      setIdentity();
      _eigenValues.setZero();
      _curvatureComputed = false;
      _curvature = 1.0f;	
    }
    
    /**
     *  Destructor.
     */
    virtual ~Stats() {}

    /**
     *  This method returns the number of points used to compute the normal of the point.
     *  @return the number of points used to compute the normal of the point.
     *  @see setN()
     */
    inline int n() { return _n; }

    /**
     *  This method sets the number of points used to compute the normal of the point to the value given in input.
     *  @param n_ is an int value used to update the number of points used to compute the normal of the point.
     *  @see n()
     */
    inline void setN(const int n_) { _n = n_; }

    /**
     *  This method returns the mean of the points used to compute the normal of the point.
     *  @return the mean of the points used to compute the normal of the point.
     *  @see setMean()
     */
    inline Point mean() { return block<4, 1>(0, 3); }

    /**
     *  This method sets the mean of the points used to compute the normal of the point to the value given in input.
     *  @param mean_ is a Point used to update the mean of the points used to compute the normal of the point.
     *  @see mean()
     */
    inline void setMean(const Point mean_) { block<4, 1>(0, 3) = mean_; }

    /**
     *  This method returns the eigenvalues of the covariance matrix of the points used to compute the normal of the point.
     *  @return the eigenvalues of the covariance matrix of the points used to compute the normal of the point.
     *  @see setEigenValues()
     */
    inline const Eigen::Vector3f& eigenValues() const { return _eigenValues; }
    
    /**
     *  This method sets the eigenvalues of the covariance matrix of the points used to compute the normal of 
     *  the point to the vector given in input.
     *  @param eigenValues_ is a vector used to update the eigenvalues of the covariance matrix of the points 
     *  used to compute the normal of the point.
     *  @see eigenValues()
     */
    inline void setEigenValues(const Eigen::Vector3f &eigenValues_)  { _eigenValues = eigenValues_; }

    /**
     *  This method returns the eigenvectors of the covariance matrix of the points used to compute the normal of the point.
     *  @return the eigenvectors of the covariance matrix of the points used to compute the normal of the point.
     *  @see setEigenVectors()
     */
    inline Eigen::Matrix3f eigenVectors() const { return block<3, 3>(0, 0); }
    /**
     *  This method sets the eigenvectora of the covariance matrix of the points used to compute the normal of 
     *  the point to the matrix given in input.
     *  @param eigenVectors_ is a matrix used to update the eigenvectors of the covariance matrix of the points 
     *  used to compute the normal of the point.
     *  @see eigenVectors()
     */
    inline void  setEigenVectors(const Eigen::Matrix3f &eigenVectors_)  { block<3, 3>(0, 0) = eigenVectors_; }

    /**
     *  This method returns the curvature the point.
     *  @return the curvature of the point.
     *  @see setCurvature()
     */
    inline float curvature() const {
      if(!_curvatureComputed)
	_curvature = _eigenValues(0) / (_eigenValues(0) + _eigenValues(1) + _eigenValues(2) + 1e-9);
      _curvatureComputed = true;
      return _curvature;
    }

    /**
     *  This method sets the curvature of the point to the value given in input.
     *  @param curvature_ is a float value used to update the curvature of the point.
     *  @see curvature()
     */
    inline void setCurvature(float curvature_) {
      _curvature = curvature_;
      _curvatureComputed = true;
    }
  
  protected:  	
    int _n; /**< Number of points use dto compute the normal. */
    Eigen::Vector3f _eigenValues;  /**< Eigenvalues associated to the covariance of the point. */
    mutable bool _curvatureComputed; /**< Bool values to know if the curvature was already computed. */
    mutable float _curvature; /**< Curvature of the point. */
  
  };

  class StatsVector : public TransformableVector<Stats> {
  public: 
    template<typename OtherDerived>
      inline void transformInPlace(const OtherDerived &m) {
      const Eigen::Matrix4f R4 = m;
      for (size_t i = 0; i < size(); ++i) {
	at(i).block<4, 4>(0, 0) = R4 * at(i).block<4, 4>(0, 0);
      }
    }
  
  };

}
