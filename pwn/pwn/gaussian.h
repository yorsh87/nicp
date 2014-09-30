#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Dense>

template <typename Scalar_, int DimensionAtCompileTime_>
struct Gaussian {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef Scalar_ Scalar;
  typedef typename Eigen::Matrix<Scalar, DimensionAtCompileTime_, DimensionAtCompileTime_> MatrixType;
  typedef typename Eigen::Matrix<Scalar, DimensionAtCompileTime_, 1> VectorType;
  static const int DimensionAtCompileTime = DimensionAtCompileTime_;

  Gaussian() {
    _covarianceMatrix.setZero();
    _informationMatrix.setZero();
    _informationVector.setZero();
    _mean.setZero();
  }

  Gaussian(int dimension_): 
    _informationMatrix(dimension_, dimension_),
    _informationVector(dimension_),
    _covarianceMatrix(dimension_, dimension_),
    _mean(dimension_)
  {
    _covarianceMatrix.setZero();
    _informationMatrix.setZero();
    _informationVector.setZero();
    _mean.setZero();
  }

  
  Gaussian(const VectorType& v, const MatrixType& m, bool useInfoForm = false) {
    _momentsUpdated = _infoUpdated = false; 
    if (useInfoForm){
      _informationVector=v;
      _informationMatrix = m;
      _infoUpdated = true;
    } else {
      _mean=v;
      _covarianceMatrix = m;
      _momentsUpdated = true;
    }
  }

  inline Gaussian<Scalar_,DimensionAtCompileTime_>& addInformation(const Gaussian<Scalar_,DimensionAtCompileTime_>& g){
    _updateInfo();
    _informationMatrix += g.informationMatrix();
    _informationVector += g.informationVector();
    _momentsUpdated = false;
    return *this;
  }

  inline Gaussian<Scalar_,DimensionAtCompileTime_>& addNoise(const Gaussian<Scalar_,DimensionAtCompileTime_>& g){
    _updateMoments();
    _covarianceMatrix += g.covarianceMatrix();
    _mean += g.mean();
    _infoUpdated = false;
    return *this;
  }


  inline int dimension() const {return _mean.rows();}
  inline const Eigen::Matrix3f& covarianceMatrix() const {_updateMoments(); return _covarianceMatrix; }
  inline const Eigen::Vector3f& mean() const {_updateMoments(); return _mean; }
  inline const Eigen::Matrix3f& informationMatrix() const { _updateInfo(); return _informationMatrix; }
  inline const Eigen::Vector3f& informationVector() const { _updateInfo();return _informationVector; }

  
protected:
  inline void _updateMoments() const {
    if (_momentsUpdated)
      return;
    _covarianceMatrix = _informationMatrix.inverse();
    _mean = _covarianceMatrix*_informationVector;
    _momentsUpdated = true;
  }

  inline void _updateInfo() const {
    if (_infoUpdated)
      return;
    _informationMatrix = _covarianceMatrix.inverse();
    _informationVector = _informationMatrix*_mean;
    _infoUpdated = true;
  }

  mutable MatrixType _informationMatrix;
  mutable VectorType _informationVector;
  mutable MatrixType _covarianceMatrix;
  mutable VectorType _mean;
  mutable bool _momentsUpdated;
  mutable bool _infoUpdated;
};
