#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>

namespace pwn {

  typedef Eigen::DiagonalMatrix<float, 3, 3> Diagonal3f;
  typedef Eigen::Matrix<float, 6, 1> Vector6f;
  typedef Eigen::Matrix<float, 6, 6> Matrix6f;
  typedef Eigen::Matrix<float, 2, 4>  Matrix2x4f;

  typedef Eigen::Matrix<float, 12, 12> Matrix12f;
  typedef Eigen::Matrix<float, 6, 12>  Matrix6x12f;
  typedef Eigen::Matrix<float, 12, 1> Vector12f;


  typedef Eigen::DiagonalMatrix<double, 3, 3> Diagonal3d;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  typedef Eigen::Matrix<double, 2, 4>  Matrix2x4d;

  typedef Eigen::Matrix<double, 12, 12> Matrix12d;
  typedef Eigen::Matrix<double, 6, 12>  Matrix6x12d;
  typedef Eigen::Matrix<double, 12, 1> Vector12d;


  typedef Eigen::Matrix<unsigned short, Eigen::Dynamic, Eigen::Dynamic> MatrixXus;

}
