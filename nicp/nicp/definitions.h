#pragma once

#include <Eigen/Core>
#include "opencv2/opencv.hpp"
#include "gaussian.h"

namespace nicp {

  /** \typedef Diagonal3f
   * \brief A 3x3 diagonal matrix.
   */
  typedef Eigen::DiagonalMatrix<float, 3> Diagonal3f;

  /** \typedef UnsignedCharImage
   * \brief An unsigned char cv::Mat.
   */
  typedef cv::Mat_<unsigned char> UnsignedCharImage;

  /** \typedef CharImage
   * \brief A char cv::Mat.
   */
  typedef cv::Mat_<char> CharImage;

  /** \typedef UnsignedShortImage
   * \brief An unsigned short cv::Mat.
   */
  typedef cv::Mat_<unsigned short> UnsignedShortImage;

  /** \typedef UnsignedIntImage
   * \brief An unsigned int cv::Mat.
   */
  typedef cv::Mat_<unsigned int> UnsignedIntImage;

  /** \typedef IntImage
   * \brief An int cv::Mat.
   */
  typedef cv::Mat_<int> IntImage;

  /** \typedef IntervalImage
   * \brief An int cv::Mat.
   */
  typedef cv::Mat_<cv::Vec2i> IntervalImage;

  /** \typedef FloatImage
   * \brief A float cv::Mat.
   */
  typedef cv::Mat_<float> FloatImage;

  /** \typedef DoubleImage
   * \brief A double cv::Mat.
   */
  typedef cv::Mat_<double> DoubleImage;

  /** \typedef RawDepthImage
   * \brief An unsigned char cv::Mat used to for depth images with depth values expressed in millimeters.
   */
  typedef UnsignedShortImage RawDepthImage;

  /** \typedef IndexImage
   * \brief An int cv::Mat used to save the indeces of the points of a depth image inside a vector of points.
   */
  typedef IntImage IndexImage;

  /** \typedef DepthImage
   * \brief A float cv::Mat used to for depth images with depth values expressed in meters.
   */
  typedef cv::Mat_< cv::Vec3b > RGBImage;

  typedef std::vector< cv::Vec3b > RGBVector;

  /** \typedef RGBImage
   * \brief A three channel unsigned char matrix for RGB images
   */
  typedef FloatImage DepthImage;

  /** \typedef Gaussian3f
   * \brief A guassian in the 3D dimension.
   */
  typedef struct Gaussian<float, 3> Gaussian3f;

  /**
   * check if an Eigen type contains a nan element
   */
  template <class T>
      bool isNan(const T& m){
      for (int i=0; i< m.rows(); i++) {
          for (int j=0; j< m.cols(); j++) {
              float v = m(i,j);
              if ( std::isnan( v ) )
                  return true;
          }
      }
      return false;
  }

}
