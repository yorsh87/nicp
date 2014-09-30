#pragma once

#include "definitions.h"

namespace pwn {
  
  /**
   *  This method scales a depth image to the size specified.
   *  @param dest is where the resized depth image will be saved.
   *  @param src is the source depth image to resize.
   *  @param step is the resize factor. If step is greater than 1 the image will be smaller
   *  (for example in case it's 2 the size of the image will be half the size of the original one).
   */
  void DepthImage_scale(DepthImage &dest, const DepthImage &src, int step, float maxDepthCov = 0.01f);

  /**
   *  This method scales a depth image to the size specified.
   *  @param dest is where the resized depth image will be saved.
   *  @param src is the source depth image to resize.
   *  @param step is the resize factor. If step is greater than 1 the image will be smaller
   *  (for example in case it's 2 the size of the image will be half the size of the original one).
   */
  void RGBImage_scale(RGBImage &dest, const RGBImage &src, int step);


  /**
   *  This method converts a float cv::Mat to an unsigned char cv::Mat.
   *  @param dest is where the converted image will be saved.
   *  @param src is the source image to convert.
   *  @param scale is a parameter that for example in the case of a depth image lets to convert
   *  the depth values from a unit measure to another. Here it is assumed by default that the elements have
   *  to be converted from millimeters to meters and so the scale is 1000.
   */
  void DepthImage_convert_32FC1_to_16UC1(cv::Mat &dest, const cv::Mat &src, float scale = 1000.0f);

  /**
   *  This method converts an unsigned char cv::Mat to a float cv::Mat.
   *  @param dest is where the converted image will be saved.
   *  @param src is the source image to convert.
   *  @param scale is a parameter that for example in the case of a depth image lets to convert
   *  the depth values from a unit measure to another. Here it is assumed by default that the elements have
   *  to be converted from meters to millimeters and so the scale is 0.001.
   */
  void DepthImage_convert_16UC1_to_32FC1(cv::Mat &dest, const cv::Mat &src, float scale = 0.001f);

}
