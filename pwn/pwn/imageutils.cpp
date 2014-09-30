#include "imageutils.h"
#include <iostream>

namespace pwn {

  void DepthImage_scale(DepthImage &dest, const DepthImage &src, int step, float maxDepthCov) {
    int rows = src.rows / step;
    int cols = src.cols / step;
    dest.create(rows, cols);
    dest.setTo(cv::Scalar(0));
    for(int r = 0; r < dest.rows; r++) {
      for(int c = 0; c < dest.cols; c++) {
	float acc = 0;
	float acc2 = 0;
	int np = 0;
	int sr = r * step;
	int sc = c * step;
	for(int i = 0; i < step; i++) {
	  for(int j = 0; j < step; j++) {
	    if(sr + i < src.rows && sc + j < src.cols) {
	      const float& f  = src(sr + i, sc + j);
	      acc += f;
	      acc2 += f*f;
	      np += src(sr + i, sc + j) > 0;
	    }
	  }
	}
	if(np){
	  float mu = acc/np;
	  float sigma = acc2/np-mu*mu;
	  if (sigma>maxDepthCov)
	    continue;
	  dest(r, c) = mu;
	}
      }
    }
  }

  void RGBImage_scale(RGBImage &dest, const RGBImage &src, int step) {
    int rows = src.rows / step;
    int cols = src.cols / step;
    dest.create(rows, cols);
    dest.setTo(cv::Scalar(0));
    for(int r = 0; r < dest.rows; r++) {
      for(int c = 0; c < dest.cols; c++) {
	cv::Vec3i acc(0,0,0);
	int np = 0;
	int sr = r * step;
	int sc = c * step;
	for(int i = 0; i < step; i++) {
	  for(int j = 0; j < step; j++) {
	    if(sr + i < src.rows && sc + j < src.cols) {
	      const cv::Vec3b& p = src(sr + i, sc + j);
	      cv::Vec3i ip(p[0], p[1], p[2]);
	      acc += ip;
	      np++;
	    }
	  }
	}
	if(np){
	  float inp=1./np;
	  cv::Vec3b mu(acc[0]*inp, acc[1]*inp, acc[2]*inp);
	  dest(r, c) = mu;
	}
      }
    }
  }

  void DepthImage_convert_32FC1_to_16UC1(cv::Mat &dest, const cv::Mat &src, float scale) {
    assert(src.type() != CV_32FC1 && "DepthImage_convert_32FC1_to_16UC1: source image of different type from 32FC1");
    const float *sptr = (const float*)src.data;
    int size = src.rows * src.cols;
    const float *send = sptr + size;
    dest.create(src.rows, src.cols, CV_16UC1);
    dest.setTo(cv::Scalar(0));
    unsigned short *dptr = (unsigned short*)dest.data;
    while(sptr<send) {
      if(*sptr < std::numeric_limits<float>::max())
	*dptr = scale * (*sptr);
      dptr ++;
      sptr ++;
    }
  }

  void DepthImage_convert_16UC1_to_32FC1(cv::Mat &dest, const cv::Mat &src, float scale) {
    assert(src.type() != CV_16UC1 && "DepthImage_convert_16UC1_to_32FC1: source image of different type from 16UC1");
    const unsigned short *sptr = (const unsigned short*)src.data;
    int size = src.rows * src.cols;
    const unsigned short *send = sptr + size;
    dest.create(src.rows, src.cols, CV_32FC1);
    dest.setTo(cv::Scalar(0.0f));
    float *dptr = (float*)dest.data;
    while(sptr < send) {
      if(*sptr)
	*dptr = scale * (*sptr);
      dptr ++;
      sptr ++;
    }
  }  

}
