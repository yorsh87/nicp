#include "statscalculatorintegralimage.h"

#include <cstdio>

#include <Eigen/Eigenvalues> 

#include "cloud.h"

#include <opencv2/highgui/highgui.hpp>

namespace pwn {
  StatsCalculatorIntegralImage::StatsCalculatorIntegralImage() : StatsCalculator() {
    _worldRadius = 0.1f;
    _maxImageRadius = 30;
    _minImageRadius = 10;
    _minPoints = 50;
    _curvatureThreshold = 0.02f;
  }

  void StatsCalculatorIntegralImage::compute(NormalVector &normals,
					     StatsVector &statsVector,
					     const PointVector &points,
					     const IntImage &indexImage,
					     bool suppressImageRadius) {
    assert(points.size() > 0 && "StatsCalculatorIntegralImage: points has zero size");
    assert(indexImage.rows > 0 && indexImage.cols > 0 && "StatsCalculatorIntegralImage: indexImage has zero size");    
    assert(_intervalImage.rows > 0 && _intervalImage.cols > 0 && "StatsCalculatorIntegralImage: _intervalImage has zero size");
    if(statsVector.size() != points.size())
      statsVector.resize(points.size());
    if(normals.size() != points.size())
      normals.resize(points.size());
    Normal dummyNormal = Normal::Zero();
    std::fill(normals.begin(), normals.end(), dummyNormal);
    std::fill(statsVector.begin(), statsVector.end(), Stats());
    // Computing the integral image
    _integralImage.compute(indexImage, points);    

#pragma omp parallel for
    for(int r = 0; r < indexImage.rows; ++r) {
      const int *index = &indexImage(r, 0);
      const cv::Vec2i *interval = &_intervalImage(r, 0);
      for(int c = 0; c < indexImage.cols; ++c, ++index, ++interval) {
	// is the point valid, is its range valid?
	cv::Vec2i imageRadius = *interval;	
	if(*index < 0 || (*interval)(0) < 0 || (*interval)(1) < 0) { continue; }
	
	if(!suppressImageRadius) {
	  if(imageRadius(0) < _minImageRadius || imageRadius(1) < _minImageRadius) { continue; }
	  if(imageRadius(0) > _maxImageRadius) { imageRadius(0) = _maxImageRadius; }
	  if(imageRadius(1) > _maxImageRadius) { imageRadius(1) = _maxImageRadius; }
	}

	assert(*index < (int)statsVector.size() && "StatsCalculatorIntegralImage: index value greater than statsVector size");

	const PointAccumulator &acc = _integralImage.getRegion(c - imageRadius(0), c + imageRadius(0),
							       r - imageRadius(1), r + imageRadius(1));
	if(acc.n() < _minPoints) { continue; }
	
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver;
	eigenSolver.computeDirect(acc.covariance().block<3, 3>(0, 0), Eigen::ComputeEigenvectors);

	Stats &stats = statsVector[*index];
	Normal &normal = normals[*index];
	const Point &point = points[*index];

	stats.setZero();
	stats.setEigenVectors(eigenSolver.eigenvectors());
	stats.setMean(acc.mean());
	Eigen::Vector3f eigenValues = eigenSolver.eigenvalues();
	if(eigenValues(0) < 0.0f) { eigenValues(0) = 0.0f; }	  
	stats.setEigenValues(eigenValues);
	stats.setN(acc.n());
      
	normal = stats.block<4, 1>(0, 0);
	if(stats.curvature() < _curvatureThreshold) {
	  if(normal.dot(point) > 0) { normal = -normal; }
	} 
	else { normal.setZero(); }      	
      }
    }
  }

}
