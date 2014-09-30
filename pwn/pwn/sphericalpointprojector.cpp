#include "sphericalpointprojector.h"

#include <stdexcept>

using namespace std;

namespace pwn {
  
  SphericalPointProjector::SphericalPointProjector() : PointProjector() {
    _imageRows = 480;
    _imageCols = 640;
    _horizontalFov = M_PI;
    _verticalFov = M_PI / 4.0f;
    _updateParameters();
  }

  void SphericalPointProjector::_updateParameters() {
    _iT =_transform.inverse();
    _iT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _horizontalResolution = _imageCols / (2.0f * _horizontalFov);
    _inverseHorizontalResolution = 1.0f / _horizontalResolution;
    _verticalResolution = _imageRows / (2.0f * _verticalFov);
    _inverseVerticalResolution = 1.0f / _verticalResolution;
    _horizontalCenter = _imageCols / 2.0f;
    _verticalCenter = _imageRows / 2.0f;
  }

  void  SphericalPointProjector::scale(float scalingFactor) {
    PointProjector::scale(scalingFactor);
    _updateParameters();
  }

  void SphericalPointProjector::project(IntImage &indexImage,
					DepthImage &depthImage, 
					const PointVector &points) const {
    if(_imageRows == 0 ||  _imageCols == 0) { 
      throw "SphericalPointProjector: _imageRows and/or _imageCols are zero";
    }
    indexImage.create(_imageRows, _imageCols);
    depthImage.create(_imageRows, _imageCols);
    depthImage.setTo(cv::Scalar(std::numeric_limits<float>::max()));
    indexImage.setTo(cv::Scalar(-1));    
    float *drowPtrs[_imageRows];
    int *irowPtrs[_imageRows];
    for(int i = 0; i < _imageRows; i++) { 
      drowPtrs[i] = &depthImage(i, 0);
      irowPtrs[i] = &indexImage(i, 0);
    }
    const Point *point = &points[0];
    for(size_t i = 0; i < points.size(); i++, point++) {
      int x, y;
      float d;
      if(!_project(x, y, d, *point) ||
	 d < _minDistance || d > _maxDistance ||
	 x < 0 || x >= indexImage.cols ||
	 y < 0 || y >= indexImage.rows) {
	continue;
      }
      float &otherDistance = drowPtrs[y][x];
      int &otherIndex = irowPtrs[y][x];
      if(!otherDistance || otherDistance > d) {
	otherDistance = d;
	otherIndex = i;
      }
    }
  }

  void SphericalPointProjector::unProject(PointVector &points, 
					  IntImage &indexImage,
					  const DepthImage &depthImage) const {
    if(_imageRows == 0 ||  _imageCols == 0) { 
      throw "SphericalPointProjector: Depth image has zero dimensions";
    }
    points.resize(depthImage.rows * depthImage.cols);
    int count = 0;
    indexImage.create(depthImage.rows, depthImage.cols);
    Point *point = &points[0];
    for(int r = 0; r < depthImage.rows; r++) {
      const float *f = &depthImage(r, 0);
      int *i = &indexImage(r, 0);
      for(int c = 0; c < depthImage.cols; c++, f++, i++) {
	if(!_unProject(*point, c, r, *f)) {
	  *i = -1;
	  continue;
	}
	point++;
	*i = count;
	count++;
      }
    }
    points.resize(count);
  }

  void SphericalPointProjector::unProject(PointVector &points, 
					  Gaussian3fVector &gaussians,
					  IntImage &indexImage,
					  const DepthImage &depthImage) const {
    if(_imageRows == 0 ||  _imageCols == 0) { 
      throw "SphericalPointProjector: Depth image has zero dimensions";
    }
    points.resize(depthImage.rows * depthImage.cols);
    gaussians.resize(depthImage.rows * depthImage.cols);
    indexImage.create(depthImage.rows, depthImage.cols);
    int count = 0;
    Point *point = &points[0];
    Gaussian3f *gaussian = &gaussians[0];
    for(int r = 0; r < depthImage.rows; r++) {
    	const float *f = &depthImage(r, 0);
    	int *i = &indexImage(r, 0);
    	for(int c = 0; c < depthImage.cols; c++, f++, i++) {      
    	  if(!_unProject(*point, c, r, *f)) {
    	    *i = -1;
    	    continue;
    	  }
    	  Eigen::Matrix3f cov = Eigen::Matrix3f::Identity();
    	  *gaussian = Gaussian3f(point->head<3>(), cov);
    	  gaussian++;
    	  point++;
    	  *i = count;
    	  count++;
    	}
    }
    points.resize(count);
    gaussians.resize(count);
  }

  void SphericalPointProjector::projectIntervals(IntervalImage &intervalImage, 
						 const DepthImage &depthImage, 
						 const float worldRadius) const {
    if(_imageRows == 0 ||  _imageCols == 0) { 
      throw "SphericalPointProjector: Depth image has zero dimensions";
    }    
    intervalImage.create(depthImage.rows, depthImage.cols);
    for(int r = 0; r < depthImage.rows; r++){
    	const float *f = &depthImage(r, 0);
	cv::Vec2i *i = &intervalImage(r, 0);
    	for(int c = 0; c < depthImage.cols; c++, f++, i++) {
    	  *i = _projectInterval(r, c, *f, worldRadius);
    	}
    }
  }
}
