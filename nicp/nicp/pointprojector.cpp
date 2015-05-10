#include "pointprojector.h"
#include "bm_se3.h"

namespace nicp {

  PointProjector::PointProjector() {
    _transform.setIdentity();
    _transform.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _minDistance = 0.01f;
    _maxDistance = 6.0f;
    _originalImageRows = 0;
    _originalImageCols = 0;
    _imageRows = 0;
    _imageCols = 0;
    _scalingFactor = 1.0f;
  }

  void PointProjector::project(IntImage &indexImage, 
			       DepthImage &depthImage, 
			       const PointVector &points) const {
    assert(indexImage.rows > 0 && indexImage.cols > 0 && "PointProjector: Index image has zero dimensions");

    depthImage.create(indexImage.rows, indexImage.cols);
    depthImage.setTo(0.0f);
    indexImage.setTo(cv::Scalar(-1));
    const Point *point = &points[0];
    for(size_t i = 0; i < points.size(); i++, point++) {
      int x, y;
      float d;
      if(!project(x, y, d, *point) ||
	 x < 0 || x >= indexImage.rows ||
	 y < 0 || y >= indexImage.cols) { continue; }
      float &otherDistance = depthImage(x, y);
      int &otherIndex = indexImage(x, y);
      if(!otherDistance || otherDistance > d) {
	otherDistance = d;
	otherIndex = i;
      }
    }
  }

  void PointProjector::unProject(PointVector &points,
				 IntImage &indexImage,
				 const DepthImage &depthImage) const {
    assert(depthImage.rows > 0 && depthImage.cols > 0 && "PointProjector: Depth image has zero dimensions");
    points.resize(depthImage.rows * depthImage.cols);
    int count = 0;
    indexImage.create(depthImage.rows, depthImage.cols);
    Point *point = &points[0];
    for(int r = 0; r < depthImage.rows; r++){
      const float* f = &depthImage(r, 0);
      int *i = &indexImage(r, 0);
      for(int c = 0; c < depthImage.cols; c++, f++, i++) {
	if(!unProject(*point, r, c, *f)) {
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

  void PointProjector::unProject(PointVector &points,
				 Gaussian3fVector &gaussians,
				 IntImage &indexImage,
				 const DepthImage &depthImage) const {
    assert(depthImage.rows > 0 && depthImage.cols > 0 && "PointProjector: Depth image has zero dimensions");
    points.resize(depthImage.rows * depthImage.cols);
    int count = 0;
    indexImage.create(depthImage.rows, depthImage.cols);
    Point *point = &points[0];
    for(int r = 0; r < depthImage.rows; r++) {
      const float *f = &depthImage(r, 0);
      int *i = &indexImage(r, 0);
      for(int c = 0; c < depthImage.cols; c++, f++, i++) {
	if(!unProject(*point, r, c, *f)) {
	  *i = -1;
	  continue;
	}
	point++;
	*i = count;
	count++;
      }
    }

    gaussians.resize(count);
    points.resize(count);
  }
  
  void PointProjector::projectIntervals(IntervalImage &intervalImage, 
					const DepthImage &depthImage, 
					const float worldRadius) const {
    assert(depthImage.rows > 0 && depthImage.cols > 0 && "PointProjector: Depth image has zero dimensions");
    intervalImage.create(depthImage.rows, depthImage.cols);
    for(int r = 0; r < depthImage.rows; r++) {
      const float *f = &depthImage(r, 0);
      cv::Vec2i *i = &intervalImage(r, 0);
      for(int c = 0; c < depthImage.cols; c++, f++, i++) { *i = projectInterval(r, c, *f, worldRadius); }
    }
  }
 
  void PointProjector::scale(float scalingFactor) {
    _scalingFactor = scalingFactor; 
    _imageRows = _originalImageRows;
    _imageCols = _originalImageCols;
    _imageRows *= scalingFactor;
    _imageCols *= scalingFactor;
  }

}
