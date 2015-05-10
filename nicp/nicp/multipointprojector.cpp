#include "multipointprojector.h"

#include "bm_se3.h"

namespace nicp {

  void MultiPointProjector::computeImageSize(int &rows, int &cols) const {
    // Compute total image size
    rows = 0;
    cols = 0;
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      if(_pointProjectors[i].pointProjector->imageRows() > rows) {
	rows = _pointProjectors[i].pointProjector->imageRows();
      }
      cols += _pointProjectors[i].pointProjector->imageCols();
    }
  }

  void MultiPointProjector::project(IntImage &indexImage, 
				    DepthImage &depthImage, 
				    const PointVector &points) {

    assert(_imageRows && _imageCols && "MultiPointProjector: _imageRows and _imageCols are zero");

    // Compute total image size
    int maxWidth = 0;
    int totalHeight = 0;
    computeImageSize(maxWidth, totalHeight);
    setImageSize(maxWidth, totalHeight);
  
    // Resize the output images
    indexImage.create(maxWidth, totalHeight);
    depthImage.create(maxWidth, totalHeight);
    depthImage.setTo(std::numeric_limits<float>::max());
    indexImage.setTo(-1);
  
    int columnOffset = 0;
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      ChildProjectorInfo &childInfo = _pointProjectors[i];
      const int currentHeight = childInfo.pointProjector->imageCols();
      PointProjector *currentPointProjector = childInfo.pointProjector;
      IntImage &currentIndexImage = childInfo.indexImage;
      DepthImage &currentDepthImage = childInfo.depthImage;
      if(currentPointProjector != 0) {
	currentPointProjector->project(currentIndexImage, currentDepthImage, points);      
      
	for(int c = 0; c < currentIndexImage.cols; c++) {
	  for(int r = 0; r < currentIndexImage.rows; r++) {
	    int i = currentIndexImage(r, c);
	    indexImage(r, c + columnOffset) = i;
	    depthImage(r, c + columnOffset) = currentDepthImage(r, c);	  
	  }
	}
      }
      columnOffset += currentHeight;
    }
  }

  void MultiPointProjector::unProject(PointVector &points,
				      IntImage &indexImage, 
				      const DepthImage &depthImage) const {
    assert(depthImage.rows > 0 && depthImage.cols > 0 && "MultiPointProjector: Depth image has zero dimensions");

    indexImage.create(depthImage.rows, depthImage.cols);
    indexImage.setTo(-1);
    points.clear();
    PointVector currentPoints;
    int columnOffset = 0;
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      const int width = _pointProjectors[i].pointProjector->imageRows();
      const int height = _pointProjectors[i].pointProjector->imageCols();

      PointProjector *currentPointProjector = _pointProjectors[i].pointProjector;
      if(currentPointProjector != 0) {
	_pointProjectors[i].depthImage = depthImage(cv::Rect(0, columnOffset, width, height));
	currentPointProjector->unProject(currentPoints,
					 _pointProjectors[i].indexImage, 
					 _pointProjectors[i].depthImage);
	for(int r = 0; r < _pointProjectors[i].indexImage.rows; r++) {
	  for(int c = 0; c < _pointProjectors[i].indexImage.cols; c++) {
	    if(_pointProjectors[i].indexImage(r, c) != -1)
	      _pointProjectors[i].indexImage(r, c) += points.size();
	  }
	}
	indexImage(cv::Rect(0, columnOffset, width, height)) = _pointProjectors[i].indexImage;
	points.insert(points.end(), currentPoints.begin(), currentPoints.end());
      }
      columnOffset += height;
    }
  }

  void MultiPointProjector::unProject(PointVector &points,
				      Gaussian3fVector &gaussians,
				      IntImage &indexImage,
				      const DepthImage &depthImage) const {
    assert(depthImage.rows > 0 && depthImage.cols > 0 && "MultiPointProjector: Depth image has zero dimensions");

    indexImage.create(depthImage.rows, depthImage.cols);
    indexImage.setTo(-1);
    points.clear();
    gaussians.clear();
    PointVector currentPoints;
    Gaussian3fVector currentGaussians;
    int columnOffset = 0;
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      const int width = _pointProjectors[i].pointProjector->imageRows();
      const int height = _pointProjectors[i].pointProjector->imageCols();

      PointProjector *currentPointProjector = _pointProjectors[i].pointProjector;
      if(currentPointProjector != 0) {
	_pointProjectors[i].depthImage = depthImage(cv::Rect(0, columnOffset, width, height));
	currentPointProjector->unProject(currentPoints,
					 currentGaussians,
					 _pointProjectors[i].indexImage, 
					 _pointProjectors[i].depthImage);
	for(int r = 0; r < _pointProjectors[i].indexImage.rows; r++) {
	  for(int c = 0; c < _pointProjectors[i].indexImage.cols; c++) {
	    if(_pointProjectors[i].indexImage(r, c) != -1)
	      _pointProjectors[i].indexImage(r, c) += points.size();
	  }
	}
	indexImage(cv::Rect(0, columnOffset, width, height)) = _pointProjectors[i].indexImage;
	points.insert(points.end(), currentPoints.begin(), currentPoints.end());
	gaussians.insert(gaussians.end(), currentGaussians.begin(), currentGaussians.end());
      }
      columnOffset += height;
    }
  }

  void MultiPointProjector::projectIntervals(IntervalImage &intervalImage, 
					     const DepthImage &depthImage, 
					     const float worldRadius) const {
    assert(depthImage.rows > 0 && depthImage.cols > 0 && "MultiPointProjector: Depth image has zero dimensions");

    intervalImage.create(depthImage.rows, depthImage.cols);
    IntervalImage currentIntervalImage;
    int columnOffset = 0;
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      const int width = _pointProjectors[i].pointProjector->imageRows();
      const int height = _pointProjectors[i].pointProjector->imageCols();

      if(currentIntervalImage.rows != width || currentIntervalImage.cols != height)
	currentIntervalImage.create(width, height);

      PointProjector *currentPointProjector = _pointProjectors[i].pointProjector;
      if(currentPointProjector != 0) {
	_pointProjectors[i].depthImage = depthImage(cv::Rect(0, columnOffset, width, height));
	currentPointProjector->projectIntervals(currentIntervalImage,
						_pointProjectors[i].depthImage,
						worldRadius);
	intervalImage(cv::Rect(0, columnOffset, width, height)) = currentIntervalImage;
      }
      columnOffset += height;
    }
  }

  bool MultiPointProjector::project(int &x, int &y, float &f, const Point &p) const {
    // Compute total image size
    int maxWidth = 0;
    int totalHeight = 0;
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      if(_pointProjectors[i].pointProjector->imageRows() > maxWidth)
	maxWidth = _pointProjectors[i].pointProjector->imageRows();
      totalHeight += _pointProjectors[i].pointProjector->imageCols();
    }
  
    int columnOffset = 0;
    int currentX = -1, currentY = -1;
    float currentF = 0.0f;
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      const int width = _pointProjectors[i].pointProjector->imageRows();
      const int height = _pointProjectors[i].pointProjector->imageCols();
    
      PointProjector *currentPointProjector = _pointProjectors[i].pointProjector;
      if(currentPointProjector != 0) {
	if(currentPointProjector->project(currentX, currentY, currentF, p)) {
	  if(currentF < 0.0f || 
	     currentX < 0 || currentX >= width || 
	     currentY < 0 || currentY >= height) {
	    currentX = -1; 
	    currentY = -1;
	    currentF = 0.0f;
	  }
	  else {
	    currentY += columnOffset;
	    break;
	  }
	}
	else {
	  currentX = -1; 
	  currentY = -1;
	  currentF = 0.0f;
	}
      }
      columnOffset += height;
    }

    x = currentX;
    y = currentY;
    f = currentF;

    if(x == -1 && y == -1 && f < 0.0f)
      return false;
    return true;
  }

  void MultiPointProjector::setTransform(const Eigen::Isometry3f &transform_) {
    PointProjector::setTransform(transform_);
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      PointProjector *currentPointProjector = _pointProjectors[i].pointProjector;
      if (currentPointProjector) {
	currentPointProjector->setTransform(transform_ * _pointProjectors[i].sensorOffset);
      }
    }
  }

  void MultiPointProjector::clearProjectors() {
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      if(_pointProjectors[i].pointProjector != 0)
	delete(_pointProjectors[i].pointProjector);
      _pointProjectors[i].pointProjector = 0;
    }
    _pointProjectors.clear();
  }

  void  MultiPointProjector::scale(float scalingFactor){
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      if(_pointProjectors[i].pointProjector != 0)
	_pointProjectors[i].pointProjector->scale(scalingFactor);
    }
    int r,c;
    computeImageSize(r,c);
    setImageSize(r,c);
  }

}
