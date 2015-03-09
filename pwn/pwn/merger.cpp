#include "merger.h"

namespace pwn {

  Merger::Merger() {
    _distanceThreshold = 0.1f;
    _normalThreshold = cosf(10 * M_PI / 180.0f);
    _maxPointDepth = 10.0f;
    _depthImageConverter = 0;
    _indexImage.create(0, 0);
    _depthImage.create(0, 0);
    _collapsedIndices.resize(0);
  }

  void Merger::merge(Cloud *cloud, Eigen::Isometry3f transform) {
    assert(_indexImage.rows > 0 && _indexImage.cols > 0 && "Merger: _indexImage has zero size");  
    assert(_depthImageConverter  && "Merger: missing _depthImageConverter");  
    assert(_depthImageConverter->projector()  && "Merger: missing projector in _depthImageConverter");  

    PointProjector *pointProjector = _depthImageConverter->projector();
    Eigen::Isometry3f oldTransform = pointProjector->transform();
    
    pointProjector->setTransform(transform);
    pointProjector->project(_indexImage, 
			    _depthImage, 
			    cloud->points());

    int target = 0;
    int distance = 0;
    _collapsedIndices.resize(cloud->points().size());
    std::fill(_collapsedIndices.begin(), _collapsedIndices.end(), -1);

    int killed = 0;
    int currentIndex = 0;
    for(size_t i = 0; i < cloud->points().size(); currentIndex++ ,i++) {
      const Point currentPoint = cloud->points()[i];
      const Normal currentNormal = cloud->normals()[i];

      int r = -1, c = -1;
      float depth = 0.0f;
      pointProjector->project(c, r, depth, currentPoint);
      if(depth < 0 || depth > _maxPointDepth || 
	 r < 0 || r >= _depthImage.rows || 
	 c < 0 || c >= _depthImage.cols) {
	distance++;
	continue;
      }
        
      float &targetZ = _depthImage(r, c);
      int targetIndex = _indexImage(r, c);
      if(targetIndex < 0) {
	target++;
	continue;
      }
      const Normal &targetNormal = cloud->normals().at(targetIndex);

      Eigen::Vector4f viewPointDirection = transform.matrix().col(3)-currentPoint;
      viewPointDirection(3)=0;
      if(targetIndex == currentIndex) {
	_collapsedIndices[currentIndex] = currentIndex;
      } 
      else if(fabs(depth - targetZ) < _distanceThreshold /*&& 
	      currentNormal.dot(targetNormal) > _normalThreshold &&
	      (viewPointDirection.dot(targetNormal)>cos(0))*/ ) {
	Gaussian3f &targetGaussian = cloud->gaussians()[targetIndex];
	Gaussian3f &currentGaussian = cloud->gaussians()[currentIndex];
	targetGaussian.addInformation(currentGaussian);
	_collapsedIndices[currentIndex] = targetIndex;
	killed++;
      }
    }

    int murdered = 0;
    int k = 0;  
    for(size_t i = 0; i < _collapsedIndices.size(); i++) {
      int collapsedIndex = _collapsedIndices[i];
      if(collapsedIndex == (int)i) {
	cloud->points()[i].head<3>() = cloud->gaussians()[i].mean();
      }
      if(collapsedIndex < 0 || collapsedIndex == (int)i) {
	cloud->points()[k] = cloud->points()[i];
	cloud->normals()[k] = cloud->normals()[i];
	cloud->stats()[k] = cloud->stats()[i];
	cloud->pointInformationMatrix()[k] = cloud->pointInformationMatrix()[i];
	cloud->normalInformationMatrix()[k] = cloud->normalInformationMatrix()[i];
	cloud->gaussians()[k] = cloud->gaussians()[i];
	if(cloud->rgbs().size())
	  cloud->rgbs()[k]=cloud->rgbs()[i];
	k++;
      } 
      else {
	murdered ++;
      }
    }    
    int originalSize = cloud->points().size();
    
    // Kill the leftover points
    cloud->points().resize(k);
    cloud->normals().resize(k);
    cloud->stats().resize(k);
    cloud->pointInformationMatrix().resize(k);
    cloud->normalInformationMatrix().resize(k);
    if(cloud->rgbs().size())
      cloud->rgbs().resize(k);
    std::cout << "[INFO]: number of suppressed points " << murdered << std::endl;
    std::cout << "[INFO]: resized cloud from " << originalSize << " to " << k << " points" <<std::endl;
    
    pointProjector->setTransform(oldTransform);
  }

}
