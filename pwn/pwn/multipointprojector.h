#pragma once

#include "pointprojector.h"

namespace pwn {

  class MultiPointProjector : virtual public PointProjector {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    MultiPointProjector() : PointProjector() {}
    virtual ~MultiPointProjector() {}

    void addPointProjector(PointProjector *pointProjector_, 
			   Eigen::Isometry3f sensorOffset_,
			   int width_, int height_) { 
      _pointProjectors.push_back(ChildProjectorInfo(pointProjector_, transform() * sensorOffset_, width_, height_));
    }
  
    void setPointProjector(PointProjector *pointProjector_, 
			   Eigen::Isometry3f sensorOffset_,
			   int width_, int height_,
			   int position) { 
      _pointProjectors[position].pointProjector = pointProjector_;
      _pointProjectors[position].sensorOffset = sensorOffset_;
      pointProjector_->setImageSize(width_, height_);
    }

    void clearProjectors();

    void computeImageSize(int &rows, int &cols) const;

    virtual void project(IntImage &indexImage, 
			 DepthImage &depthImage, 
			 const PointVector &points);

    virtual void unProject(PointVector &points,
			   IntImage &indexImage, 
			   const DepthImage &depthImage) const;

    virtual void unProject(PointVector &points,
			   Gaussian3fVector &gaussians,
			   IntImage &indexImage,
			   const DepthImage &depthImage) const;
  
    virtual void projectIntervals(IntervalImage& intervalImage, 
				  const DepthImage &depthImage, 
				  const float worldRadius) const;

    //virtual inline int projectInterval(const int x, const int y, const float d, const float worldRadius) const;

    virtual bool project(int &x, int &y, float &f, const Point &p) const;
  
    //virtual bool unProject(Point &p, const int x, const int y, const float d) const;

    virtual void setTransform(const Eigen::Isometry3f &transform_);

    virtual void scale(float scalingFactor);

  protected:
    struct ChildProjectorInfo {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      PointProjector *pointProjector;
      Eigen::Isometry3f sensorOffset;
      DepthImage depthImage;
      IntImage indexImage;

      ChildProjectorInfo(PointProjector *pointProjector_,
			 Eigen::Isometry3f sensorOffset_ = Eigen::Isometry3f::Identity(),
			 int width_ = 0, int height_ = 0) {
	pointProjector = pointProjector_;
	sensorOffset = sensorOffset_;
	pointProjector->setImageSize(width_, height_);
	if(indexImage.rows != width_ || indexImage.cols != height_)
	  indexImage.create(width_, height_);    
      }
      virtual ~ChildProjectorInfo() {}
    };  

    mutable std::vector<ChildProjectorInfo> _pointProjectors;
  };

}
