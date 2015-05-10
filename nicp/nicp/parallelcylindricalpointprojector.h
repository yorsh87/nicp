#pragma once

#include "pointprojector.h"

namespace nicp {

  /** \class ParallelCylindricalPointProjector parallelcylindricalpointprojector.h "parallelcylindricalpointprojector.h"
   *  \brief Class for point projection/unprojection based on a parallel cylindrical camera mdel.
   *  
   *  This class extends the PointProjector class in order to provide point 
   *  projection/unprojection based on a parallel cylindrical camera model.
   */
  class ParallelCylindricalPointProjector : virtual public PointProjector {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Empty constructor.
     *  This constructor creates a PinholePointProjector object with default values for all its
     *  attributes. The camera matrix has to be set. 
     */
    ParallelCylindricalPointProjector(); 
    
    /**
     *  Destructor.
     */
    virtual ~ParallelCylindricalPointProjector() {}

    /**
     *  This method returns the horizontal field of view set to the parallel cylindrical point projector.
     *  @return the horizontal field of view set to the parallel cylindrical point projector. 
     *  @see setHorizontalFov()
     */
    inline float horizontalFov() const { return _horizontalFov; } 

    /**
     *  This method set the horizontal field of view to the one given in input.
     *  @param horizontalFov_ is a float value used to update the angular field of view. 
     *  @see horizontalFov()
     */
    inline void setHorizontalFov(float horizontalFov_)  { 
      _horizontalFov = horizontalFov_; 
      _updateParameters();
    }

    /**
     *  This method returns the height set to the parallel cylindrical point projector.
     *  @return the height set to the parallel cylindrical point projector. 
     *  @see setHeight()
     */
    inline float height() const { return _height; } 

    /**
     *  This method set the height to the one given in input.
     *  @param verticalFov_ is a float value used to update the height. 
     *  @see height()
     */
    inline void setHeight(float height_)  { 
      _height = height_; 
      _updateParameters();
    }

    /**
     *  This method returns the vertical center set to the parallel cylindrical point projector.
     *  @return the vertical center set to the parallel cylindrical point projector. 
     *  @see setVerticalCenter()
     */
    inline float verticalCenter() const { return _verticalCenter; } 

    /**
     *  This method set the vertical center to the one given in input.
     *  @param verticalCenter_ is a float value used to update the verticalCenter. 
     *  @see verticalCenter()
     */
    inline void setVerticalCenter(float verticalCenter_)  { 
      _verticalCenter = verticalCenter_; 
      _updateParameters();
    }

    /**
     *  Virtual method that set the size (rows and columns) of the image where the points are projected.
     *  @param imageRows_ is a constant int value used to update the number of rows. 
     *  @param imageCols_ is a constant int value used to update the number of columns. 
     *  @see imageRows()
     *  @see imageCols()
     */
    virtual inline void setImageSize(const int imageRows_, const int imageCols_) {
      _imageRows = imageRows_;
      _imageCols = imageCols_;
      _updateParameters();
    }    

    /**
     *  Virtual method that set the pose transform to the one given in input.
     *  @param transform_ is a constant reference to the isometry used to update the pose transform. 
     *  @see transform()
     */
    virtual inline void setTransform(const Eigen::Isometry3f &transform_) { 
      _transform = transform_; 
      _transform.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      _updateParameters();
    }
    
    /**
     *  Virtual method that projects a given set of homogeneous points from the 3D euclidean space to 
     *  the parallel cylindrical image space space. This method stores the result
     *  in two images given in input. The projected points that falls out of the image or that are 
     *  behind other points are not stored in the images.
     *  @param indexImage is an output parameter which is a reference to an image containing indices. 
     *  Each element of this matrix contains the index of the corresponding point in the vector of points given 
     *  in input.
     *  @param depthImage is an output parameter which is a reference to an image containing the depth values 
     *  of the projected points in meters.
     *  @param points is the input parameter which is a constant reference to a vector containing the set 
     *  of homogeneous points to project.
     *  @see unProject()
     *  @see projectIntervals()
     */
    virtual void project(IntImage &indexImage, 
			 DepthImage &depthImage, 
			 const PointVector& points);

    /**
     *  Virtual method that unprojects to the 3D euclidean space the points contained in a parallel cylindrical depth image
     *  This method stores the unprojected points in a vector of points and generates
     *  the associated index image.
     *  @param points is an output parameter which is a reference to a vector containing the set 
     *  of homogeneous points to unproject to the 3D euclidean space.
     *  @param indexImage is an output parameter which is a reference to an image containing indices. 
     *  Each element of this matrix contains the index of the corresponding point in the computed vector of points.
     *  @param depthImage is an output parameter which is a constant reference to an image containing the depth values 
     *  in meters.
     *  @see project()
     *  @see projectIntervals()
     */    
    virtual void unProject(PointVector &points, 
			   IntImage &indexImage, 
			   const DepthImage &depthImage) const;

    /**
     *  Virtual method that unprojects to the 3D euclidean space the points contained in a parallel cylindrical depth image
     *  This method stores the unprojected points in a vector of points and generates
     *  the associated index image. It also compute a vector of gaussians reprensenting the intrinsic error
     *  of the sensor used to acquire the input depth image.
     *  @param points is an output parameter which is a reference to a vector containing the set 
     *  of homogeneous points to unproject to the 3D euclidean space.
     *  @param gaussians is an output parameter which is a reference to a vector containing the set 
     *  of gaussians representing the intrinsic error of the sensor used to acquire the input depth image.
     *  @param indexImage is an output parameter which is a reference to an image containing indices. 
     *  Each element of this matrix contains the index of the corresponding point in the computed vector of points.
     *  @param depthImage is an output parameter which is a constant reference to an image containing the depth values 
     *  in meters.
     *  @see project()
     *  @see projectIntervals()
     */
    virtual void unProject(PointVector &points,
			   Gaussian3fVector &gaussians,
			   IntImage &indexImage,
			   const DepthImage &depthImage) const;
    /**
     *  Virtual method that projects on the output image the size in pixels of a square regions
     *  around the respective point in the input parallel cylindrical depth image.
     *  @param intervalImage is the output parameter which is a reference to an image containing the size of the square 
     *  regions around the point in the depth image given in input.
     *  @param depthImage is an input parameter which is a constant reference to an image containing depth values in meters.
     *  @param worldRadius is an input parameter containing a float representing the radius of a sphere in the 3D euclidean 
     *  space used to determine the size of the square regions.
     *  @see project()
     *  @see projectInterval()
     */
    virtual void projectIntervals(IntervalImage &intervalImage, 
				  const DepthImage &depthImage, 
				  const float worldRadius) const;  
  
    /**
     *  Virtual method that projects a given point from the 3D euclidean space to 
     *  the parallel cylindrical image space. This method stores the result
     *  in three output parameters representing the image coordinates (row and column) and a depth value.
     *  @param x is an output int value that will contain the raw coordinate of the projected input point in the depth image.
     *  @param y is an output int value that will contain the column coordinate of the projected input point in the depth image.
     *  @param f is an output parameter which is a reference to a float that will contain the depth values of the projected input point.
     *  @param p is the input parameter which is a constant reference to an homogeneous point to project.
     *  @return true if the projection is valid, false otherwise.
     *  @see project() 
     */
    virtual inline bool project(int &x, int &y, float &f, const Point &p) const { return _project(x, y, f, p); }

    /**
     *  Virtual method that unprojects to the 3D euclidean space the parallel cylindrical point given in input.
     *  This method stores the unprojected point in an output parameter as an homogeneous
     *  point in the 3D euclidean space.
     *  @param p is the output parameter which is a reference to a the unprojected homogenous point to the 3D euclidean space.
     *  @param x is an input parameter representing the row coordinate of the input depth point.
     *  @param y is an input parameter representing the column coordinate of the input depth point.
     *  @param d is an input value containing the depth value of the depth point. 
     *  @return true if the unprojection is valid, false otherwise.
     *  @see unProject() 
     */
    virtual inline bool unProject(Point &p, const int x, const int y, const float d) const { return _unProject(p, x, y, d); }

    /**
     *  Virtual method that projects the size in pixels of a square regions around the point specified by the 
     *  the input values.
     *  @param x is an input int value representing the raw of the input point in the depth image.  
     *  @param y is an input int value representing the column of the input point in the depth image.
     *  @param d is an input value containing the depth value of the input point. 
     *  @param worldRadius is an input parameter containing a float representing the radius of a sphere in the 3D euclidean 
     *  space used to determine the size of the square regions.
     *  @return an int value representing the size in pixels of the square region around the input point.
     *  @see projectIntervals()
     */
    virtual inline cv::Vec2i projectInterval(const int x, const int y, const float d, const float worldRadius) const { return _projectInterval(x, y, d, worldRadius); }

    /**
     *  This method is called when is necessary to update the internal parameters used 
     *  for point projection/unprojection.
     */
    void _updateParameters();
    
    /**
     *  Method that updates the projector structures in order to handle different size of the
     *  image where the point are projected. If scalingFactor is greater than 1.0 the image will be bigger
     *  (for example in case it's 2.0 the size of the image will be 2 times the size of the original one).
     *  If scalingFactor is lesser than 1.0 the image will be smaller (for example in case it's 0.5 the size 
     *  of the image will be half the size of the original one).
     *  @param scalingFactor is a float value used to update the projector structures.
     */
    virtual void scale(float scalingFactor);

  protected:
    /**
     *  Internal method that projects a given point from the 3D euclidean space to 
     *  the parallel cylindrical image space. This method stores the result
     *  in three output parameters representing the image coordinates (row and column) and a depth value.
     *  @param x is an output int value that will contain the raw coordinate of the projected input point in the depth image.
     *  @param y is an output int value that will contain the column coordinate of the projected input point in the depth image.
     *  @param d is an output parameter which is a reference to a float that will contain the depth values of the projected input point.
     *  @param p is the input parameter which is a constant reference to an homogeneous point to project.
     *  @return true if the projection is valid, false otherwise.
     *  @see project() 
     */
    inline bool _project(int &x, int &y, float &d, const Point &p) const {
      // Point in camera coordinates;
      Eigen::Vector4f cp = _iT * p;
      float theta = atan2(cp.x(), cp.z());
      d = sqrt(cp.x() * cp.x() + cp.z() * cp.z());
      if(d > _maxDistance || d < _minDistance) { return false; }
      if(fabs(theta > _horizontalFov)) { return false; }
      if(fabs(cp.y()) > _height) { return false; }
      x = (int)(_horizontalResolution * theta + _horizontalCenter);
      y = (int)(_verticalResolution * cp.y() + _verticalCenter);
      return true;
    } 

    /**
     *  Internal method that unprojects to the 3D euclidean space the parallel cylindrical point given in input.
     *  This method stores the unprojected point in an output parameter as an homogeneous
     *  point in the 3D euclidean space.
     *  @param p is the output parameter which is a reference to a the unprojected homogenous point to the 3D euclidean space.
     *  @param x_ is an input parameter representing the row coordinate of the input depth point.
     *  @param y_ is an input parameter representing the column coordinate of the input depth point.
     *  @param d is an input value containing the depth value of the depth point. 
     *  @return true if the unprojection is valid, false otherwise.
     *  @see unProject() 
     */
    inline bool _unProject(Point &p, const int x_, const int y_, const float d) const {      
      float theta = _inverseHorizontalResolution * (x_ - _horizontalCenter);
      if(d < _minDistance || d > _maxDistance) {return false; }
      if(fabs(theta) > _horizontalFov) { return false; }
      float x = sin(theta) * d;
      float z = cos(theta) * d;
      float y = (y_ - _verticalCenter) * _inverseVerticalResolution;
      p = _transform * Eigen::Vector3f(x, y, z);
      return true;
    }
  
    /**
     *  Internal method that projects the size in pixels of a square regions around the point specified by the 
     *  the input values.
     *  @param x_ is an input int value representing the raw of the input point in the depth image.  
     *  @param y_ is an input int value representing the column of the input point in the depth image.
     *  @param d is an input value containing the depth value of the input point. 
     *  @param worldRadius is an input parameter containing a float representing the radius of a sphere in the 3D euclidean 
     *  space used to determine the size of the square regions.
     *  @return an int value representing the size in pixels of the square region around the input point.
     *  @see projectIntervals()
     */
    inline cv::Vec2i _projectInterval(const int x_, const int y_, const float d, const float worldRadius) const {
      // Just to avoid compilation warnings
      if(x_ || y_) {}
      // Point in camera coordinates;
      if(d > _maxDistance || d < _minDistance)
      	return cv::Vec2i(-1, -1);      
      Eigen::Vector4f cp = Eigen::Vector4f(worldRadius, worldRadius, d, 1.0f);
      float theta = atan2(cp.x(), cp.z());
      int x = (int)(_horizontalResolution * theta);
      int y = (int)(_verticalResolution * worldRadius);
      return cv::Vec2i(x, y);
    }

    float _horizontalFov; /**< Half horizontal field of view in radians. */
    float _horizontalCenter; /**< Horizontal center which is equal to the horizontal resolution times the horizontal field of view. */
    float _height; /**< Half height in meters. */
    float _verticalCenter; /**< Vertical center which is equal to the half of the rows of the depth image where the projector projects the points. */
    float _horizontalResolution; /**< Horizontal resolution indicating how many degrees there are between two columns of the depth image where the projector projects the points. */
    float _inverseHorizontalResolution; /**< Inverse of the horizontal resolution. */
    float _verticalResolution; /**< Vertical resolution indicating how many degrees there are between two columns of the depth image where the projector projects the points. */
    float _inverseVerticalResolution; /**< Inverse of the vertical resolution. */    
    Eigen::Isometry3f _iT; /**< Inverse of the homogeneous transformation of the projector. */
  };

}
