#pragma once

#include "pointprojector.h"

namespace pwn {

  /** \class CylindricalPointProjector cylindricalpointprojector.h "cylindricalpointprojector.h"
   *  \brief Class for point projection/unprojection based on a cylindrical camera mdel.
   *  
   *  This class extends the PointProjector class in order to provide point 
   *  projection/unprojection based on a cylindrical camera model.
   */
  class CylindricalPointProjector : virtual public PointProjector {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Empty constructor.
     *  This constructor creates a PinholePointProjector object with default values for all its
     *  attributes. The camera matrix has to be set. 
     */
    CylindricalPointProjector(); 
    
    /**
     *  Destructor.
     */
    virtual ~CylindricalPointProjector() {}
  
    /**
     *  Virtual method that set the pose transform to the one given in input.
     *  @param transform_ is a constant reference to the isometry used to update the pose transform. 
     *  @see transform()
     */
    virtual void setTransform(const Eigen::Isometry3f &transform_) {   
      _transform = transform_;
      _updateMatrices();
    }

    /**
     *  This method returns the horizontal field of view set to the cylindrical point projector.
     *  @return the horizontal field of view set to the cylindrical point projector. 
     *  @see setAngularFov()
     */
    inline float angularFov() const { return _angularFov; } 

    /**
     *  This method set the angular field of view to the one given in input.
     *  @param angularFov_ is a float value used to update the angular field of view. 
     *  @see angularFov()
     */
    inline void setAngularFov(float angularFov_)  { 
      _angularFov = angularFov_; 
      _updateMatrices();
    } 
    
    /**
     *  This method returns the original angular resolution set to the cylindrical point projector before any scaling modifies it.
     *  @return the angular original resolution set to the cylindrical point projector before any scaling modifies it. 
     *  @see angularResolution()
     *  @see setAngularResolution()
     */
    inline float originalAngularResolution() const { return _originalAngularResolution; } 
    
    /**
     *  This method returns the angular resolution set to the cylindrical point projector.
     *  @return the angular resolution set to the cylindrical point projector. 
     *  @see originalAngularResolution()
     *  @see setAngularResolution()
     */
    inline float angularResolution() const { return _angularResolution; } 
    
    /**
     *  This method set the original angular resolution to the one given in input before any scaling modifies it.
     *  @param angularResolution_ is a float value used to update the original angular resolution before any scaling modifies it. 
     *  @see angularResolution()
     *  @see originalAngularResolution()
     */
    inline void setAngularResolution(float angularResolution_)  { 
      _originalAngularResolution = angularResolution_; 
      _updateMatrices();
    }
    
    /**
     *  This method returns the original vertical focal length set to the cylindrical point projector.
     *  @return the original vertical focal length set to the cylindrical point projector. 
     *  @see verticalFocalLength()
     *  @see setVerticalFocalLength()
     */
    inline float originalVerticalFocalLength() const { return _originalVerticalFocalLength; }

    /**
     *  This method returns the vertical focal length set to the cylindrical point projector.
     *  @return the vertical focal length angular resolution set to the cylindrical point projector. 
     *  @see originalVerticalFocalLength()
     *  @see setVerticalFocalLength()
     */
    inline float verticalFocalLength() const { return _verticalFocalLength; } 

    /**
     *  This method set the original vertical focal length to the one given in input.
     *  @param verticalFocalLength_ is a float value used to update the original vertical focal length. 
     *  @see originalVerticalFocalLength()
     *  @see verticalFocalLength()
     */
    inline void setVerticalFocalLength(float verticalFocalLength_)  { 
      _originalVerticalFocalLength = verticalFocalLength_; 
      _updateMatrices();
    } 
    
    /**
     *  This method returns the original vertical center set to the cylindrical point projector.
     *  @return the original vertical center set to the cylindrical point projector. 
     *  @see verticalCenter()
     *  @see setVerticalCenter()
     */
    inline float originalVerticalCenter() const { return _originalVerticalCenter; } 
    
    /**
     *  This method returns the vertical center set to the cylindrical point projector.
     *  @return the vertical center set to the cylindrical point projector. 
     *  @see originalVerticalCenter()
     *  @see setVerticalCenter()
     */
    inline float verticalCenter() const { return _verticalCenter; } 
    
    /**
     *  This method set the original vertical center to the one given in input.
     *  @param verticalCenter_ is a float value used to update the original vertical center. 
     *  @see originalVerticalCenter()
     *  @see verticalCenter()
     */
    inline void setVerticalCenter(float verticalCenter_)  { 
      _originalVerticalCenter = verticalCenter_; 
      _updateMatrices();
    } 

    /**
     *  Virtual method that projects a given set of homogeneous points from the 3D euclidean space to 
     *  the cylindrical image space space. This method stores the result
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
			 const PointVector& points) ;

    /**
     *  Virtual method that unprojects to the 3D euclidean space the points contained in a cylindrical depth image
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
     *  Virtual method that unprojects to the 3D euclidean space the points contained in a cylindrical depth image
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
     *  Virtual method that projects on the output image the size in pixels of a rectangular regions
     *  around the respective point in the input cylindrical depth image.
     *  @param intervalImage is the output parameter which is a reference to an image containing the size of the square 
     *  regions around the point in the depth image given in input.
     *  @param depthImage is an input parameter which is a constant reference to an image containing depth values in meters.
     *  @param worldRadius is an input parameter containing a float representing the radius of a sphere in the 3D euclidean 
     *  space used to determine the size of the rectangular regions.
     *  @see project()
     *  @see projectInterval()
     */
    virtual void projectIntervals(IntervalImage &intervalImage, 
				  const DepthImage &depthImage, 
				  const float worldRadius) const;  
  
    /**
     *  Virtual method that projects a given point from the 3D euclidean space to 
     *  the cylindrical image space. This method stores the result
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
     *  Virtual method that unprojects to the 3D euclidean space the cylindrical point given in input.
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
     *  Virtual method that projects the size in pixels of a rectangular region around the point specified by the 
     *  the input values.
     *  @param x is an input int value representing the raw of the input point in the depth image.  
     *  @param y is an input int value representing the column of the input point in the depth image.
     *  @param d is an input value containing the depth value of the input point. 
     *  @param worldRadius is an input parameter containing a float representing the radius of a sphere in the 3D euclidean 
     *  space used to determine the size of the square regions.
     *  @return an int value representing the size in pixels of the rectangular region around the input point.
     *  @see projectIntervals()
     */
    virtual inline cv::Vec2i projectInterval(const int x, const int y, const float d, const float worldRadius) const { return _projectInterval(x, y, d, worldRadius); }

    /**
     *  This method is called when is necessary to update the internal matrices used 
     *  for point projection/unprojection.
     */
    void _updateMatrices();
    
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
     *  the cylindrical image space. This method stores the result
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
      Eigen::Vector4f cp = _iT*p;
      d = sqrt(cp.x() * cp.x() + cp.z() * cp.z());
      if(d > _maxDistance || d < _minDistance) {
	return false;
      }
      float theta = atan2(cp.x(), cp.z());
      if(fabs(theta) > _angularFov) {
	return false;
      }
      x = (int)(_angularResolution * theta + _angularCenter);
      y = (int)(cp.y() * _verticalFocalLength/d + _verticalCenter);
      return true;
    } 

    /**
     *  Internal method that unprojects to the 3D euclidean space the cylindrical point given in input.
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
      if(d < _minDistance || d > _maxDistance)
	return false;
      float theta = _inverseAngularResolution * (x_ - _angularCenter);
      if(fabs(theta > _angularFov))
	return false;
      float x = sin(theta) * d;
      float z = cos(theta) * d;
      float y = (y_ - _verticalCenter) * d * _inverseVerticalFocalLength;
      p = _transform * Eigen::Vector3f(x, y, z);
      return true;
    }
  
    /**
     *  Internal method that projects the size in pixels of a rectangular regions around the point specified by the 
     *  the input values.
     *  @param x_ is an input int value representing the raw of the input point in the depth image.  
     *  @param y_ is an input int value representing the column of the input point in the depth image.
     *  @param d is an input value containing the depth value of the input point. 
     *  @param worldRadius is an input parameter containing a float representing the radius of a sphere in the 3D euclidean 
     *  space used to determine the size of the square regions.
     *  @return an int value representing the size in pixels of the rectangular region around the input point.
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
      int x = (int)(_angularResolution * theta);
      int y = (int)(cp.y() *_verticalFocalLength / d);
      return cv::Vec2i(x, y);
    }

    float _originalVerticalFocalLength; /**< Original veritcal focal length equal to the vertical center divided by the tangent of the vertical field of view before any scaling. */
    float _originalVerticalCenter; /**< Original veritcal center which is equal to the half of the rows of the depth image where the projector projects the points before any scaling. */
    float _originalAngularResolution; /**< Original angular resolution indicating how many degrees there are between two columns of the depth image where the projector projects the points before any scaling. */
    float _angularFov; /**< Half orizontal field of view in radians. */
    float _angularCenter; /**< Orizontal angular center which is equal to the angular resolution times the angular field of view. */
    float _verticalFocalLength; /**< Veritcal focal length equal to the vertical center divided by the tangent of the vertical field of view. */
    float _inverseVerticalFocalLength; /**< Inverse of the veritcal focal length. */
    float _verticalCenter; /**< Veritcal center which is equal to the half of the rows of the depth image where the projector projects the points. */
    float _angularResolution; /**< Angular resolution indicating how many degrees there are between two columns of the depth image where the projector projects the points. */
    float _inverseAngularResolution; /**< Inverse of the angular resolution. */
    Eigen::Isometry3f _iT; /**< Inverse of the homogeneous transformation of the projector. */
  };

}
