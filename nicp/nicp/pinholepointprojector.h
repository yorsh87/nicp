#pragma once

#include "pointprojector.h"

namespace nicp {

  /** \class PinholePointProjector pinholepointprojector.h "pinholepointprojector.h"
   *  \brief Class for point projection/unprojection based on pinhole camera mdel.
   *  
   *  This class extends the PointProjector class in order to provide point 
   *  projection/unprojection based on pinhole camera model.
   */
  class PinholePointProjector : virtual public PointProjector {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Empty constructor.
     *  This constructor creates a PinholePointProjector object with default values for all its
     *  attributes. The camera matrix has to be set. 
     */
    PinholePointProjector();  

    /**
     *  Destructor.
     */
    virtual ~PinholePointProjector() {}
  
    /**
     *  Virtual method that set the pose transform to the one given in input.
     *  @param transform_ is a constant reference to the isometry used to update the pose transform. 
     *  @see transform()
     */
    virtual inline void setTransform(const Eigen::Isometry3f &transform_) {   
      _transform = transform_;
      _updateMatrices();
    }

    /**
     *  This method returns the camera matrix set to the pinholepointprojector.
     *  @return the camera matrix set to the pinholepointprojector. 
     *  @see inverseCameraMatrix()
     *  @see originalCameraMatrix()
     *  @see inverseOriginalCameraMatrix()
     *  @see setCameraMatrix()
     */
    inline const Eigen::Matrix3f& cameraMatrix() const { return _cameraMatrix; }  

    /**
     *  This method returns the original camera matrix set to the pinholepointprojector.
     *  @return the original camera matrix set to the pinholepointprojector. 
     *  @see cameraMatrix()
     *  @see inverseCameraMatrix()
     *  @see originalInverseCameraMatrix()
     *  @see setCameraMatrix()
     */
    inline const Eigen::Matrix3f& originalCameraMatrix() const { return _originalCameraMatrix; }  

    /**
     *  This method set the original camera matrix to the one given in input.
     *  @param cameraMatrix_ is a constant reference to the 3x3 float matrix used to update the original camera matrix. 
     *  @see cameraMatrix()
     *  @see inverseCameraMatrix()
     *  @see originalCameraMatrix()
     *  @see inverseOriginalCameraMatrix()
     */
    inline void setCameraMatrix(const Eigen::Matrix3f &cameraMatrix_) {
      _originalCameraMatrix = cameraMatrix_;
      _updateMatrices();
    }

    /**
     *  This method returns the inverse of the original camera matrix set to the pinholepointprojector.
     *  @return the inverse of the original camera matrix set to the pinholepointprojector. 
     *  @see cameraMatrix()
     *  @see inverseCameraMatrix()
     *  @see originalCameraMatrix()
     *  @see setCameraMatrix()
     */
    inline const Eigen::Matrix3f& originalInverseCameraMatrix() const { return _originaliK; }

    /**
     *  This method returns the inverse of the camera matrix set to the pinholepointprojector.
     *  @return the inverse of the camera matrix set to the pinholepointprojector. 
     *  @see cameraMatrix()
     *  @see originalCameraMatrix()
     *  @see inverseOriginalCameraMatrix()
     *  @see setCameraMatrix()
     */
    inline const Eigen::Matrix3f& inverseCameraMatrix() const { return _iK; }

    /**
     *  This method returns the baseline set to the pinholepointprojector.
     *  @return the baseline set to the pinholepointprojector. 
     *  @see setBaseline()
     */
    inline float baseline() const { return _baseline; }

    /**
     *  This method set the baseline to the one given in input.
     *  @param baseline_ is a float value used to update the baseline. 
     *  @see baseline()
     */
    inline void setBaseline(float baseline_) { _baseline = baseline_; }

    /**
     *  This method returns the alpha increment set to the pinholepointprojector.
     *  @return the alpha increment set to the pinholepointprojector. 
     *  @see setAlpha()
     */
    inline float alpha() const { return _alpha; }  

    /**
     *  This method set the alpha increment to the one given in input.
     *  @param alpha_ is a float value used to update the alpha increment. 
     *  @see alpha()
     */
    inline void setAlpha(float alpha_) { _alpha = alpha_; }

    /**
     *  Virtual method that projects a given set of homogeneous points from the 3D euclidean space to 
     *  the image space space. This method stores the result
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
			 const PointVector& points) const;

    /**
     *  Virtual method that unprojects to the 3D euclidean space the points contained in a depth image
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
     *  Virtual method that unprojects to the 3D euclidean space the points contained in a depth image
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
     *  around the respective point in the input depth image.
     *  @param intervalImage is the output parameter which is a reference to an image containing the size of the rectangular 
     *  regions around the point in the depth image given in input.
     *  @param depthImage is an input parameter which is a constant reference to an image containing depth values in meters.
     *  @param worldRadius is an input parameter containing a float representing the radius of a sphere in the 3D euclidean 
     *  space used to determine the size of the rectangular regions.
     *  @see project()
     *  @see projectInterval()
     */
    void projectIntervals(IntervalImage &intervalImage, 
			  const DepthImage &depthImage, 
			  const float worldRadius) const;

    /**
     *  Virtual method that projects a given point from the 3D euclidean space to 
     *  the image space. This method stores the result
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
     *  Virtual method that unprojects to the 3D euclidean space the point given in input.
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
     *  Virtual method that projects the size in pixels of a rectangular regions around the point specified by the 
     *  the input values.
     *  @param x is an input int value representing the raw of the input point in the depth image.  
     *  @param y is an input int value representing the column of the input point in the depth image.
     *  @param d is an input value containing the depth value of the input point. 
     *  @param worldRadius is an input parameter containing a float representing the radius of a sphere in the 3D euclidean 
     *  space used to determine the size of the rectangular regions.
     *  @return an int value representing the size in pixels of the rectangular region around the input point.
     *  @see projectIntervals()
     */
    virtual inline cv::Vec2i projectInterval(const int x, const int y, const float d, const float worldRadius) const { return _projectInterval(x, y, d, worldRadius); }  

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
     *  the image space. This method stores the result
     *  in three output parameters representing the image coordinates (row and column) and a depth value.
     *  @param x is an output int value that will contain the raw coordinate of the projected input point in the depth image.
     *  @param y is an output int value that will contain the column coordinate of the projected input point in the depth image.
     *  @param d is an output parameter which is a reference to a float that will contain the depth values of the projected input point.
     *  @param p is the input parameter which is a constant reference to an homogeneous point to project.
     *  @return true if the projection is valid, false otherwise.
     *  @see project() 
     */
    inline bool _project(int &x, int &y, float &d, const Point &p) const {
      Eigen::Vector4f ip = _KRt * p;
      d = ip.coeff(2);
      if(d < _minDistance || d > _maxDistance)
	return false;
      ip *= (1.0f / d);
      x = (int)round(ip.coeff(0));
      y = (int)round(ip.coeff(1)); 
      return true;
    }
  
    /**
     *  Internal method that unprojects to the 3D euclidean space the point given in input.
     *  This method stores the unprojected point in an output parameter as an homogeneous
     *  point in the 3D euclidean space.
     *  @param p is the output parameter which is a reference to a the unprojected homogenous point to the 3D euclidean space.
     *  @param x is an input parameter representing the row coordinate of the input depth point.
     *  @param y is an input parameter representing the column coordinate of the input depth point.
     *  @param d is an input value containing the depth value of the depth point. 
     *  @return true if the unprojection is valid, false otherwise.
     *  @see unProject() 
     */
    inline bool _unProject(Point &p, const int x, const int y, const float d) const {
      if(d == 0.0f || d < _minDistance || d > _maxDistance)
	return false;
      p = _iKRt * Eigen::Vector4f(x * d,y * d, d, 1.0f);
      return true;
    }
  
    /**
     *  Internal method that projects the size in pixels of a rectangular regions around the point specified by the 
     *  the input values.
     *  @param x is an input int value representing the raw of the input point in the depth image.  
     *  @param y is an input int value representing the column of the input point in the depth image.
     *  @param d is an input value containing the depth value of the input point. 
     *  @param worldRadius is an input parameter containing a float representing the radius of a sphere in the 3D euclidean 
     *  space used to determine the size of the rectangular regions.
     *  @return an int value representing the size in pixels of the rectangular region around the input point.
     *  @see projectIntervals()
     */
    inline cv::Vec2i _projectInterval(const int x, const int y, const float d, const float worldRadius) const {
      // Just to avoid compilation warnings
      if(x || y) {}
      if (d < _minDistance || d > _maxDistance)
      	return cv::Vec2i(-1, -1);      
      Eigen::Vector3f p = _cameraMatrix * Eigen::Vector3f(worldRadius, worldRadius, 0);
      p *= (1.0f / d);
      float maxSide = std::max(p.coeff(0), p.coeff(1));
      return cv::Vec2i(maxSide, maxSide);
    }

    /**
     *  This method is called when is necessary to update the internal matrices used 
     *  for point projection/unprojection.
     */
    void _updateMatrices();
  
    float _baseline; /**< Horizontal baseline between the cameras (in meters). */
    float _alpha; /**< Alpha increment. */
 
    Eigen::Matrix3f _originalCameraMatrix; /**< Original Camera matrix K. */
    Eigen::Matrix3f _originaliK; /**< Inverse of the original camera matrix K. */
    Eigen::Matrix3f _cameraMatrix; /**< Camera matrix K. */    
    Eigen::Matrix4f _KRt; /**< Camera matrix multiplied with the rotation matrix and translation vector of the projector pose transoform. */
    Eigen::Matrix4f _iKRt; /**< Inverse of the camera matrix multiplied with the rotation matrix and translation vector of the projector pose transoform. */
    Eigen::Matrix3f _iK; /**< Inverse of the camera matrix K. */
    Eigen::Matrix3f _KR; /**< Camera matrix multiplied with the rotation matrix of the projector pose transoform. */
    Eigen::Vector3f _Kt; /**< Camera matrix multiplied with the translation vector of the projector pose transoform. */
    Eigen::Matrix3f _iKR; /**< Inverse of the camera matrix multiplied with the rotation matrix of the projector pose transoform. */
    Eigen::Vector3f _iKt; /**< Inverse of the camera matrix multiplied with the translation vector of the projector pose transoform. */
  };

}
