#pragma once

#include "gaussian3.h"

namespace nicp {

  /** \class PointProjector pointprojector.h "pointprojector.h"
   *  \brief Base class interface for point projection/unprojection.
   *  
   *  Points in the 3D euclidean space can be projected to a subspace, which can be a plane,
   *  a cylinder, a sphere or whatever you can image. At the same way they can be unprojected
   *  from their projection subspace to the 3D euclidean space. This class provides a useful 
   *  interface that can be extended in order to build a specific PointProjector and then use it 
   *  to do the basic projection/unprojection operations on set of points.
   */
  class PointProjector {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Empty constructor.
     *  This constructor creates a PointProjector object setting the pose transform to the identity, 
     *  the maximum and minimum distance are imposed to be respectively 10.0 and 0.01 meters,
     *  image rows and columns are setted to 0.
     */
    PointProjector();
    
    /**
     *  Destructor.
     */
    virtual ~PointProjector() {}
  
    /**
     *  Virtual method that returns the pose transform.
     *  @return a constant reference to the pose transform.
     *  @see setTransform()
     */
    virtual inline const Eigen::Isometry3f &transform() const { return _transform; };  
    
    /**
     *  Virtual method that set the pose transform to the one given in input.
     *  @param transform_ is a constant reference to the isometry used to update the pose transform. 
     *  @see transform()
     */
    virtual inline void setTransform(const Eigen::Isometry3f &transform_) { 
      _transform = transform_; 
      _transform.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    }

    /**
     *  Method that returns the minimum distance value.
     *  @return the minimum distance value.
     *  @see setMinDistance()
     */    
    inline float minDistance() const { return _minDistance; }
    
    /**
     *  Method that set the minimum distance value to the one given in input.
     *  @param minDistance_ is a float value used to update the minimum distance variable. 
     *  @see minDistance()
     */
    inline void setMinDistance(const float minDistance_) { _minDistance = minDistance_; }

    /**
     *  Method that returns the scaling factor value.
     *  @return the scaling factor value.
     */    
    inline float scalingFactor() const { return _scalingFactor; }
    
    /**
     *  Method that returns the maximum distance value.
     *  @return the maximum distance value.
     *  @see setMaxDistance()
     */
    inline float maxDistance() const { return _maxDistance; }

    /**
     *  Method that set the maximum distance value to the one given in input.
     *  @param maxDistance_ is a float value used to update the maximum distance variable. 
     *  @see maxDistance()
     */
    inline void setMaxDistance(const float maxDistance_) { _maxDistance = maxDistance_; }

    /**
     *  Method that returns the number of rows of the image where the points are projected.
     *  @return the number of rows of the image where the points are projected.
     *  @see imageCols()
     *  @see originalImageRows()
     *  @see originalImageCols()
     *  @see setImageSize()
     */
    inline int imageRows() const { return _imageRows; }

    /**
     *  Method that returns the number of columns of the image where the points are projected.
     *  @return the number of columns of the image where the points are projected.
     *  @see imageRows()
     *  @see originalImageRows()
     *  @see originalImageCols()
     *  @see setImageSize()
     */
    inline int imageCols() const { return _imageCols; }

    /**
     *  Method that returns the original number of rows of the image where the points are projected.
     *  @return the original number of rows of the image where the points are projected.
     *  @see imageRows()
     *  @see imageCols()
     *  @see originalImageCols()
     *  @see setImageSize()
     */
    inline int originalImageRows() const { return _originalImageRows;}

    /**
     *  Method that returns the original number of columns of the image where the points are projected.
     *  @return the original number of columns of the image where the points are projected.
     *  @see imageRows()
     *  @see imageCols()
     *  @see originalImageRows()
     *  @see setImageSize()
     */
    inline int originalImageCols() const { return _originalImageCols;}

    /**
     *  Virtual method that set the original size (rows and columns) of the image where the points are projected.
     *  @param imageRows_ is a constant int value used to update the original number of rows. 
     *  @param imageCols_ is a constant int value used to update the original number of columns. 
     *  @see imageRows()
     *  @see imageCols()
     */
    virtual inline void setImageSize(const int imageRows_, const int imageCols_) {
      _originalImageRows = imageRows_;
      _originalImageCols = imageCols_;
    }    
  
    /**
     *  Virtual method that projects a given set of homogeneous points from the 3D euclidean space to 
     *  a destination space defined by the user extending this class. This method stores the result
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
			 const PointVector &points) const;
    
    /**
     *  Virtual method that unprojects to the 3D euclidean space the points contained in a depth image
     *  The unprojection operation is specific for the projection space defined by the class  
     *  extending this one. This method stores the unprojected points in a vector of points and generates
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
     *  The unprojection operation is specific for the projection space defined by the class  
     *  extending this one. This method stores the unprojected points in a vector of points and generates
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
     *  around the respective point in the input depth image.
     *  @param intervalImage is the output parameter which is a reference to an image containing the size of the rectangular 
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
     *  a destination space defined by the user extending this class. This method stores the result
     *  in three output parameters representing the image coordinates (row and column) and a depth value.
     *  @param x is an output int value that will contain the raw coordinate of the projected input point in the depth image.
     *  @param y is an output int value that will contain the column coordinate of the projected input point in the depth image.
     *  @param f is an output parameter which is a reference to a float that will contain the depth values of the projected input point.
     *  @param p is the input parameter which is a constant reference to an homogeneous point to project.
     *  @return true if the projection is valid, false otherwise.
     *  @see project() 
     */
    virtual inline bool project(int &x, int &y, float &f, const Point &p) const { 
      // Just to avoid warnings in the compilation and in doxygen
      if(x || y || f || p.x()) {} 

      return false; 
    }

    /**
     *  Virtual method that unprojects to the 3D euclidean space the point given in input.
     *  The unprojection operation is specific for the projection space defined by the class  
     *  extending this one. This method stores the unprojected point in an output parameter as an homogeneous
     *  point in the 3D euclidean space.
     *  @param p is the output parameter which is a reference to a the unprojected homogenous point to the 3D euclidean space.
     *  @param x is an input parameter representing the row coordinate of the input depth point.
     *  @param y is an input parameter representing the column coordinate of the input depth point.
     *  @param d is an input value containing the depth value of the depth point. 
     *  @return true if the unprojection is valid, false otherwise.
     *  @see unProject() 
     */
    virtual inline bool unProject(Point &p, const int x, const int y, const float d) const { 
      // Just to avoid warnings in the compilation and in doxygen
      if(x || y || d || p.x()) {} 

      return false; 
    }
    
    /**
     *  Virtual method that projects the size in pixels of a rectangular regions around the point specified by the 
     *  the input values.
     *  @param x is an input int value representing the raw of the input point in the depth image.  
     *  @param y is an input int value representing the column of the input point in the depth image.
     *  @param d is an input value containing the depth value of the input point. 
     *  @param worldRadius is an input parameter containing a float representing the radius of a sphere in the 3D euclidean 
     *  space used to determine the size of the rectangular regions.
     *  @return a Vec2i representing the size in pixels of the rectangular region around the input point.
     *  @see projectIntervals()
     */
    virtual inline cv::Vec2i projectInterval(const int x, const int y, const float d, const float worldRadius) const { 
      // Just to avoid warnings in the compilation and in doxygen
      if(x || y || d || worldRadius) {} 

      return cv::Vec2i(-1, -1); 
    }

    /**
     *  Virtual method that updates the projector structures in order to handle different size of the
     *  image where the point are projected. If scalingFactor is greater than 1.0 the image will be bigger
     *  (for example in case it's 2.0 the size of the image will be 2 times the size of the original one).
     *  If scalingFactor is lesser than 1.0 the image will be smaller (for example in case it's 0.5 the size 
     *  of the image will be half the size of the original one).
     *  @param scalingFactor is a float value used to update the projector structures.
     */
    virtual void scale(float scalingFactor);
   
  protected:
    Eigen::Isometry3f _transform; /**< Transform to apply to the pose in order to change the point of view from which the points are seen. */
  
    float _minDistance; /**< Minimum distance in meters for which all the points below its value are cutted. */
    float _maxDistance; /**< Maximum distance in meters for which all the points above its value are cutted. */

    int _originalImageRows; /**< Original rows of the image to which the points will projected before any scaling. */
    int _originalImageCols; /**< Original columns of the image to which the points will projected before any scaling. */
    int _imageRows; /**< Rows of the image to which the points will projected. */
    int _imageCols; /**< Columns of the image to which the points will projected. */

    float _scalingFactor; /**< Scaling factor applied to the projector parameters. */
  };

}
