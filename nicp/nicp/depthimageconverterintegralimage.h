#pragma once

#include "depthimageconverter.h"

namespace nicp {

  /** \class DepthImageConverterIntegralImage depthimageconverterintegralimage.h "depthimageconverterintegralimage.h"
   *  \brief Class for depth image conversion to point cloud with normal computation using integral image.
   *  
   *  This class implements the interface given by the class DepthImageConverter and allows to compute a point
   *  cloud and relative normals by using images structures called Integral Images. Integral image speed up
   *  the normal computatio with respect to classical methods since using them it is possible to compute the 
   *  covariance of a point in constant time.
   */
  class DepthImageConverterIntegralImage : virtual public DepthImageConverter {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    /**
     *  Constructor.
     *  This constructor creates a DepthImageConverterIntegralImage with default values for all its attributes
     *  and will set the necessary objects with the objets given in input. If nothing is passed to the
     *  constructor, then all the pointers to objects implementing an algorithm have to be setted since
     *  this constructor sets them to zero by default.
     */
    DepthImageConverterIntegralImage(PointProjector *_projector = 0,
				     StatsCalculator *_statsCalculator = 0,
				     PointInformationMatrixCalculator *_pointInformationMatrixCalculator = 0,
				     NormalInformationMatrixCalculator *_normalInformationMatrixCalculator = 0);

    /**
     *  Destructor.
     */
    virtual ~DepthImageConverterIntegralImage() {}
    
    /**
     *  This method computes the point cloud included the properties of the points like the normals starting 
     *  from the depth image given in input. If given, it also applies a sensor offset to the cloud.
     *  @param cloud is the output parameter that will contain the computed point cloud. 
     *  @param depthImage is an input parameter containing the depth image to transform in a point cloud. 
     *  @param sensorOffset is an optional input parameter which represents a sensor offset to apply to the 
     *  computed point cloud. 
     */
    virtual void compute(Cloud &cloud,
			 const DepthImage &depthImage, 
			 const Eigen::Isometry3f &sensorOffset = Eigen::Isometry3f::Identity());


    /**
     *  This method computes the point cloud included the properties of the points starting from the depth image
     *  given in input. If given, it also applies a sensor offset to the cloud.
     *  @param cloud is the output parameter that will contain the computed point cloud. 
     *  @param depthImage is an input parameter containing the depth image to transform in a point cloud. 
     *  @param rgbImage is an input parameter containing the registered rgb image
     *  @param sensorOffset is an optional input parameter which represents a sensor offset to apply to the 
     *  computed point cloud. 
     */
    virtual void compute(Cloud &cloud,
			 const DepthImage &depthImage, 
			 const RGBImage &rgbImage, 
			 const Eigen::Isometry3f &sensorOffset = Eigen::Isometry3f::Identity());

  };

}
