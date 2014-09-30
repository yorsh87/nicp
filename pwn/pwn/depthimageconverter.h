#pragma once

#include "cloud.h"
#include "pointprojector.h"
#include "statscalculator.h"
#include "informationmatrixcalculator.h"

namespace pwn {

  /** \class DepthImageConverter depthimageconverter.h "depthimageconverter.h"
   *  \brief Base class interface for depth image conversion to point cloud.
   *  
   *  This class defines an easy interface useful to load a depth image in a point cloud.
   *  It is possible to do all the stuffs, including additional point properties computation,
   *  in the compute function. This allows to compact the code in a unique class that will
   *  handle the depth images loading.
   */
  class DepthImageConverter {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Constructor.
     *  This constructor creates a DepthImageConverter with default values for all its attributes
     *  and will set the necessary objects with the objets given in input. If nothing is passed to the
     *  constructor, then all the pointers to objects implementing an algorithm have to be setted since
     *  this constructor sets them to zero by default.
     */
    DepthImageConverter(PointProjector *_projector = 0,
			StatsCalculator *_statsCalculator = 0,
			PointInformationMatrixCalculator *_pointInformationMatrixCalculator = 0,
			NormalInformationMatrixCalculator *_normalInformationMatrixCalculator = 0);

    /**
     *  Destructor.
     */
    virtual ~DepthImageConverter() {}

    /**
     *  This method computes the point cloud included the properties of the points starting from the depth image
     *  given in input. If given, it also applies a sensor offset to the cloud.
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

    /**
     *  Method that returns a pointer to the point projector used by the DepthImageConverter.
     *  @return a pointer to the DepthImageConverter's point projector.
     *  @see setProjector()
     */
    inline PointProjector* projector() { return _projector; }
    
    /**
     *  Method that set the point projector used by the DepthImageConverter to the one given in input.
     *  @param projector_ is a pointer to the point projector used to update the DepthImageConverter's point projector. 
     *  @see projector()
     */
    inline void setProjector(PointProjector *projector_) { _projector = projector_; }

    /**
     *  Method that returns a pointer to the StatsCalaculator used by the DepthImageConverter.
     *  @return a pointer to the DepthImageConverter's StatsCalaculator.
     *  @see setStatsCalculator()
     */
    inline StatsCalculator* statsCalculator() { return _statsCalculator; }

    /**
     *  Method that set the StatsCalculator used by the DepthImageConverter to the one given in input.
     *  @param statsCalculator_ is a pointer to the StatsCalculator used to update the DepthImageConverter's StatsCalcultor. 
     *  @see statsCalculator()
     */
    inline void setStatsCalculator(StatsCalculator *statsCalculator_) { _statsCalculator = statsCalculator_; }

    /**
     *  Method that returns a pointer to the PointInformationMatrixCalculator used by the DepthImageConverter.
     *  @return a pointer to the DepthImageConverter's PointInformationMatrixCalculator.
     *  @see setPointInformationMatrixCalculator()
     */
    inline PointInformationMatrixCalculator* pointInformationMatrixCalculator() { return _pointInformationMatrixCalculator; }
    
    /**
     *  Method that set the PointInformationMatrixCalculator used by the DepthImageConverter to the one given in input.
     *  @param pointInformationMatrixCalculator_ is a pointer to the PointInformationMatrixCalculator used to update the DepthImageConverter's PointInformationMatrixCalculator. 
     *  @see pointInformationMatrixCalculator()
     */
    inline void setPointInformationMatrixCalculator(PointInformationMatrixCalculator *pointInformationMatrixCalculator_) { _pointInformationMatrixCalculator = pointInformationMatrixCalculator_; }

    /**
     *  Method that returns a pointer to the NormalInformationMatrixCalculator used by the DepthImageConverter.
     *  @return a pointer to the DepthImageConverter's NormalInformationMatrixCalculator.
     *  @see setNormalInformationMatrixCalculator()
     */
    inline NormalInformationMatrixCalculator* normalInformationMatrixCalculator() { return _normalInformationMatrixCalculator; }

    /**
     *  Method that set the NormalInformationMatrixCalculator used by the DepthImageConverter to the one given in input.
     *  @param normalInformationMatrixCalculator_ is a pointer to the NormalInformationMatrixCalculator used to update the DepthImageConverter's NormalInformationMatrixCalculator. 
     *  @see normalInformationMatrixCalculator()
     */
    inline void setNormalInformationMatrixCalculator(NormalInformationMatrixCalculator *normalInformationMatrixCalculator_) { _normalInformationMatrixCalculator = normalInformationMatrixCalculator_; }

    /**
     *  Method that returns a reference to the index image computed during the depth image loading process.
     *  @return a reference to the index image computed during the depth image loading process.
     */
    inline IntImage& indexImage() { return _indexImage; }

  protected:
    PointProjector *_projector; /**< Pointer to the point projector used by the DepthImageConverter to reproject points. */
    StatsCalculator *_statsCalculator; /**< Pointer to the StatsCalculator used by the DepthImageConverter to compute the properties of the points like the normals. */
    PointInformationMatrixCalculator *_pointInformationMatrixCalculator; /**< Pointer to the PointInformationMatrixCalculator used by the DepthImageConverter to compute the information matrix of the points. */
    NormalInformationMatrixCalculator *_normalInformationMatrixCalculator; /**< Pointer to the NormalInformationMatrixCalculator used by the DepthImageConverter to compute the information matrix of the normals. */

    IntImage _indexImage; /**< Index image computed during the depth image loading process. */
  };
}
