#pragma once

#include "statscalculator.h"
#include "pointintegralimage.h"

namespace pwn {

  /** \class StatsCalculatorIntegralImage statscalculatorintegralimage.h "statscalculatorintegralimage.h"
   *  \brief Class for point properties computation using integral images.
   *  
   *  This class provides the necessary methods for normals and additional point properties using
   *  a fast structure called integral image.
   */
  class StatsCalculatorIntegralImage : virtual public StatsCalculator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
    /**
     *  Empty constructor.
     *  This empty constructor creates a StatsCalculatorIntegralImage with default values for all
     *  its attributes.
     */
    StatsCalculatorIntegralImage();

    /**
     *  Destructor.
     */
    virtual ~StatsCalculatorIntegralImage() {}

    /**
     *  This virtual method computes the normals and additional properties of an input vector of points.
     *  @param normals is the vector where the normals computed are stored.
     *  @param statsVector is the vector where the additional properties computed are stored.
     *  @param points is the input vector of points.
     *  @param indexImage is the IndexImage containing the indeces of the depth image points in the vector of points.
     */
    virtual void compute(NormalVector &normals,
			 StatsVector &statsVector,
			 const PointVector &points,
			 const IntImage &indexImage,
			 bool suppressImageRadius = false);
    
    /**
     *  This method sets the maximum image radius of the StatsCalculatorIntegralImage to the one given in input.
     *  @param maxImageRadius_ is an int value used to update the maximum image radius set to the StatsCalculatorIntegralImage.
     *  @see maxImageRadius()
     */
    inline void setMaxImageRadius(const int maxImageRadius_) { _maxImageRadius = maxImageRadius_; }
    
    /**
     *  This method returns the maximum image radius set to the StatsCalculatorIntegralImage.
     *  @return the maximum image radius set to the StatsCalculatorIntegralImage.
     *  @see setMaxImageRadius()
     */
    inline int maxImageRadius() const { return _maxImageRadius; }
    
    /**
     *  This method sets the minimum image radius of the StatsCalculatorIntegralImage to the one given in input.
     *  @param minImageRadius_ is an int value used to update the minimum image radius set to the StatsCalculatorIntegralImage.
     *  @see minImageRadius()
     */
    inline void setMinImageRadius(const int minImageRadius_) { _minImageRadius = minImageRadius_; }
    
    /**
     *  This method returns the minimum image radius set to the StatsCalculatorIntegralImage.
     *  @return the minimum image radius set to the StatsCalculatorIntegralImage.
     *  @see setMinImageRadius()
     */
    inline int minImageRadius() const { return _minImageRadius; }
    
    /**
     *  This method sets the minimum number of points of the StatsCalculatorIntegralImage to the one given in input.
     *  @param minPoints_ is an int value used to update the minimum number of points set to the StatsCalculatorIntegralImage.
     *  @see minPoints()
     */
    inline void setMinPoints(const int minPoints_) { _minPoints = minPoints_; }
    
    /**
     *  This method returns the minimum number of points set to the StatsCalculatorIntegralImage.
     *  @return the minimum number of points set to the StatsCalculatorIntegralImage.
     *  @see setMinPoints()
     */
    inline int minPoints() const { return _minPoints; }
    
    /**
     *  This method sets the curvature threshold of StatsCalculatorIntegralImage to the one given in input.
     *  @param curvatureThreshold_ is a float value used to update the curvature threshold set to the StatsCalculatorIntegralImage.
     *  @see curvatureThreshold()
     */
    inline void setCurvatureThreshold(float curvatureThreshold_) {_curvatureThreshold = curvatureThreshold_;}
    
    /**
     *  This method returns the curvature threshold set to the StatsCalculatorIntegralImage.
     *  @return the curvature threshold set to the StatsCalculatorIntegralImage.
     *  @see setCurvatureThreshold()
     */
    inline float curvatureThreshold() const { return _curvatureThreshold; }
    
    /**
     *  This method sets the world radius of the StatsCalculatorIntegralImage to the one given in input.
     *  @param worldRadius_ is a float value used to update the world radius set to the StatsCalculatorIntegralImage.
     *  @see worldRadius()
     */
    inline void setWorldRadius(const float worldRadius_) { _worldRadius = worldRadius_; }    

    /**
     *  This method returns the world radius set to the StatsCalculatorIntegralImage.
     *  @return the world radius set to the StatsCalculatorIntegralImage.
     *  @see setWorldRadius()
     */
    inline float worldRadius() const { return _worldRadius; }

    /**
     *  This method returns the integral image computed by the StatsCalculatorIntegralImage.
     *  @return the the integral image computed by the StatsCalculatorIntegralImage.
     */
    inline PointIntegralImage& integralImage() { return _integralImage; }
    
    /**
     *  This method returns the interval image computed by the StatsCalculatorIntegralImage.
     *  @return the interval image computed by the StatsCalculatorIntegralImage.
     */
    inline IntervalImage& intervalImage() { return _intervalImage; }

  protected:
    int _maxImageRadius; /**< Maximum radius in the image space where to select neighboring points for normal computation. */
    int _minImageRadius; /**< Minimum radius in the image space where to select neighboring points for normal computation. */
    int _minPoints; /**< Minimum number of points to consider a normal valid. */
    float _curvatureThreshold; /**< Curvature threshold for which point normals with higher curvature are not valid. */
    float _worldRadius; /**< Radius in the 3D euclidean space in which neighboring points are selceted for normal computation. */
    PointIntegralImage _integralImage; /**< Integral image used to compute the normals and additional properties. */
    IntervalImage _intervalImage; /**< Interval image used to compute the normals and additional properties. */
  };

}
