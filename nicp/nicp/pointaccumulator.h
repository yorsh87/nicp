#pragma once

#include "definitions.h"
#include "homogeneousvector4f.h"

namespace nicp {

  /** \class PointAccumulator pointaccumulator.h "pointaccumulator.h"
   *  \brief Class to be used as point accumulator.
   *  
   *  This class provides the operators and the structures to accumulate points,
   *  in particular accumulates the sum and the squared sum of the points.
   */
  class PointAccumulator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
    /**
     *  Empty constructor.
     */
    inline PointAccumulator() { clear(); }
    
    /**
     *  Destructor.
     */
    virtual ~PointAccumulator() {}

    /**
     *  This method reset to zero the sum of the points and put a zero 4x4 matrix in
     *  the squared sum of the points.
     */
    inline void clear() {
      _sum.setZero();
      _squaredSum.setZero();
    }
  
    /**
     *  This methods implements the plus operator between two PointAccumulator.
     */
    inline void operator +=(const PointAccumulator &pa) {
      _sum += pa._sum;
      _squaredSum += pa._squaredSum;
    }
  
    /**
     *  This methods implements the minus operator between two PointAccumulator.
     */
    inline void operator -=(const PointAccumulator &pa) {
      _sum -= pa._sum;
      _squaredSum -= pa._squaredSum;
    }

    /**
     *  This methods implements the plus operator between a PointAccumulator and a Point.
     */
    inline void operator +=(const Point &v) {
      _sum += (Eigen::Vector4f&)v;
      _squaredSum += v * v.transpose();
    }

    /**
     *  This method returns the mean of the accumulated points.
     *  @return the mean of the accumulated points.
     *  @see covariance()
     */
    inline Point mean() const { 
      const float &d = _sum.coeff(3, 0);
      if(d)
	return Point(_sum * (1.0f / d));
      return Point::Zero();
    }

    /**
     *  This method returns the covariance matrix of the accumulated points.
     *  @return the covariance matrix of the accumulated points.
     *  @see mean()
     */
    inline Eigen::Matrix4f covariance() const { 
      float d = _sum.coeff(3, 0);
      if(d) {
	d = 1.0f / d;
	Eigen::Vector4f mean_ = _sum * d;
	return _squaredSum * d - mean_ * mean_.transpose();
      }
      return Eigen::Matrix4f::Zero(); 
    }

    /**
     *  This method returns the sum of the accumulated points.
     *  @return the sum of the accumulated points.
     *  @see squaredSum()
     *  @see n()
     */
    inline const Eigen::Vector4f& sum() const { return _sum; }
  
    /**
     *  This method returns the squared sum of the accumulated points.
     *  @return the squared sum of the accumulated points.
     *  @see sum()
     *  @see n()
     */
    inline const Eigen::Matrix4f& squaredSum() const { return _squaredSum; }

    /**
     *  This method returns the number of accumulated points.
     *  @return the number of accumulated points.
     *  @see sum()
     *  @see squaredSum()
     */  
    inline int n() const { return _sum.coeff(3, 0); }

  protected:
    Eigen::Vector4f _sum; /**< Sum of the points accumulated. */
    Eigen::Matrix4f  _squaredSum; /**< Squared sum of the points accumulated. */
  };

}

