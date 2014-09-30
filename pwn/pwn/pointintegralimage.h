#pragma once 

#include "pointaccumulator.h"

namespace pwn {

  /** \class PointIntegralImage pointintegralimage.h "pointintegralimage.h"
   *  \brief Class for integral image computation.
   *  
   *  This class extends the Eigen::Matrix class and implements all the methods necessary
   *  to compute an integral image of the points in a depth image. The elements of a PointIntegralImage
   *  are PointAccumulator.
   */
  class PointIntegralImage : public Eigen::Matrix<PointAccumulator, Eigen::Dynamic, Eigen::Dynamic> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Empty constructor.
     *  This constructor creates a PointIntegralImage of zero size.
     */
    PointIntegralImage() : Eigen::Matrix<PointAccumulator, Eigen::Dynamic, Eigen::Dynamic>(0, 0) {}

    /**
     *  Destructor.
     */
    virtual ~PointIntegralImage() {}

    /**
     *  This method computes the integral image, filling the PointIntegralImage matrix, starting from the vector of points 
     *  and the associated index image.
     *  @param pointIndices is the IndexImage associated to the vector of points.
     *  @param points is the vector of points.
     */
    void compute(const IntImage &pointIndices, const PointVector &points);
  
    /**
     *  This method clear the PointIntegralImage matrix.
     */
    void clear();
  
    /**
     *  This method returns the PointAccumulator associated to a certain region of the PointIntegralImage matrix.
     *  @param xmin is the smaller row index of the region to compute.
     *  @param xmax is the larger row index of the region to compute.
     *  @param ymin is the smaller lower column index of the region to compute.
     *  @param ymax is the larger lower column index of the region to compute.
     *  @return the PointAccumulator of the input region.
     */
    PointAccumulator getRegion(int xmin, int xmax, int ymin, int ymax);
    
  protected:
    /**
     *  This method check if the input parameter v is inside the interval [min;max].
     *  @param v is the input parameter to check.
     *  @param min is lower bound.
     *  @param max is upper bound.
     *  @return v if it is inside the interval [min;max], otherwise it returns the 
     *  lower/upper bound depending which one was exceded.
     */
    inline int _clamp(int v, int min, int max);
  };

}
