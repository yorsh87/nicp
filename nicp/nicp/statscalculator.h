#pragma once

#include "stats.h"
#include "definitions.h"

namespace nicp {

  /** \class StatsCalculator statscalculator.h "statscalculator.h"
   *  \brief Base class interface for point properties computation.
   *  
   *  This class provides an easy interface for point properties computation including
   *  normals computation.
   *  It should be extended in order to implement your own properties to compute.
   */
  class StatsCalculator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Empty constructor.
     */
    StatsCalculator() {}

    /**
     *  Destructor.
     */
    virtual ~StatsCalculator() {}

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
			 const IntImage &indexImage);
  };

}
