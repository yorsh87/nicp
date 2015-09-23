#pragma once

#include "aligner.h"
#include "correspondencefindernn.h"

namespace nicp {

  /** \class Aligner aligner.h "aligner.h"
   *  \brief Class for point cloud alignment.
   *
   *  This class allows to compute the homogeneous transformation that brings
   *  a point cloud to superpose an other reference point cloud. Data association
   *  is computed via nearest neighbor.
   */
class AlignerNN : public Aligner {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Empty constructor.
     *  This constructor creates a Projective Aligner with default values for all its attributes.
     *  All the pointers to objects implementing an algorithm have to be set since
     *  this constructor sets them to zero.
     */
    AlignerNN() : Aligner() {}

    /**
     *  This constructor creates an Aligner with default values for all its attributes.
     *  All internal algorithms are set using the input values.
     *  @param projector_ is a pointer to the point projector that will be used by the Aligner.
     *  @param linearizer_ is a pointer to the linearizer that will be used by the Aligner.
     *  @param correspondenceFinder_ is a pointer to the point correspondence finder that will be used by the Aligner.
     */
    AlignerNN(PointProjector* projector_, Linearizer* linearizer_, CorrespondenceFinderNN* correspondenceFinder_) : Aligner(projector_, linearizer_, correspondenceFinder_) {}

    /**
     *  Destructor.
     */
    virtual ~AlignerNN() {}

    /**
     *  This method computes the final transformation that brings the cloud to align to superpose the reference
     *  cloud.
     */
    virtual void align();

  };

}
