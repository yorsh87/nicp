#pragma once

#include "definitions.h"
#include "homogeneousvector4f.h"

namespace nicp {

  class PinholePointProjector;

  /** \class Gaussian3fVector gaussian3.h "gaussian3.h"
   *  \brief Class for Gaussian3f vector manipulation.
   *  
   *  This class implements a structure for memorize, create and make operations on
   *  a vector of Gausian3f. A Gaussian is a struct used to represent a Gaussian allowing
   *  to define the dimension at compile time. This struct memorize all the foundamental
   *  informations about a gaussian like the mean, the covariance matrix and the information matrix.
   */
  class Gaussian3fVector : public TransformableVector<Gaussian3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Constructor.
     *  This constructor creates a Gaussian3fVector of dimesion and initialized with the values
     *  given in input.
     *  @param s is the dimension of the vector to create.
     *  @param p is a Gaussian3f to which all the vector elements are initialized.
     */
    Gaussian3fVector(size_t s = 0, const Gaussian3f &p = Gaussian3f());

    /**
     *  Destructor.
     */
    virtual ~Gaussian3fVector() {}

    /**
     *  This method loads and fills the Gaussian3fVector starting from a depth image
     *  and some sensor parameters. The Gaussian3fVector will represents the intrinsic error
     *  of each point due to the sensor uncertainty.
     *  @param depthImage is a reference to the DepthImage used to fill the Gaussian3fVector.
     *  @param pointProjector is a reference to the PinholePointProjector used to unproject the points.
     *  @param dmax is the max depth value for which points over it are discarded.
     *  @param baseline is the horizontal baseline between the cameras used by the depth sensor (in meters).
     *  @param alpha is a little perturbation to apply during the Gaussian3fVector computation.
     *  @see toPointAndNormalVector()
     */
    void fromDepthImage(const DepthImage &depthImage, 
			const PinholePointProjector &pointProjector, 
			float dmax = std::numeric_limits<float>::max(), 
			float baseline = 0.075f, float alpha = 0.1f);
    
    /**
     *  This method updates the points coordinates using the Gaussian3fVector.
     *  @param destPoints is a reference to the vector of Point that will be updated using the Gaussian3fVector.
     *  @param destNormals is a reference to the vector of Normal.
     *  @param eraseNormals is an input boolean parameter, if it is true the vector of Normal will be erased.
     *  @see tfromDepthImage()
     */
    void toPointAndNormalVector(PointVector &destPoints, NormalVector &destNormals, bool eraseNormals = false) const;

    /**
     *  This method applies a transformation like an isometry to the Gaussian3fVector.
     *  @param m is a reference to the transformation to apply to the Gaussian3fVector.
     */
    template<typename OtherDerived>
      inline void transformInPlace(const OtherDerived &m) {
      const Eigen::Matrix4f t = m;
      const Eigen::Matrix3f rotation = t.block<3, 3>(0, 0);
      const Eigen::Vector3f translation = t.block<3, 1>(0, 3);
      for (size_t i = 0; i < size(); ++i) {
	at(i) = Gaussian3f(rotation * at(i).mean() + translation, rotation * at(i).covarianceMatrix() * rotation.transpose(), false);
      }
    }
  };

}
