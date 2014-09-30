#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "transformable_vector.h"

namespace pwn {

  /** \class HomogeneousVector4f homogeneousvector4f.h "homogeneousvector4f.h"
   *  \brief Class for homogenous points manipulation.
   *  
   *  This class allows to creates and operates on homogeneous points. The homogeneous
   *  corrdinates is specified via template.
   */
  template <int wCoordinate_>
    class HomogeneousVector4f : public Eigen::Vector4f {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    static const float wCoordinate = wCoordinate_;

    /**
     *  Empty constructor.
     */
    inline HomogeneousVector4f() : Eigen::Vector4f() { this->data()[3] = wCoordinate; }

    /**
     *  This constructor creates an homogeneous points with the values of the input
     *  3D vector.
     *  @param other is a 3D vector used to initialize the Homogeneous4fVector.
     */
    inline HomogeneousVector4f(const Eigen::Vector3f &other) {
      this->data()[0] = other.data()[0];
      this->data()[1] = other.data()[1];
      this->data()[2] = other.data()[2];
      this->data()[3] = wCoordinate;
    }

    /**
     *  This constructor creates an homogeneous points with the values of the input object.
     *  @param other is an Eigen::MatrixBase object used to initialize the Homogeneous4fVector.
     */
    template<typename OtherDerived>
      inline HomogeneousVector4f(const Eigen::MatrixBase<OtherDerived> &other) : Eigen::Vector4f(other) { this->data()[3] = wCoordinate; }
    
    /**
     *  Destructor.
     */
    virtual ~HomogeneousVector4f() {}

    /**
     *  This methods implements the operator equal between an Homogeneous4fVector and an Eigen::MatrixBase object.
     */
    template<typename OtherDerived>
      inline HomogeneousVector4f& operator = (const Eigen::MatrixBase<OtherDerived> &other) {
      this->Eigen::Vector4f::operator = (other);
      this->data()[3] = wCoordinate;
      return *this;
    }

    /**
     *  This methods implements the operator equal between an Homogeneous4fVector and a 3D vector.
     */
    inline HomogeneousVector4f& operator = (const Eigen::Vector3f &other) {
      this->data()[0] = other.data()[0];
      this->data()[1] = other.data()[1];
      this->data()[2] = other.data()[2];
      this->data()[3] = wCoordinate;
      return *this;
    }

  };

  /** \typedef Point
   * \brief An Homogeneous4fVector where the homogeneous coordinates is equal to 1.
   */
  typedef HomogeneousVector4f<1> Point;

  /** \typedef Normal
   * \brief An Homogeneous4fVector where the homogeneous coordinates is equal to 0.
   */
  typedef HomogeneousVector4f<0> Normal;

  /** \typedef PointVector
   * \brief A vector of Point.
   */
  typedef TransformableVector<Point> PointVector;

  /** \typedef NormalVector
   * \brief A vector of Normal.
   */
  typedef TransformableVector<Normal> NormalVector;

}
