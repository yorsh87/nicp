#pragma once

#include "homogeneousvector4f.h"

namespace pwn {

  /** \struct InformationMatrix informationmatrix.h "informationmatrix.h"
   *  \brief Class that can be used to represent a 3x3 information matrix.
   *  
   *  This class is a structure thought to contain a 3x3 information matrix. It
   *  also provides some operators that can be applied to the infromation matrix.
   */
  struct InformationMatrix : public Eigen::Matrix4f {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Empty constructor.
     *  This constructor creates an InformationMatrix and set its values to zero.
     */
    inline InformationMatrix() : Eigen::Matrix4f() { setZero(); }

    /**
     *  This constructor creates an InformationMatrix using the 4x4 input matrix.
     *  @param other is the 4x4 input matrix used to fill the InformationMatrix.
     */
    inline InformationMatrix(const Eigen::Matrix4f &other) : Eigen::Matrix4f(other) {
      block<1, 4>(3, 0).setZero();
      block<4, 1>(0, 3).setZero();
    }

    /**
     *  This constructor creates an InformationMatrix using the input Eigen::MatrixBase object.
     *  @param other is the input Eigen::MatrixBase object used to fill the InformationMatrix.
     */
    template<typename OtherDerived>
      inline InformationMatrix(const Eigen::MatrixBase<OtherDerived> &other) : Eigen::Matrix4f(other) {
      block<1, 4>(3, 0).setZero();
      block<4, 1>(0, 3).setZero();
    }
    
    /**
     *  Destructor.
     */
    virtual ~InformationMatrix() {}
    
    /**
     *  This methods implements the operator equal between an InformationMatrix and an Eigen::MatrixBase object.
     */
    template<typename OtherDerived>
      inline InformationMatrix& operator = (const Eigen::MatrixBase<OtherDerived> &other) {
      this->Eigen::Matrix4f::operator=(other);
      block<1, 4>(3, 0).setZero();
      block<4, 1>(0, 3).setZero();
      return *this;
    }

    /**
     *  This methods apply the input Eigen::MatrixBase transformation to the InformationMatrix.
     *  @param other is the input Eigen::MatrixBase transformation to apply to the InformationMatrix.
     *  @return the transformed InformationMatrix.
     *  @see transformInPlace()
     */
    template<typename OtherDerived>
      inline InformationMatrix transform(const Eigen::MatrixBase<OtherDerived> &other) const {
      Eigen::Matrix4f T = other;
      T.block<1, 4>(3, 0).setZero();
      T.block<4, 1>(0, 3).setZero();
      InformationMatrix s(T*(*this)*T.transpose());
      return s;
    }

    /**
     *  This methods apply the input Eigen::MatrixBase transformation to the current InformationMatrix.
     *  @param other is the input Eigen::MatrixBase transformation to apply to the InformationMatrix.
     *  @see transform()
     */
    template<typename OtherDerived>
      inline InformationMatrix& transformInPlace(const Eigen::MatrixBase<OtherDerived> &other) const {
      const Eigen::Matrix3f& R = other.block<3,3>(0,0);
      block<3, 3>(0, 0) = R * block<3, 3>(0, 0) * R.transpose(); 
      return *this;
    }

  };

  /** \class InformationMatrixVector informationmatrix.h "informationmatrix.h"
   *  \brief Class that can be used to represent a 3x3 information matrix vector.
   *  
   *  This class provides a structure to handle creation and manipulation of a vector
   *  of InformationMatrix.
   */
  class InformationMatrixVector : public TransformableVector<InformationMatrix> {
  public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Empty constructor.
     *  This constructor creates an InformationMatrixVector of size zero.
     */
    inline InformationMatrixVector() : TransformableVector<InformationMatrix>() {}
    
    /**
     *  Destructor.
     */
    virtual ~InformationMatrixVector() {}

    /**
     *  This methods apply the input transformation to each InformationMatrix contained in the InformationMatrixVector.
     *  @param m is the input transformation to apply to the elements of the InformationMatrixVector.
     */
    template<typename OtherDerived>
      inline void transformInPlace(const OtherDerived &m) {
      Eigen::Matrix4f T = m;
      T.row(3).setZero();
      T.col(3).setZero();
      Eigen::Matrix4f Tt = T.transpose();
      InformationMatrix *t = &(*this)[0];
      for (size_t i = 0; i < size(); i++, t++) {
	*t = T * (*t) * Tt;
      }
    }

  };

  /** \typedef Matrix6f
   * \brief A 6x6 float matrix.
   */
  typedef Eigen::Matrix<float, 6, 6> Matrix6f;
  
  /** \typedef Matrix6fVector
   * \brief A vector of Matrix6f.
   */
  typedef std::vector<Matrix6f, Eigen::aligned_allocator<Matrix6f> > Matrix6fVector;

}
