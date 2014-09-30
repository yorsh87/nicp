#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <iostream>

/** \struct TransformableVector
 *  \brief Base class for the creation and manipulation of vectors of objects.
 *
 *  This class allows to create and manipulate a vector of object where the type of the objects composing
 *  the vector is specified inside the template. The only manipulation allowed is the transformation of all
 *  obejcts composing the vector by means of a matrix multiplication. 
*/

template <typename Transformable>
struct TransformableVector: public std::vector<Transformable, Eigen::aligned_allocator<Transformable> > {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  /**
   *  Empty constructor.
   *  This constructor creates an TransformableVector object using the empty constructor of 
   *  the std vector class and imposing the types of the objects to use to be equal to the type
   *  specified inside the template.
   */
  TransformableVector() : std::vector<Transformable, Eigen::aligned_allocator<Transformable> >() {}
  
  /**
   *  Constructor specifying the vector dimension.
   *  This constructor creates an TransformableVector object using the constructor of 
   *  the std vector class and imposing the types of the objects to use to be equal to the type
   *  specified inside the template. The dimension of the vector is initialized to the value given in input.
   *  @param s is the size value used to initialize the vector.
   */
  TransformableVector(size_t s) : std::vector<Transformable, Eigen::aligned_allocator<Transformable> >(s) {}
  
  /**
   *  This method transforms all the objects inside the vector by multiplying each element with the object given
   *  in input. The type of the input object is defined inside the template.
   *  @param m is a generic object used to transform the vector elements via multiplication. The type of this 
   *  object must be defined inside the template.
   */
  template<typename OtherDerived>
    inline void transformInPlace(const OtherDerived& m) {
    Transformable* t = &(*this)[0];
    for (size_t i = 0; i < std::vector<Transformable, Eigen::aligned_allocator<Transformable> >::size(); i++, t++) {
      *t = m * (*t);
    }
  }

  /**
   *  This method transforms all the objects inside the vector by multiplying each element with the object m given
   *  in input. The type of m must be defined inside the template. The original vector is not modified, the result is
   *  inserted in the TransformableVector dest given in input.
   *  @param dest is the destination vector where the trasformed elements are saved.
   *  @param m is a generic object used to transform the vector elements via multiplication. The type of this 
   *  object must be defined inside the template.
   */
  template<typename OtherDerived>
  inline void transform(TransformableVector& dest, const OtherDerived& m) const {
    dest.resize(this->size());
    const Transformable* tSrc= &(*this)[0];
    Transformable* tDest= &dest[0];
    for (size_t i = 0; i < std::vector<Transformable, Eigen::aligned_allocator<Transformable> >::size(); ++i, ++tSrc, ++tDest ) {
      *tDest = m * (*tSrc);
    }
  }

};
