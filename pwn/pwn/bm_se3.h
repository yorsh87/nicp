#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "bm_defs.h"

namespace pwn {

  template <typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 3> quat2mat(const Eigen::MatrixBase<Derived>& q) {
    const typename Derived::Scalar& qx = q.x();
    const typename Derived::Scalar& qy = q.y();
    const typename Derived::Scalar& qz = q.z();
    typename Derived::Scalar qw = sqrt(1.f - q.squaredNorm());
    Eigen::Matrix<typename Derived::Scalar, 3, 3> R;
    R << qw*qw + qx*qx - qy*qy - qz*qz, 2*(qx*qy - qw*qz) , 2*(qx*qz + qw*qy),
      2*(qx*qy + qz*qw) , qw*qw - qx*qx + qy*qy - qz*qz, 2*(qy*qz - qx*qw),
      2*(qx*qz - qy*qw) , 2*(qy*qz + qx*qw), qw*qw - qx*qx - qy*qy + qz*qz;
  
    return R;
  }


  template <typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 1> mat2quat(const Eigen::MatrixBase<Derived>& R) {
    Eigen::Quaternion<typename Derived::Scalar> q(R); 
    q.normalize();
    Eigen::Matrix<typename Derived::Scalar,3,1> rq;
    rq << q.x(), q.y(), q.z();
    if (q.w()<0){
      rq = -rq;
    }
    return rq;
  }

  template <typename Derived>
    inline Eigen::Transform<typename Derived::Scalar, 3, Eigen::Isometry> v2t(const Eigen::MatrixBase<Derived>& x_) {
    Eigen::Transform<typename Derived::Scalar, 3, Eigen::Isometry> X;
    Eigen::Matrix<typename Derived::Scalar, 6, 1> x(x_);
    X.template linear() = quat2mat(x.template block<3,1>(3,0));
    X.template translation() = x.template block<3,1>(0,0);
    return X;
  }

  template <typename Scalar>
    inline Eigen::Matrix<Scalar, 6, 1> t2v(const Eigen::Transform<Scalar, 3, Eigen::Isometry>& X)
    {
      Eigen::Matrix<Scalar,6,1> v;  
      v.template block<3,1>(0,0) = X.translation();
      v.template block<3,1>(3,0) = mat2quat(X.linear());
      return v;
    }

  template <typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, Eigen::MatrixBase<Derived>::RowsAtCompileTime, Eigen::MatrixBase<Derived>::RowsAtCompileTime> skew(const Eigen::MatrixBase<Derived>& v)
    {
      typename Derived::Scalar tx = 2*v.x();
      typename Derived::Scalar ty = 2*v.y();
      typename Derived::Scalar tz = 2*v.z();
      Eigen::Matrix<typename Derived::Scalar, Eigen::MatrixBase<Derived>::RowsAtCompileTime, Eigen::MatrixBase<Derived>::RowsAtCompileTime> S = Eigen::Matrix<typename Derived::Scalar, Eigen::MatrixBase<Derived>::RowsAtCompileTime, Eigen::MatrixBase<Derived>::RowsAtCompileTime>::Zero();
      S.coeffRef(0,1)= tz;  S.coeffRef(1,0)=-tz;
      S.coeffRef(0,2)=-ty;  S.coeffRef(2,0)= ty;
      S.coeffRef(1,2)= tx;  S.coeffRef(2,1)=-tx;

      return S;
    }

  template <typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 12, 1> homogeneous2vector(const Eigen::MatrixBase<Derived>& transform){
    Eigen::Matrix<typename Derived::Scalar, 12, 1> x;
    x.template block<3,1>(0,0)=transform.template block<1,3>(0,0).transpose();
    x.template block<3,1>(3,0)=transform.template block<1,3>(1,0).transpose();
    x.template block<3,1>(6,0)=transform.template  block<1,3>(2,0).transpose();
    x.template block<3,1>(9,0)=transform.template block<3,1>(0,3);
    return x;
  }


  template <typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 4, 4> vector2homogeneous(const Eigen::MatrixBase<Derived>& x){
    Eigen::Matrix<typename Derived::Scalar, 4, 4> transform= Eigen::Matrix<typename Derived::Scalar, 4, 4>::Identity();
    transform.template block<1,3>(0,0)=x.template block<3,1>(0,0).transpose();
    transform.template block<1,3>(1,0)=x.template block<3,1>(3,0).transpose();
    transform.template block<1,3>(2,0)=x.template block<3,1>(6,0).transpose();
    transform.template block<1,3>(3,0)=x.template block<3,1>(9,0);
    return transform;
  }


  inline Eigen::Matrix4f skew4f(const Eigen::Vector3f& v)
  {
    const float& tx = v.x();
    const float& ty = v.y();
    const float& tz = v.z();
    Eigen::Matrix4f S;
    S << 0, (2*tz), (-2*ty), 0,
      (-2*tz), 0, (2*tx), 0,				
      (2*ty),  (-2*tx), 0, 0,
      0, 0, 0, 0;
    return S;
  }

  inline float t2angle(const Eigen::Isometry3d& iso){
    Eigen::AngleAxisd aa(iso.linear());
    float angle = aa.angle();
    if (aa.axis().z()<0){
      angle=-angle;
    }
    return angle;
  }

}
