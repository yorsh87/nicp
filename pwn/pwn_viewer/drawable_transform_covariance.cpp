#include <Eigen/Eigenvalues> 

#include "drawable_transform_covariance.h"
#include "gl_parameter_covariances.h"
#include "pwn_qglviewer.h"
#include "opengl_primitives.h"

namespace pwn_viewer {

  DrawableTransformCovariance::DrawableTransformCovariance() : Drawable() {
    _parameter = 0;
    _covariance.setZero();
    _mean.setZero();
    _viewer = 0;
    _covarianceDrawList = glGenLists(1);
    _sphereDrawList = glGenLists(1);
    glNewList(_sphereDrawList, GL_COMPILE);
    drawSphere(1.0f);
    glEndList();
    updateCovarianceDrawList();
  }

  DrawableTransformCovariance::DrawableTransformCovariance(Eigen::Isometry3f transformation_, GLParameter *parameter_, Eigen::Matrix3f covariance_, Eigen::Vector3f mean_) : Drawable(transformation_) {
    setParameter(parameter_);
    _covariance = covariance_;
    _mean = mean_;
    _covarianceDrawList = glGenLists(1);
    _sphereDrawList = glGenLists(1);
    glNewList(_sphereDrawList, GL_COMPILE);
    drawSphere(1.0f);
    glEndList();
    updateCovarianceDrawList();
  }

  bool DrawableTransformCovariance::setParameter(GLParameter *parameter_) {
    GLParameterTransformCovariance *covarianceParameter = dynamic_cast<GLParameterTransformCovariance*>(parameter_);
    if (covarianceParameter == 0) {
      _parameter = 0;
      return false;
    }
    _parameter = covarianceParameter;
    updateCovarianceDrawList();
    return true;
  }

  void DrawableTransformCovariance::draw() {
    GLParameterTransformCovariance *covarianceParameter = dynamic_cast<GLParameterTransformCovariance*>(_parameter);
    if(covarianceParameter && 
       covarianceParameter->show() && 
       covarianceParameter->scale() > 0.0f) {
      glPushMatrix();
      glMultMatrixf(_transformation.data());
      covarianceParameter->applyGLParameter();
      glCallList(_covarianceDrawList);
      glPopMatrix();
    }
  }

  void DrawableTransformCovariance::updateCovarianceDrawList() {
    GLParameterTransformCovariance *covarianceParameter = dynamic_cast<GLParameterTransformCovariance*>(_parameter);
    glNewList(_covarianceDrawList, GL_COMPILE); 
    if(_covariance != Eigen::Matrix3f::Zero() && 
       covarianceParameter && 
       covarianceParameter->show() && 
       covarianceParameter->scale() > 0.0f) {
      float scale = covarianceParameter->scale();
      Eigen::Vector4f color = covarianceParameter->color();
      
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver;
      eigenSolver.computeDirect(_covariance, Eigen::ComputeEigenvectors);

      Eigen::Vector3f lambda = eigenSolver.eigenvalues();      
      Eigen::Isometry3f I = Eigen::Isometry3f::Identity();
      I.linear() = eigenSolver.eigenvectors();
      I.translation() = Eigen::Vector3f(_mean.x(), _mean.y(), _mean.z());
      
      float sx = sqrt(lambda[0]) * scale;
      float sy = sqrt(lambda[1]) * scale;
      float sz = sqrt(lambda[2]) * scale;
      
      glPushMatrix();
      glMultMatrixf(I.data());
      glColor4f(color[0], color[1], color[2], color[3]);
      glScalef(sx, sy, sz);
      glCallList(_sphereDrawList);
      glPopMatrix();	    
    }
    glEndList();
  }

}
