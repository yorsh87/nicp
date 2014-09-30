#include "drawable_uncertainty.h"
#include "pwn_qglviewer.h"
#include "opengl_primitives.h"

#include <Eigen/Eigenvalues>

namespace pwn_viewer {

  DrawableUncertainty::DrawableUncertainty() : Drawable() {
    _parameter = 0;
    _covariances = 0;
    _covarianceDrawList = glGenLists(1);
    _sphereDrawList = glGenLists(1);
    glNewList(_sphereDrawList, GL_COMPILE);
    drawSphere(1.0f);
    glEndList();
    updateCovarianceDrawList();
  }

  DrawableUncertainty::DrawableUncertainty(Eigen::Isometry3f transformation_, GLParameter *parameter_, Gaussian3fVector *covariances_) : Drawable(transformation_) {
    setParameter(parameter_);
    _covariances = covariances_;
    _covarianceDrawList = glGenLists(1);
    _sphereDrawList = glGenLists(1);
    glNewList(_sphereDrawList, GL_COMPILE);
    drawSphere(1.0f);
    glEndList();
    updateCovarianceDrawList();
  }

  bool DrawableUncertainty::setParameter(GLParameter *parameter_) {
    GLParameterUncertainty *uncertaintyParameter = (GLParameterUncertainty*)parameter_;
    if(uncertaintyParameter == 0) {
      _parameter = 0;
      return false;
    }
    _parameter = uncertaintyParameter;
    return true;
  }

  void DrawableUncertainty::draw() {
    GLParameterUncertainty *uncertaintyParameter = dynamic_cast<GLParameterUncertainty*>(_parameter);
    if(_covarianceDrawList &&
       uncertaintyParameter && 
       uncertaintyParameter->show() && 
       uncertaintyParameter->ellipsoidScale() > 0.0f) {
      glPushMatrix();
      glMultMatrixf(_transformation.data());
      glCallList(_covarianceDrawList);
      glPopMatrix();
    }
  }

  void DrawableUncertainty::updateCovarianceDrawList() {
    GLParameterUncertainty *uncertaintyParameter = dynamic_cast<GLParameterUncertainty*>(_parameter);
    glNewList(_covarianceDrawList, GL_COMPILE); 
    if(_covarianceDrawList &&
       _covariances && 
       uncertaintyParameter && 
       uncertaintyParameter->ellipsoidScale() > 0.0f) {
      uncertaintyParameter->applyGLParameter();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver;
      float ellipsoidScale = uncertaintyParameter->ellipsoidScale();
      for(size_t i = 0; i < _covariances->size(); i += uncertaintyParameter->step()) {
	Gaussian3f &gaussian3f = _covariances->at(i);
	Eigen::Matrix3f covariance = gaussian3f.covarianceMatrix();
	Eigen::Vector3f mean = gaussian3f.mean();
	eigenSolver.computeDirect(covariance, Eigen::ComputeEigenvectors);
	Eigen::Vector3f eigenValues = eigenSolver.eigenvalues();      
	Eigen::Isometry3f I = Eigen::Isometry3f::Identity();
	I.linear() = eigenSolver.eigenvectors();
	I.translation() = mean;
	float sx = sqrt(eigenValues[0]) * ellipsoidScale;
	float sy = sqrt(eigenValues[1]) * ellipsoidScale;
	float sz = sqrt(eigenValues[2]) * ellipsoidScale;
	glPushMatrix();
	glMultMatrixf(I.data());	
	sx = sx;
	sy = sy;
	sz = sz;
	glScalef(sx, sy, sz);
	glCallList(_sphereDrawList);
	glPopMatrix();	    
      }   
    }
    glEndList();
  }

}
