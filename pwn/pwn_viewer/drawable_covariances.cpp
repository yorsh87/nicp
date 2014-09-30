#include "drawable_covariances.h"
#include "gl_parameter_covariances.h"
#include "pwn_qglviewer.h"
#include "opengl_primitives.h"

namespace pwn_viewer {

  DrawableCovariances::DrawableCovariances() : Drawable() {
    _parameter = 0;
    _covariances = 0;
    _viewer = 0;
    _covarianceDrawList = glGenLists(1);
    _sphereDrawList = glGenLists(1);
    glNewList(_sphereDrawList, GL_COMPILE);
    drawSphere(1.0f);
    glEndList();
    updateCovarianceDrawList();
  }

  DrawableCovariances::DrawableCovariances(Eigen::Isometry3f transformation_, GLParameter *parameter_, StatsVector *covariances_) : Drawable(transformation_) {
    setParameter(parameter_);
    _covariances = covariances_;
    _covarianceDrawList = glGenLists(1);
    _sphereDrawList = glGenLists(1);
    glNewList(_sphereDrawList, GL_COMPILE);
    drawSphere(1.0f);
    glEndList();
    updateCovarianceDrawList();
  }

  bool DrawableCovariances::setParameter(GLParameter *parameter_) {
    GLParameterCovariances *covariancesParameter = (GLParameterCovariances*)parameter_;
    if (covariancesParameter == 0) {
      _parameter = 0;
      return false;
    }
    _parameter = covariancesParameter;
    return true;
  }

  void DrawableCovariances::draw() {
    GLParameterCovariances *covariancesParameter = dynamic_cast<GLParameterCovariances*>(_parameter);
    if(_covarianceDrawList &&
       covariancesParameter && 
       covariancesParameter->show() && 
       covariancesParameter->ellipsoidScale() > 0.0f) {
      glPushMatrix();
      glMultMatrixf(_transformation.data());
      glCallList(_covarianceDrawList);
      glPopMatrix();
    }
  }

  void DrawableCovariances::updateCovarianceDrawList() {
    GLParameterCovariances *covariancesParameter = dynamic_cast<GLParameterCovariances*>(_parameter);
    glNewList(_covarianceDrawList, GL_COMPILE); 
    if(_covarianceDrawList &&
       _covariances && 
       covariancesParameter && 
       covariancesParameter->ellipsoidScale() > 0.0f) {
      covariancesParameter->applyGLParameter();
      float ellipsoidScale = covariancesParameter->ellipsoidScale();
      // Find max curvature
      float maxCurvature = 0.0f;
      for(size_t i = 0; i < _covariances->size(); i += covariancesParameter->step()) {
	if(_covariances->at(i).curvature() > maxCurvature)
	  maxCurvature = _covariances->at(i).curvature();
      }
      maxCurvature = maxCurvature / 2.0f ;
      for(size_t i = 0; i < _covariances->size(); i += covariancesParameter->step()) {
	Stats cov = _covariances->at(i);
	Eigen::Vector3f lambda = cov.eigenValues();
	Eigen::Isometry3f I = Eigen::Isometry3f::Identity();
	I.linear() = cov.eigenVectors();
	if(cov.n() == 0)
	  continue;
	I.translation() = Eigen::Vector3f(cov.mean()[0], cov.mean()[1], cov.mean()[2]);
	float sx = sqrt(lambda[0]) * ellipsoidScale;
	float sy = sqrt(lambda[1]) * ellipsoidScale;
	float sz = sqrt(lambda[2]) * ellipsoidScale;
	float curvature = cov.curvature();
	glPushMatrix();
	glMultMatrixf(I.data());
	//std::cerr << "Curvature: " << curvature << std::endl;
	glColor4f(curvature/maxCurvature, 1.0f - curvature/maxCurvature, 0.0f, 1.0f);
	sx = 1e-03;
	sy = ellipsoidScale;
	sz = ellipsoidScale;
	glScalef(sx, sy, sz);
	glCallList(_sphereDrawList);
	glPopMatrix();	    
      }   
    }
    glEndList();
  }

}
