#include "gl_parameter_cloud.h"

namespace pwn_viewer {

  GLParameterCloud::GLParameterCloud(int step) : GLParameter() {
    _parameterPoints = new GLParameterPoints(1.0f, Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
    _parameterNormals = new GLParameterNormals(1.0f, Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f), 0.0f);
    _parameterCovariances = new GLParameterCovariances(1.0f, 
						       Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f), Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f),
						       0.02f, 0.0f);
    _parameterCorrespondences = new GLParameterCorrespondences(1.0f, Eigen::Vector4f(1.0f, 0.0f, 1.0f, 1.0f), 0.0f);
  
    _parameterPoints->setStep(step);
    _parameterNormals->setStep(step);
    _parameterCovariances->setStep(step);
    _parameterCorrespondences->setStep(step);  
  }

  GLParameterCloud::~GLParameterCloud() {
    delete _parameterPoints;
    delete _parameterNormals;
    delete _parameterCovariances;    
    delete _parameterCorrespondences;
  }

  void GLParameterCloud::setStep(int step_) {
    _step = step_;
    _parameterPoints->setStep(step_);
    _parameterNormals->setStep(step_);
    _parameterCovariances->setStep(step_);
    _parameterCorrespondences->setStep(step_);  
  }

  void GLParameterCloud::setPointSize(float pointSize) {
    _parameterPoints->setPointSize(pointSize);
    _parameterNormals->setPointSize(pointSize);
    _parameterCovariances->setPointSize(pointSize);
    _parameterCorrespondences->setPointSize(pointSize);  
  }

  void GLParameterCloud::setShow(bool show_) {
    _show = show_;
    _parameterPoints->setShow(show_);
    _parameterNormals->setShow(show_);
    _parameterCovariances->setShow(show_);
    _parameterCorrespondences->setShow(show_);  
  }

}
