#include "drawable_cloud.h"
#include "opengl_primitives.h"

namespace pwn_viewer {

  DrawableCloud::DrawableCloud(const Eigen::Isometry3f &transformation_, GLParameter *parameter_, 
			       Cloud *cloud_) : Drawable(transformation_) {
    setParameter(parameter_);
    _cloud = cloud_;
    _drawablePoints = 0;
    _drawableCorrespondences = 0;
    _drawableCovariances = 0;
    _drawableCorrespondences = 0;
    _previousDrawableCloud = 0;
    constructDrawableObjects();
  }

  bool DrawableCloud::setParameter(GLParameter *parameter_) {
    GLParameterCloud *cloudParameter = (GLParameterCloud*)parameter_;
    if(cloudParameter == 0) {
      _parameter = 0;
      return false;
    }
    _parameter = cloudParameter;
    return true;
  }

  void DrawableCloud::clearDrawableObjects() {
    if (! _cloud)
      return;
    if(_drawablePoints)
      delete _drawablePoints;
    if(_drawableNormals)
      delete _drawableNormals;
    if(_drawableCovariances)
      delete _drawableCovariances;
    if(_drawableCorrespondences)
      delete _drawableCorrespondences;
    _drawablePoints = 0;
    _drawableNormals = 0;
    _drawableCovariances = 0;
    _drawableCorrespondences = 0;
  }

  void DrawableCloud::constructDrawableObjects(){
    if(_cloud) {
      _drawablePoints = new DrawablePoints(Eigen::Isometry3f::Identity(), 
					   (GLParameter*)_parameter->parameterPoints(), &_cloud->points(), &_cloud->normals());
      if (_cloud->rgbs().size()) {
	_drawablePoints->setRGBs(&(_cloud->rgbs())); 
      }
      _drawableNormals = new DrawableNormals(Eigen::Isometry3f::Identity(), 
					     (GLParameter*)_parameter->parameterNormals(), &_cloud->points(), &_cloud->normals());
      _drawableCovariances = new DrawableCovariances(Eigen::Isometry3f::Identity(), 
						     (GLParameter*)_parameter->parameterCovariances(), &_cloud->stats());
      _drawableCorrespondences = new DrawableCorrespondences();
      _drawableCorrespondences->setParameter((GLParameter*)_parameter->parameterCorrespondences());
    }
  }

  void DrawableCloud::setCloud(Cloud *f) {
    if(f && f != _cloud) {
      clearDrawableObjects();
      _cloud = f;
      constructDrawableObjects();
    }
  }

  void DrawableCloud::draw() {
    if(_parameter->show() && _cloud) {
      glPushMatrix();
      glMultMatrixf(_transformation.data());
      if(_drawablePoints)
	_drawablePoints->draw();
      if(_drawableNormals)
	_drawableNormals->draw();
      if(_drawableCovariances)
	_drawableCovariances->draw();
      if(_drawableCorrespondences)
	_drawableCorrespondences->draw();

      glPushMatrix();
      Eigen::Isometry3f sensorOffset;
      sensorOffset.translation() = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
      Eigen::Quaternionf quaternion = Eigen::Quaternionf(-.5f, -0.5f, 0.5f, 0.5f);
      sensorOffset.linear() = quaternion.toRotationMatrix();
      sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      glMultMatrixf(sensorOffset.data());
      glColor4f(1,0,0,0.5);
      glPopMatrix();

      glPopMatrix();
    }
  }

}
