#include "drawable_points.h"

namespace pwn_viewer {

  DrawablePoints::DrawablePoints() : Drawable() {
    _parameter = 0;
    _points = 0;
    _normals = 0;
    _rgbs = 0;    
    _pointDrawList = 0;
    _pointDrawList = glGenLists(1);
    updatePointDrawList();
  }

  DrawablePoints::DrawablePoints(const Eigen::Isometry3f& transformation_, GLParameter *parameter_, PointVector *points_,  
				 NormalVector *normals_, RGBVector *rgbs_) : Drawable(transformation_) {
    setParameter(parameter_);
    _points = points_;
    _normals = normals_;
    _rgbs = rgbs_;
    _pointDrawList = 0;
    _pointDrawList = glGenLists(1);
    updatePointDrawList();
  }

  bool DrawablePoints::setParameter(GLParameter *parameter_) {
    GLParameterPoints *pointsParameter = (GLParameterPoints*)parameter_;
    if(pointsParameter == 0) {
      _parameter = 0;
      return false;
    }
    _parameter = pointsParameter;
    return true;
  }

  void DrawablePoints::draw() {
    GLParameterPoints *pointsParameter = dynamic_cast<GLParameterPoints*>(_parameter);
    if(_pointDrawList &&
       pointsParameter && 
       pointsParameter->show() && 
       pointsParameter->pointSize() > 0.0f) {
      glPushMatrix();
      glMultMatrixf(_transformation.data());
      glCallList(_pointDrawList);
      glPopMatrix();
    }
  }

  void DrawablePoints::updatePointDrawList() {
    GLParameterPoints *pointsParameter = dynamic_cast<GLParameterPoints*>(_parameter);
    glNewList(_pointDrawList, GL_COMPILE);    
    float scale = 1./255;
    if(_pointDrawList &&
       _points && 
       _normals &&
       pointsParameter && 
       pointsParameter->pointSize() > 0.0f) {
      pointsParameter->applyGLParameter();
      glBegin(GL_POINTS);
      for(size_t i = 0; i < _points->size(); i += pointsParameter->step()) {      	
	const Point &p = _points->at(i);
      	const Normal &n = _normals->at(i);
	// if(n.z() < -0.5f) { continue; }
	if(_rgbs && _rgbs->size() > 0) { 
	  cv::Vec3b c=_rgbs->at(i);
	  glColor3f(scale*c[0], scale*c[1], scale*c[2]);
	}
	glColor4f(fabs(n[0]), fabs(n[1]), fabs(n[2]), 1.0f);
      	glNormal3f(n[0], n[1], n[2]);
      	glVertex3f(p[0], p[1], p[2]);
      }    
      glEnd();
    }
    glEndList();
  }

}
