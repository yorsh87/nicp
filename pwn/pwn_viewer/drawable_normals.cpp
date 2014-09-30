#include "drawable_normals.h"
#include "gl_parameter_normals.h"

namespace pwn_viewer {

  DrawableNormals::DrawableNormals() : Drawable() {
    _parameter = 0;
    _points = 0;
    _normals = 0;
    _normalDrawList = glGenLists(1);
    updateNormalDrawList();
  }

  DrawableNormals::DrawableNormals(const Eigen::Isometry3f &transformation_, GLParameter *parameter_, PointVector *points_, 
				   NormalVector *normals_) : Drawable(transformation_) {
    setParameter(parameter_);
    _points = points_;
    _normals = normals_;
    _normalDrawList = glGenLists(1);
    updateNormalDrawList();
  }

  bool DrawableNormals::setParameter(GLParameter *parameter_) {
    GLParameterNormals *normalsParameter = (GLParameterNormals*)parameter_;
    if (normalsParameter == 0) {
      _parameter = 0;
      return false;
    }
    _parameter = normalsParameter;
    return true;
  }

  void DrawableNormals::draw() {
    GLParameterNormals *normalsParameter = dynamic_cast<GLParameterNormals*>(_parameter);
    if(_normalDrawList &&
       normalsParameter &&
       normalsParameter->show() && 
       normalsParameter->normalLength() > 0.0f) {
      glPushMatrix();
      glMultMatrixf(_transformation.data());
      glCallList(_normalDrawList);
      glPopMatrix();
    }
  }

  void DrawableNormals::updateNormalDrawList() {
    GLParameterNormals *normalsParameter = dynamic_cast<GLParameterNormals*>(_parameter);
    glNewList(_normalDrawList, GL_COMPILE);  
    if(_normalDrawList &&
       _points &&
       _normals && 
       normalsParameter &&
       normalsParameter->normalLength() > 0.0f) {
      normalsParameter->applyGLParameter();
      float normalLength = normalsParameter->normalLength();
      glBegin(GL_LINES);
      for(size_t i = 0; i < _normals->size(); i += normalsParameter->step()) {
	const Point &p = _points->at(i);
	const Normal &n = _normals->at(i);
	glVertex3f(p[0], p[1], p[2]);
	glVertex3f(p[0] + n[0]*normalLength,
		   p[1] + n[1]*normalLength, 
		   p[2] + n[2]*normalLength);
      }
      glEnd();
    }
    glEndList();
  }

}
