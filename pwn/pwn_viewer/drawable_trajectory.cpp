#include "drawable_trajectory.h"
#include "opengl_primitives.h"

namespace pwn_viewer {

  DrawableTrajectory::DrawableTrajectory() : Drawable() {
    _parameter = 0;
    _trajectory = 0;
    _pyramidDrawList = glGenLists(1);
    _trajectoryDrawList = glGenLists(1);
    glNewList(_pyramidDrawList, GL_COMPILE);
    drawPyramid(1.0f, 1.0f);
    glEndList();
    _trajectorySize = 0;
    updateTrajectoryDrawList();
  }

  DrawableTrajectory::DrawableTrajectory(const Eigen::Isometry3f &transformation_, GLParameter *parameter_, 
					 std::vector<Eigen::Isometry3f>* trajectory_, 
					 std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >* trajectoryColors_) : Drawable(transformation_) {
    setParameter(parameter_);
    _trajectory = trajectory_;
    _trajectoryColors = trajectoryColors_;
    _pyramidDrawList = glGenLists(1);
    _trajectoryDrawList = glGenLists(1);
    glNewList(_pyramidDrawList, GL_COMPILE);
    drawPyramid(1.0f, 1.0f);
    glEndList();
    if(_trajectory != 0)
      _trajectorySize = _trajectory->size();
    else
      _trajectorySize = 0;
    updateTrajectoryDrawList();
  }

  bool DrawableTrajectory::setParameter(GLParameter *parameter_) {
    GLParameterTrajectory *trajectoryParameter = (GLParameterTrajectory*)parameter_;
    if(trajectoryParameter == 0) {
      _parameter = 0;
      return false;
    }
    _parameter = trajectoryParameter;
    return true;
  }

  void DrawableTrajectory::draw() {
    GLParameterTrajectory *trajectoryParameter = dynamic_cast<GLParameterTrajectory*>(_parameter);
    if(_trajectory &&
       trajectoryParameter && 
       trajectoryParameter->show() && 
       trajectoryParameter->pyramidScale() > 0.0f) {
      if(_trajectorySize != _trajectory->size())
	updateTrajectoryDrawList();
      glPushMatrix();
      glMultMatrixf(_transformation.data());
      trajectoryParameter->applyGLParameter();
      glCallList(_trajectoryDrawList);
      glPopMatrix();
    }
  }

  void DrawableTrajectory::updateTrajectoryDrawList() {
    GLParameterTrajectory *trajectoryParameter = dynamic_cast<GLParameterTrajectory*>(_parameter);
    glNewList(_trajectoryDrawList, GL_COMPILE);    
    if(_trajectory && 
       trajectoryParameter && 
       trajectoryParameter->show() && 
       trajectoryParameter->pyramidScale() > 0.0f) {
      for(size_t i = 0; i < _trajectory->size(); i += trajectoryParameter->step()) {
	if (i>0) {
	  Eigen::Vector3f p1=_trajectory->at(i).translation();
	  Eigen::Vector3f p2=_trajectory->at(i-1).translation();

	  glBegin(GL_LINES);
	  glVertex3f(p1.x(), p1.y(), p1.z());
	  glVertex3f(p2.x(), p2.y(), p2.z());
	  glEnd();
	}
	glPushMatrix();
	glMultMatrixf(_trajectory->at(i).data());
	glRotatef(90.0f, 0.0f, 1.0f, 0.0f);        
	glScalef(trajectoryParameter->pyramidScale() ,trajectoryParameter->pyramidScale(), trajectoryParameter->pyramidScale() * 2.0f);
	glColor4f(_trajectoryColors->at(i).x(),_trajectoryColors->at(i).y(), _trajectoryColors->at(i).z(), _trajectoryColors->at(i)[3]);
	glCallList(_pyramidDrawList);
	glPopMatrix();
      }
    }
    glEndList();
  }
  
}
