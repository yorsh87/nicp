#include "drawable_correspondences.h"
#include "gl_parameter_correspondences.h"

namespace pwn_viewer {

  DrawableCorrespondences::DrawableCorrespondences() : Drawable() {
    _parameter = 0;
    _numCorrespondences = 0;
    _correspondences = 0;
    _referencePoints = 0;
    _currentPoints = 0;
    _referencePointsTransformation = Eigen::Isometry3f::Identity();
    _correspondenceDrawList = glGenLists(1);
    updateCorrespondenceDrawList();
  }

  DrawableCorrespondences::DrawableCorrespondences(Eigen::Isometry3f transformation_, GLParameter *parameter_,  int numCorrespondences_, PointVector *referencePoints_, 
						   PointVector *currentPoints_, CorrespondenceVector *correspondences_) : Drawable(transformation_) {
    setParameter(parameter_);
    _numCorrespondences = numCorrespondences_;
    _correspondences = correspondences_;
    _referencePoints = referencePoints_;
    _currentPoints = currentPoints_;
    _referencePointsTransformation = Eigen::Isometry3f::Identity();
    _correspondenceDrawList = glGenLists(1);
    updateCorrespondenceDrawList();
  }

  bool DrawableCorrespondences::setParameter(GLParameter *parameter_) {
    GLParameterCorrespondences *correspondencesParameter = (GLParameterCorrespondences*)parameter_;
    if (correspondencesParameter == 0) {
      _parameter = 0;
      return false;
    }
    _parameter = correspondencesParameter;
    return true;
  }

  void DrawableCorrespondences::draw() {
    GLParameterCorrespondences *correspondencesParameter = dynamic_cast<GLParameterCorrespondences*>(_parameter);
    if(correspondencesParameter && 
       correspondencesParameter->show() && 
       correspondencesParameter->lineWidth() > 0.0f) {
      glPushMatrix();
      correspondencesParameter->applyGLParameter();
      glCallList(_correspondenceDrawList);
      glPopMatrix();
    }
  }

  void DrawableCorrespondences::updateCorrespondenceDrawList() {
    GLParameterCorrespondences *correspondencesParameter = dynamic_cast<GLParameterCorrespondences*>(_parameter);
    glNewList(_correspondenceDrawList, GL_COMPILE);   
    if(_referencePoints && 
       _currentPoints && 
       _correspondences && 
       correspondencesParameter && 
       correspondencesParameter->show() && 
       correspondencesParameter->lineWidth() > 0.0f) {
    
      glBegin(GL_LINES);
      for(int i = 0; i < _numCorrespondences; i += correspondencesParameter->step()) {
	const Correspondence &correspondence = _correspondences->at(i);
	const Point &referencePoint = _referencePointsTransformation * _referencePoints->at(correspondence.referenceIndex);
	const Point &currentPoint = _transformation * _currentPoints->at(correspondence.currentIndex);
	glVertex3f(referencePoint.x(), referencePoint.y(), referencePoint.z());
	glVertex3f(currentPoint.x(), currentPoint.y(), currentPoint.z());
      }
      glEnd();
    }
    glEndList();
  }

}
