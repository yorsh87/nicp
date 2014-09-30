#pragma once

#include "pwn/correspondencefinder.h"
#include "drawable.h"
#include "gl_parameter_correspondences.h"

using namespace pwn;

namespace pwn_viewer {

  class DrawableCorrespondences : public Drawable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    DrawableCorrespondences();
    DrawableCorrespondences(Eigen::Isometry3f transformation_, GLParameter *parameter_,  int numCorrespondences_, PointVector *referencePoints_, 
			    PointVector *currentPoints_, CorrespondenceVector *correspondences_);
    virtual ~DrawableCorrespondences() { glDeleteLists(_correspondenceDrawList, 1); }
  
    virtual GLParameter* parameter() { return _parameter; };
    virtual bool setParameter(GLParameter *parameter_);
    
    Eigen::Isometry3f referencePointsTransformation() { return _referencePointsTransformation; }
    void setReferencePointsTransformation(Eigen::Isometry3f referencePointsTransformation_) { 
      _referencePointsTransformation = referencePointsTransformation_; 
      updateCorrespondenceDrawList();
    }
    
    int numCorrespondances() { return _numCorrespondences; }
    void setNumCorrespondences(int numCorrespondences_) { 
      _numCorrespondences = numCorrespondences_; 
      updateCorrespondenceDrawList();
    }
    
    CorrespondenceVector* correspondences() { return _correspondences; }
    void setCorrespondences(CorrespondenceVector *correspondences_) { 
      _correspondences = correspondences_;
      updateCorrespondenceDrawList();
    }
    
    PointVector* referencePoints() { return _referencePoints; }
    void setReferencePoints(PointVector *referencePoints_) { 
      _referencePoints = referencePoints_; 
      updateCorrespondenceDrawList();
    }
    
    PointVector* currentPoints() { return _currentPoints; }
    void setCurrentPoints(PointVector *currentPoints_) { 
      _currentPoints = currentPoints_; 
      updateCorrespondenceDrawList();
    }
    
    inline GLuint correspondenceDrawList() { return _correspondenceDrawList; }

    void setStep(int step_) {
      _parameter->setStep(step_);
      updateCorrespondenceDrawList();
    }
    void setLineWidth(float lineWidth_) {
      _parameter->setLineWidth(lineWidth_);
      updateCorrespondenceDrawList();
    }

    virtual void draw();
    void updateCorrespondenceDrawList();
 
  protected:
    Eigen::Isometry3f _referencePointsTransformation;
    GLParameterCorrespondences *_parameter;  
    int _numCorrespondences;
    CorrespondenceVector *_correspondences;
    PointVector *_referencePoints;
    PointVector *_currentPoints;
    GLuint _correspondenceDrawList; 
  };  

}
