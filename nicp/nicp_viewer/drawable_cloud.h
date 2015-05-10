#pragma once

#include "nicp_qglviewer.h"
#include "imageview.h"
#include "drawable_points.h"
#include "drawable_normals.h"
#include "drawable_covariances.h"
#include "drawable_correspondences.h"
#include "gl_parameter.h"
#include "gl_parameter_points.h"
#include "gl_parameter_normals.h"
#include "gl_parameter_covariances.h"
#include "gl_parameter_correspondences.h"
#include "gl_parameter_cloud.h"

namespace nicp_viewer {

  class DrawableCloud : public Drawable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    DrawableCloud(const Eigen::Isometry3f &transformation_ = Eigen::Isometry3f::Identity(), 
		  GLParameter *parameter_ = 0, Cloud *cloud_ = 0);
    virtual ~DrawableCloud() { clearDrawableObjects(); }

    const Eigen::Isometry3f& localTransformation() const { return _localTransform; }
    void setLocalTransformation(const Eigen::Isometry3f &localTransform_) { _localTransform = localTransform_; }

    virtual GLParameter* parameter() { return _parameter; } 
    virtual bool setParameter(GLParameter *parameter_);

    Cloud* cloud() const { return _cloud; }
    void setCloud(Cloud *cloud_);

    DrawableCorrespondences* drawableCorrespondences() { return _drawableCorrespondences; }
    void setDrawableCorrespondences(DrawableCorrespondences* drawableCorrespondences_) {
      if(_drawableCorrespondences)
	delete _drawableCorrespondences;
      _drawableCorrespondences = drawableCorrespondences_;
    }

    DrawablePoints* drawablePoints() { return _drawablePoints; }
    DrawableNormals* drawableNormals() { return _drawableNormals; }
    DrawableCovariances* drawableCovariances() { return _drawableCovariances; }
  
    void clearDrawableObjects();
    void constructDrawableObjects();

    void draw();

  protected:
    Eigen::Isometry3f _localTransform;
    Cloud *_cloud;
    DrawableCloud* _previousDrawableCloud;
    GLParameterCloud * _parameter;
    DrawablePoints *_drawablePoints;
    DrawableNormals *_drawableNormals;
    DrawableCovariances *_drawableCovariances;
    DrawableCorrespondences *_drawableCorrespondences;
  };

}
