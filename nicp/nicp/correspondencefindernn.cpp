#include "correspondencefindernn.h"

namespace nicp {

   CorrespondenceFinderNN::CorrespondenceFinderNN(): CorrespondenceFinder() {
      _index = 0;
      _knn = 1;
      _normal_scaling = 1.0f;
   }

   CorrespondenceFinderNN::~CorrespondenceFinderNN() { if(_index) { delete _index; } }

   void CorrespondenceFinderNN::_model2linear(std::vector<float>& dest, const Cloud& src, float nscale){
      dest.resize(src.points().size() * 6);
      float* dp = &dest[0];
      for(size_t i=0; i < src.points().size(); ++i) {
	 const Point& p = src.points()[i];
	 const Normal& n = nscale * src.normals()[i];
	 *dp = p.x(); ++dp;
	 *dp = p.y(); ++dp;
	 *dp = p.z(); ++dp;
	 *dp = n.x(); ++dp;
	 *dp = n.y(); ++dp;
	 *dp = n.z(); ++dp;
      }
   }

   void CorrespondenceFinderNN::_model2linear(std::vector<float>& dest, const Cloud& src, float nscale, const Eigen::Isometry3f& T) {
      dest.resize(src.points().size() * 6);
      float* dp = &dest[0];
      Eigen::Matrix3f sR = T.linear() * nscale;
      for(size_t i = 0; i < src.points().size(); ++i) {
	 Point p = T * src.points()[i];
	 Eigen::Vector3f n = sR * src.normals()[i].head<3>();
	 *dp=p.x(); ++dp;
	 *dp=p.y(); ++dp;
	 *dp=p.z(); ++dp;
	 *dp=n.x(); ++dp;
	 *dp=n.y(); ++dp;
	 *dp=n.z(); ++dp;
      }
   }

   void CorrespondenceFinderNN::init(const Cloud &referenceScene, const Cloud &currentScene) {
      _model2linear(_current_points, currentScene, _normal_scaling);
      if(_index) { delete(_index); }
      _index = 0;

      _current_matrix = flann::Matrix<float> (&_current_points[0],
					      currentScene.points().size(),
					      6);
      _index = new flann::Index< flann::L2<float> >(_current_matrix, flann::KDTreeIndexParams());
      _index->buildIndex();
      _model2linear(_reference_points, referenceScene, _normal_scaling);
      _reference_matrix = flann::Matrix<float> (&_reference_points[0],
						referenceScene.points().size(),
						6);
      _reference_indices.resize(referenceScene.points().size() * _knn);
      _indices_matrix = flann::Matrix<int>(&_reference_indices[0],
					   referenceScene.points().size(),
					   _knn);
      _reference_distances.resize(referenceScene.points().size() * _knn);
      _distances_matrix = flann::Matrix<float>(&_reference_distances[0],
					       referenceScene.points().size(),
					       _knn);
      _correspondences.reserve(_indices_matrix.rows * _indices_matrix.cols);
   }

   void CorrespondenceFinderNN::compute(const Cloud &referenceScene, const Cloud &currentScene, Eigen::Isometry3f T) {
      _model2linear(_reference_points, referenceScene, _normal_scaling, T);

      flann::SearchParams params(16);
      params.cores = 8;
      _index->radiusSearch(_reference_matrix, _indices_matrix, _distances_matrix,
			   _squaredThreshold, flann::SearchParams(16));

      _correspondences.clear();
      _numCorrespondences = 0;
      for(size_t ridx = 0; ridx < _indices_matrix.rows; ++ridx) {
	int* currentIndexPtr = _indices_matrix.ptr() + (ridx * _knn);
        Eigen::Vector3f rn = T.linear() * referenceScene.normals()[ridx].head<3>();
        Eigen::Vector3f rp = T * referenceScene.points()[ridx].head<3>();
	for(size_t j = 0; j < _indices_matrix.cols; ++j) {
	   int cidx = *currentIndexPtr;
	   currentIndexPtr++;
	   if(cidx < 0) { continue; }
	   const Eigen::Vector3f& cn = currentScene.normals()[cidx].head<3>();
	   const Eigen::Vector3f& cp = currentScene.points()[cidx].head<3>();
	   if(!_demotedToGICP && (rn.squaredNorm() == 0.0f || cn.squaredNorm() == 0.0f)) { continue; }
	   if((rp - cp).squaredNorm() > _squaredThreshold) { continue; }
	   if(isNan(cn) || isNan(rn)) { continue; }
	   if(!_demotedToGICP && rn.dot(cn) < _inlierNormalAngularThreshold) { continue; }
	   _correspondences.push_back(Correspondence(ridx, cidx));
	   _numCorrespondences++;
	}
      }
   }

}
