#include "cloud.h"
#include <stdexcept>

#include <fstream>

#include "bm_se3.h"

using namespace std;


namespace pwn {

  bool Cloud::load(Eigen::Isometry3f &T, const char *filename) {
    ifstream is(filename);
    if(!is)
      return false;
    return load(T, is);
  }
  
  bool Cloud::save(const char *filename, Eigen::Isometry3f T, int step, bool binary) {
    ofstream os(filename);
    if(!os)
      return false;
    return save(os, T, step, binary);  
  }

  void Cloud::resize(size_t s, bool hasRGB) {
    if (s) {
      _points.resize(s);
      _normals.resize(s);
      _stats.resize(s);
      _normalInformationMatrix.resize(s);
      _pointInformationMatrix.resize(s);
      _gaussians.resize(s);
      if(hasRGB) { _rgbs.resize(s); }
    } else
      clear();
  }
  
  size_t Cloud::size() { return _points.size(); }

  bool Cloud::load(Eigen::Isometry3f &T, istream &is) {
    _points.clear();
    _normals.clear();
    char buf[1024];
    is.getline(buf, 1024);
    istringstream ls(buf);
    string tag;
    size_t numPoints;
    bool binary;
    ls >> tag;
    if(tag != "PWNCLOUD")
      return false;
    ls >> numPoints >> binary;
    _points.resize(numPoints);
    _normals.resize(numPoints);
    _stats.resize(numPoints);
    cerr << "Reading " << numPoints << " points, binary : " << binary << endl;
    is.getline(buf, 1024);
    istringstream lst(buf);
    Vector6f transform;
    lst >> transform[0] >> transform[1] >> transform[2] 
	>> transform[3] >> transform[4] >> transform[5];
    T = v2t(transform);
    size_t k = 0;
    while(k < _points.size() && is.good()) {
      Point& point = _points[k];
      Normal& normal = _normals[k];
      Stats& stats = _stats[k];
      if(!binary) {
	is.getline(buf, 1024);
	istringstream ls(buf);
	string s;
	ls >> s;
	if (s != "POINTWITHSTATS")
	  continue;
	for(int i = 0; i < 3 && ls; i++) {
	  ls >> point[i];
	}
	for(int i = 0; i < 3 && ls; i++) {
	  ls >> normal[i];
	}
	for(int r = 0; r < 4 && ls; r++) {
	  for(int c = 0; c < 4 && ls; c++) {
	    ls >> stats(r, c);
	  }
	}
      } 
      else {
	is.read((char*) &point, sizeof(Point));
	is.read((char*) &normal, sizeof(Normal));
	is.read((char*) &stats, sizeof(Stats));
      }
      k++;
    }
    return is.good();
  }

  bool Cloud::save(ostream &os, Eigen::Isometry3f T, int step, bool binary) {
    os << "PWNCLOUD " << _points.size() / step << " " << binary << endl; 
    Vector6f transform = t2v(T);
    os << transform[0] << " " << transform[1] << " " << transform[2] << " " 
       << transform[3] << " " << transform[4] << " " << transform[5] << " " << endl;
    for(size_t i = 0; i < _points.size(); i += step) {
      const Point &point = _points[i];
      const Normal &normal = _normals[i];
      const Stats &stats = _stats[i];
      if(!binary) {
	os << "POINTWITHSTATS ";
	for (int k = 0; k < 3; k++)
	  os << point[k] << " ";
	for (int k = 0; k < 3; k++) {
	  if(_normals.size() == _points.size())
	    os << normal[k] << " ";
	  else {
	    float zero = 0.0f;
	    os << zero << " ";
	  }
	}
	for (int r = 0; r < 4; r++) {
	  for (int c = 0; c < 4; c++) {
	    if(_stats.size() == _points.size())
	      os << stats(r, c) << " ";
	    else {
	      float zero = 0.0f;
	      os << zero << " ";
	    }
	  }
	}
	os << endl;
      } 
      else {
	os.write((const char*) &point, sizeof(Point));
	if(_normals.size() == _points.size())
	  os.write((const char*) &normal, sizeof(Normal));
	else {
	  const Normal zero = Normal(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
	  os.write((const char*) &zero, sizeof(Normal));	
	}
	if(_stats.size() == _points.size())
	  os.write((const char*) &stats, sizeof(Stats));
	else {
	  Stats zero;
	  zero.setZero();
	  os.write((const char*) &zero, sizeof(Stats));	
	}
      }
    }
    return os.good();
  }

  void Cloud::clear() {
    _points.clear();
    _normals.clear(); 
    _stats.clear();
    _pointInformationMatrix.clear();
    _normalInformationMatrix.clear();
    _gaussians.clear();
    //_traversabilityVector.clear();
    _rgbs.clear();
  }

  void Cloud::add(Cloud& cloud, const Eigen::Isometry3f &T) {
    cloud.transformInPlace(T);
    size_t k = _points.size(); 
    _points.resize(k + cloud.points().size());
    _normals.resize(k + cloud.normals().size());
    _stats.resize(k + cloud.stats().size());
    _pointInformationMatrix.resize(k + cloud.pointInformationMatrix().size());
    _normalInformationMatrix.resize(k + cloud.normalInformationMatrix().size());
    _gaussians.resize(k + cloud.gaussians().size());
    if (_rgbs.size() + cloud.rgbs().size())
      _rgbs.resize(k + cloud.rgbs().size());
    for(int i = 0; k < _points.size(); k++, i++) {
      _points[k] = cloud.points()[i];
      _normals[k] = cloud.normals()[i];
      _stats[k] = cloud.stats()[i];
      // if(cloud.pointInformationMatrix().size() != 0) {
	_pointInformationMatrix[k] = cloud.pointInformationMatrix()[i];
	_normalInformationMatrix[k] = cloud.normalInformationMatrix()[i];
	// }
	// if(cloud.gaussians().size() != 0) {
	_gaussians[k] = cloud.gaussians()[i];
	// }
      if(cloud.rgbs().size() != 0) {
      	_rgbs[k] = cloud.rgbs()[i];
      }
    }
  }

  void Cloud::transformInPlace(const Eigen::Isometry3f& T){
    Eigen::Matrix4f m = T.matrix();
    m.row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    if(m != Eigen::Matrix4f::Identity()) {
      _points.transformInPlace(m);
      _normals.transformInPlace(m);
      _stats.transformInPlace(m);
      _gaussians.transformInPlace(m);
      m.row(3) << 0.0f, 0.0f, 0.0f, 0.0f;
      m.col(3) << 0.0f, 0.0f, 0.0f, 0.0f;
      _pointInformationMatrix.transformInPlace(m);
      _normalInformationMatrix.transformInPlace(m);
    }
  }


  void Cloud::projectRGB(RGBImage& dest, const IndexImage& indices) const {
    if (dest.rows != indices.rows || dest.cols !=indices.cols){
      dest.create(indices.rows, indices.cols);
    }
    if (! _rgbs.size())
      return;
    for (int i=0; i<indices.rows; i++)
      for (int j=0; j<indices.cols; j++){
	int idx = indices(i,j);
	if (idx>=0 && idx<(int)_rgbs.size())
	  dest(i,j)=_rgbs[idx];
	else
	  dest(i,j)=cv::Vec3b(0,0,0);
      }
  }

  void Cloud::unprojectRGB(const RGBImage& rgb, const IndexImage& indices) {
    if (rgb.rows != indices.rows || rgb.cols !=indices.cols){
      cerr << rgb.rows << " != " << indices.rows << endl;
      cerr << rgb.cols << " != " << indices.cols << endl;
      throw::std::runtime_error("irgb and index image sizes dont match");
    }
    if (_points.size()==0) {
      _rgbs.resize(0);
      return;
    }
    _rgbs.resize(_points.size(), cv::Vec3b(0,0,0));
    for (int i=0; i<indices.rows; i++)
      for (int j=0; j<indices.cols; j++){
	int idx = indices(i,j);
	if (idx>=0 && idx<(int)_rgbs.size()) {
	  _rgbs[idx] = rgb(i,j); 
	} else
	  _rgbs[idx] = cv::Vec3b(0,0,0);
      }
  }

  struct IndexTriplet {
    int x, y, z, index;

    IndexTriplet() {
      x = y = z = 0;
      index = -1;
    }

    IndexTriplet(const Eigen::Vector4f& v, int idx, float ires) {
      x = (int)(ires*v.x());
      y = (int)(ires*v.y());
      z = (int)(ires*v.z());
      index = idx;
    }

    bool operator < (const IndexTriplet& o) const {
      if(z < o.z) { return true; }
      if(z > o.z) { return false; }
      if(x < o.x) { return true; }
      if(x > o.x) { return false; }
      if(y < o.y) { return true; }
      if(y > o.y) { return false; }
      if(index < o.index) { return true; }
      return false;
    }

    bool sameCell(const IndexTriplet& o) const {
      return x == o.x && y == o.y && z == o.z;
    }
  };

  struct StatsAccumulator {
    StatsAccumulator() {
      acc = 0;
      u = Eigen::Vector3f::Zero();
      omega = Eigen::Matrix3f::Zero();
    }

    int acc;
    Eigen::Vector3f u;    
    Eigen::Matrix3f omega;
  };

  void voxelize(Cloud* model, float res) {
    float ires = 1.0f / res;
    std::vector<IndexTriplet> voxels(model->size());
    for(int i = 0; i < (int)model->size(); i++) { voxels[i] = IndexTriplet(model->points()[i], i , ires); }
    Cloud sparseModel;
    sparseModel.resize(model->size());
    std::sort(voxels.begin(), voxels.end());
    int k = -1;
    std::vector<StatsAccumulator> statsAccs;
    statsAccs.resize(model->size());
    std::fill(statsAccs.begin(), statsAccs.begin(), StatsAccumulator());
    for(size_t i = 0; i < voxels.size(); i++) { 
      IndexTriplet& triplet = voxels[i];
      int idx = triplet.index;
      StatsAccumulator& statsAcc = statsAccs[i];
      if(k >= 0 && voxels[i].sameCell(voxels[i-1])) { 	
	Eigen::Matrix3f U = model->stats()[idx].eigenVectors();
	Eigen::Vector3f lambdas = model->stats()[idx].eigenValues();
	Eigen::Matrix3f sigma = U * Diagonal3f(lambdas.x(), lambdas.y(), lambdas.z()) * U.inverse(); 
	Eigen::Matrix3f omega = sigma.inverse();
	statsAcc.omega += omega;
	statsAcc.u += omega * model->stats()[idx].mean().head<3>();
	statsAcc.acc++;
      } 
      else {
	k++;
	Eigen::Matrix3f U = model->stats()[idx].eigenVectors();
	Eigen::Vector3f lambdas = model->stats()[idx].eigenValues();
	Eigen::Matrix3f sigma = U * Diagonal3f(lambdas.x(), lambdas.y(), lambdas.z()) * U.inverse(); 
	Eigen::Matrix3f omega = sigma.inverse();
	statsAcc.omega += omega;
	statsAcc.u += omega * model->stats()[idx].mean().head<3>();
	statsAcc.acc++;
	sparseModel.points()[k] = model->points()[idx];
	sparseModel.normals()[k] = model->normals()[idx];
	sparseModel.stats()[k] = model->stats()[idx];
	sparseModel.pointInformationMatrix()[k] = model->pointInformationMatrix()[idx];
	sparseModel.normalInformationMatrix()[k] = model->normalInformationMatrix()[idx];
	sparseModel.gaussians()[k] = model->gaussians()[idx];
      } 
    }
    sparseModel.resize(k); 
    statsAccs.resize(k);
    *model = sparseModel;
    
    InformationMatrix _flatPointInformationMatrix = InformationMatrix::Zero();
    InformationMatrix _nonFlatPointInformationMatrix = InformationMatrix::Zero();
    InformationMatrix _flatNormalInformationMatrix = InformationMatrix::Zero();
    InformationMatrix _nonFlatNormalInformationMatrix = InformationMatrix::Zero();
    _flatPointInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1000.0f, 0.001f, 0.001f));
    _nonFlatPointInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1.0f, 0.1f, 0.1f));
    _flatNormalInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1000.0f, 0.001f, 0.001f));
    _nonFlatNormalInformationMatrix.diagonal() = Normal(Eigen::Vector3f(0.1f, 1.0f, 1.0f));
    for(size_t i = 0; i < model->size(); i++) { 
      StatsAccumulator& statsAcc = statsAccs[i];
      if(statsAcc.acc > 1) {
	statsAcc.u = statsAcc.omega * statsAcc.u;      
	Point& p = model->points()[i];
	p = statsAcc.u;      
            
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver;
	eigenSolver.computeDirect(statsAcc.omega.inverse(), Eigen::ComputeEigenvectors);
	Stats& stats = model->stats()[i]; 
	stats.setZero();
	stats.setEigenVectors(eigenSolver.eigenvectors());
	stats.setMean(statsAcc.u);
	InformationMatrix U = Eigen::Matrix4f::Zero();
	U.block<3, 3>(0, 0)  = eigenSolver.eigenvectors();
	Eigen::Vector3f eigenValues = eigenSolver.eigenvalues();
	if(eigenValues(0) < 0.0f) { eigenValues(0) = 0.0f; }	  
	stats.setEigenValues(eigenValues);
	stats.setN(statsAcc.acc);

	Normal& n = model->normals()[i];
	n = stats.block<4, 1>(0, 0);
	if(stats.curvature() < 0.3f) {
	  if(n.dot(p) > 0) { n = -n; }
	} 
	else { n.setZero(); }      

	if(stats.curvature() < 0.1f) {
	  model->pointInformationMatrix()[i] = U * _flatPointInformationMatrix * U.transpose();
	  model->normalInformationMatrix()[i] = U * _flatNormalInformationMatrix * U.transpose();
	}
	else {
	  model->pointInformationMatrix()[i] = U * _nonFlatPointInformationMatrix * U.transpose();
	  model->normalInformationMatrix()[i] = U * _nonFlatNormalInformationMatrix * U.transpose();
	}
      }
    }
  }
}
