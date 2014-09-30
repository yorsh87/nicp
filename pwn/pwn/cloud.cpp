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

  void Cloud::resize(size_t s) {
    if (s) {
      _points.resize(s);
      _normals.resize(s);
      _stats.resize(s);
      _normalInformationMatrix.resize(s);
      _pointInformationMatrix.resize(s);
      _gaussians.resize(s);
      _rgbs.resize(s);
    } else
      clear();
  }
  
  size_t Cloud::size() {
    return _points.size();
  }

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

  void Cloud::add(Cloud cloud, const Eigen::Isometry3f &T) {
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
    // if (_traversabilityVector.size() && cloud.traversabilityVector().size())
    //   _traversabilityVector.resize(k + cloud.traversabilityVector().size());
    for(int i = 0; k < _points.size(); k++, i++) {
      _points[k] = cloud.points()[i];
      _normals[k] = cloud.normals()[i];
      _stats[k] = cloud.stats()[i];
      if(cloud.pointInformationMatrix().size() != 0) {
	_pointInformationMatrix[k] = cloud.pointInformationMatrix()[i];
	_normalInformationMatrix[k] = cloud.normalInformationMatrix()[i];
      }
      if(cloud.gaussians().size() != 0) {
	_gaussians[k] = cloud.gaussians()[i];
      }
      if(cloud.rgbs().size() != 0) {
      	_rgbs[k] = cloud.rgbs()[i];
      }
      // if(cloud.traversabilityVector().size() != 0) {
      // 	_traversabilityVector[k] = cloud.traversabilityVector()[i];
      // }
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


}
