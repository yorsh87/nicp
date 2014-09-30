#include "gaussian3.h"
#include "pinholepointprojector.h"

#include <fstream>
#include <sstream>

using namespace Eigen;
using namespace std;

namespace pwn {

  Gaussian3fVector::Gaussian3fVector(size_t s, const Gaussian3f &p) { 
    resize(s);
    std::fill(begin(), end(), p);
  }

  void Gaussian3fVector::fromDepthImage(const DepthImage &depthImage, 
					const PinholePointProjector &pointProjector, 
					float dmax, float baseline, float alpha) {
    assert(depthImage.rows > 0 && depthImage.cols > 0 && "Gaussian3fVector: Depth image has zero dimensions");     
    clear();
    resize(depthImage.rows * depthImage.cols, Gaussian3f());
    const float *f = (float*)depthImage.data;
    int k = 0;
    float fB = (baseline * pointProjector.cameraMatrix()(0, 0));
    Matrix3f inverseCameraMatrix = pointProjector.cameraMatrix().inverse();
    Matrix3f J;
    for(int c = 0; c < depthImage.cols; c++) {
      for(int r = 0; r < depthImage.rows; r++, f++) {
	if(*f >= dmax || *f <= 0)
	  continue;
	Point mean;
	pointProjector.unProject(mean, r, c, *f);
	float z = mean.z();
	float zVariation = (alpha * z * z) / (fB + z * alpha);
	J <<       
	  z, 0, (float)r,
	  0, z, (float)c,
	  0, 0, 1;
	J = inverseCameraMatrix * J;
	Diagonal3f imageCovariance(1.0f, 1.0f, zVariation);
	Matrix3f cov = J * imageCovariance * J.transpose();
	at(k) = Gaussian3f(mean.head<3>(), cov);
	k++;
      }
    }

    resize(k);
  }

  void Gaussian3fVector::toPointAndNormalVector(PointVector &destPoints, NormalVector &destNormals, bool eraseNormals) const {
    destPoints.resize(size());
    destNormals.resize(size());
    for(size_t k = 0; k < size(); k++) {
      Point &point = destPoints[k];
      Normal &normal = destNormals[k];    
      point.head<3>() = at(k).mean();
      if(eraseNormals)
	normal.setZero();
    }
  }

}
