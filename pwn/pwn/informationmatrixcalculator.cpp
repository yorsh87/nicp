#include "informationmatrixcalculator.h"

#include <omp.h>

using namespace Eigen;

namespace pwn {

  void PointInformationMatrixCalculator::compute(InformationMatrixVector &informationMatrix,
						 const StatsVector &statsVector,
						 const NormalVector &imageNormals) {
    assert(statsVector.size() > 0 && "PointInformationMatrixCalculator: statsVector has zero size");
    assert(imageNormals.size() > 0 && "PointInformationMatrixCalculator: imageNormals has zero size");

    informationMatrix.resize(statsVector.size());
    
#pragma omp parallel for
    for(size_t i = 0; i < statsVector.size(); i++) {
      const Stats &stats = statsVector[i];
      InformationMatrix U = Matrix4f::Zero();
      U.block<3, 3>(0, 0) = stats.eigenVectors(); 
      if(imageNormals[i].squaredNorm() > 0) {
	if(stats.curvature() < _curvatureThreshold)
	  informationMatrix[i] = U * _flatInformationMatrix * U.transpose();
	else {
	  informationMatrix[i] = U * _flatInformationMatrix * U.transpose();
	}
      } 
      else 
	informationMatrix[i] = InformationMatrix();
    }
  }

  void NormalInformationMatrixCalculator::compute(InformationMatrixVector &informationMatrix,
						  const StatsVector &statsVector,
						  const NormalVector &imageNormals) {
    assert(statsVector.size() > 0 && "PointInformationMatrixCalculator: statsVector has zero size");
    assert(imageNormals.size() > 0 && "PointInformationMatrixCalculator: imageNormals has zero size");

    informationMatrix.resize(statsVector.size());

#pragma omp parallel for
    for(size_t i = 0; i < statsVector.size(); i++) {
      const Stats &stats = statsVector[i];
      InformationMatrix U = Matrix4f::Zero();
      U.block<3, 3>(0, 0) = stats.eigenVectors(); 
      if(imageNormals[i].squaredNorm()>0) {
	if(stats.curvature() < _curvatureThreshold)
	  informationMatrix[i] = U * _flatInformationMatrix * U.transpose();
	else {	  
	  informationMatrix[i] = U * _flatInformationMatrix * U.transpose();
	}
      } 
      else 
	informationMatrix[i] = InformationMatrix();
    }
  }

}
