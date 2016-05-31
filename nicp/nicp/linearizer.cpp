#include "aligner.h"

#include <omp.h>

using namespace std;

namespace nicp {

  Linearizer::Linearizer() {
    _aligner = 0;
    _H.setZero();
    _b.setZero();
    _inlierMaxChi2 = 9e3;
    _robustKernel = true;
    _demotedToGeneralizedICP = false;
    _zScaling = true;
    _scale = 1.0f;
  }

  void Linearizer::update() {
    assert(_aligner && "Aligner: missing _aligner");

    // Variables initialization.
    _b = Vector6f::Zero();
    _H = Matrix6f::Zero();
    const InformationMatrixVector &pointOmegas = _aligner->currentCloud()->pointInformationMatrix();
    const InformationMatrixVector &normalOmegas = _aligner->currentCloud()->normalInformationMatrix();

    // Allocate the variables for the sum reduction;
    int numThreads = omp_get_max_threads();
    Matrix4f _Htt[numThreads], _Htr[numThreads], _Hrr[numThreads];
    Vector4f _bt[numThreads], _br[numThreads];
    int _inliers[numThreads];
    float _errors[numThreads];
    int iterationsPerThread = _aligner->correspondenceFinder()->numCorrespondences() / numThreads;
#pragma omp parallel
    {
      int threadId = omp_get_thread_num();
      int imin = iterationsPerThread * threadId;
      int imax = imin + iterationsPerThread;
      if(imax > _aligner->correspondenceFinder()->numCorrespondences())
	imax = _aligner->correspondenceFinder()->numCorrespondences();

      Eigen::Matrix4f Htt;
      Eigen::Matrix4f Htr;
      Eigen::Matrix4f Hrr;
      Eigen::Vector4f bt;
      Eigen::Vector4f br;
      int inliers;
      float error;
      Htt = Matrix4f::Zero();
      Htr = Matrix4f::Zero();
      Hrr = Matrix4f::Zero();
      bt = Vector4f::Zero();
      br = Vector4f::Zero();
      error = 0;
      inliers = 0;
      for(int i = imin; i < imax; i++) {
	const Correspondence &correspondence = _aligner->correspondenceFinder()->correspondences()[i];
	const Point referencePoint = _T * _aligner->referenceCloud()->points()[correspondence.referenceIndex];
	const Normal referenceNormal = _T * _aligner->referenceCloud()->normals()[correspondence.referenceIndex];
	const Point &currentPoint = _aligner->currentCloud()->points()[correspondence.currentIndex];
	const Normal &currentNormal = _aligner->currentCloud()->normals()[correspondence.currentIndex];
	InformationMatrix omegaP = pointOmegas[correspondence.currentIndex];
	InformationMatrix omegaN = _scale * normalOmegas[correspondence.currentIndex];
	if(_zScaling) {
	  omegaP *= 1.0f / fabs(currentPoint.z());
	  omegaN *= 1.0f / fabs(currentPoint.z());
	}

	if(_demotedToGeneralizedICP) { omegaN.setZero(); }

	const Vector4f pointError = referencePoint - currentPoint;
	const Vector4f normalError = referenceNormal - currentNormal;
	const Vector4f ep = omegaP * pointError;
	const Vector4f en = omegaN * normalError;

	float localError = pointError.dot(ep) + normalError.dot(en);
	float kscale = 1;
	if(localError > _inlierMaxChi2) {
	  if (_robustKernel) {
	    kscale = _inlierMaxChi2 / localError;
	  }
	  else {
	    continue;
	  }
	}
	inliers++;

	error += kscale * localError;
	Matrix4f Sp = skew(referencePoint);
	Matrix4f Sn = skew(referenceNormal);
	Htt.noalias() += omegaP * kscale;
	Htr.noalias() += omegaP * Sp * kscale;
	Hrr.noalias() += (Sp.transpose() * omegaP * Sp + Sn.transpose() * omegaN * Sn) * kscale;
	bt.noalias() += kscale * ep;
	br.noalias() += kscale * (Sp.transpose() * ep + Sn.transpose() * en);
      }

      _Htt[threadId] = Htt;
      _Htr[threadId] = Htr;
      _Hrr[threadId] = Hrr;
      _bt[threadId] = bt;
      _br[threadId] = br;
      _errors[threadId] = error;
      _inliers[threadId] = inliers;
    }

    // Now do the reduce
    Eigen::Matrix4f Htt = Eigen::Matrix4f::Zero();
    Eigen::Matrix4f Htr = Eigen::Matrix4f::Zero();
    Eigen::Matrix4f Hrr = Eigen::Matrix4f::Zero();
    Eigen::Vector4f bt = Eigen::Vector4f::Zero();
    Eigen::Vector4f br = Eigen::Vector4f::Zero();
    this->_inliers = 0;
    this->_error = 0;
    for(int t = 0; t < numThreads; t++) {
      Htt += _Htt[t];
      Htr += _Htr[t];
      Hrr += _Hrr[t];
      bt += _bt[t];
      br += _br[t];
      this->_inliers += _inliers[t];
      this->_error += _errors[t];
    }
    _H.block<3, 3>(0, 0) = Htt.block<3, 3>(0, 0);
    _H.block<3, 3>(0, 3) = Htr.block<3, 3>(0, 0);
    _H.block<3, 3>(3, 3) = Hrr.block<3, 3>(0, 0);
    _H.block<3, 3>(3, 0) = _H.block<3, 3>(0, 3).transpose();
    _b.block<3, 1>(0, 0) = bt.block<3, 1>(0, 0);
    _b.block<3, 1>(3, 0) = br.block<3, 1>(0, 0);
  }

}
