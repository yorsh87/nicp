#include "aligner.h"

#include <sys/time.h>
#include <omp.h>

#include <opencv2/highgui/highgui.hpp>
#include "pwn/imageutils.h"

#include "unscented.h"
#include "bm_se3.h"

#include <cstdio>
#include "sphericalpointprojector.h"
#include "pinholepointprojector.h"
#include <opencv2/highgui/highgui.hpp>

using namespace std;

namespace pwn {

  Aligner::Aligner() {
    _projector = 0;
    _linearizer = 0;
    _correspondenceFinder = 0;
    _referenceCloud = 0;
    _currentCloud = 0;
    _outerIterations = 10;
    _innerIterations = 1;
    _T = Eigen::Isometry3f::Identity();
    _initialGuess = Eigen::Isometry3f::Identity();
    _referenceSensorOffset = Eigen::Isometry3f::Identity();
    _currentSensorOffset = Eigen::Isometry3f::Identity();
    _totalTime = 0.0f;
    _error = 0.0f;
    _inliers = 0;
    _minInliers = 100;
    _rotationalMinEigenRatio = 50;
    _translationalMinEigenRatio = 50;
    _debug = false;
  };

  void Aligner::addRelativePrior(const Eigen::Isometry3f &mean, const Matrix6f &informationMatrix) {
    _priors.push_back(new SE3RelativePrior(mean, informationMatrix));
  }
  
  void Aligner::addAbsolutePrior(const Eigen::Isometry3f &referenceTransform, const Eigen::Isometry3f &mean, const Matrix6f &informationMatrix) {
    _priors.push_back(new SE3AbsolutePrior(referenceTransform, mean, informationMatrix));
  }

  void Aligner::clearPriors() {
    for(size_t i = 0; i < _priors.size(); i++) {
      delete _priors[i];
    }
    _priors.clear();
  }

  void Aligner::align() {
    
    assert(_projector && "Aligner: missing _projector");
    assert(_linearizer && "Aligner: missing _linearizer");
    assert(_correspondenceFinder && "Aligner: missing _correspondenceFinder");
    assert(_referenceCloud && "Aligner: missing _referenceCloud");
    assert(_currentCloud && "Aligner: missing _currentCloud");

    struct timeval tvStart, tvEnd;
    gettimeofday(&tvStart, 0);

    // The current points are seen from the frame of the sensor
    _projector->setTransform(_currentSensorOffset);
    _projector->project(_correspondenceFinder->currentIndexImage(),
			_correspondenceFinder->currentDepthImage(),
			_currentCloud->points());

    if (_currentCloud->rgbs().size())
      _currentCloud->projectRGB(_correspondenceFinder->currentRGBImage(), 
				_correspondenceFinder->currentIndexImage());
    _T = _initialGuess;
        
    for(int i = 0; i < _outerIterations; i++) {
      /************************************************************************
       *                         Correspondence Computation                   *
       ************************************************************************/
      // Compute the indices of the current scene from the point of view of the sensor
      _T.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      _projector->setTransform(_T * _referenceSensorOffset);
      _projector->project(_correspondenceFinder->referenceIndexImage(),
			  _correspondenceFinder->referenceDepthImage(),
			  _referenceCloud->points());

      if (_referenceCloud->rgbs().size())
	_referenceCloud->projectRGB(_correspondenceFinder->referenceRGBImage(), 
				    _correspondenceFinder->referenceIndexImage());
    
      // Correspondences computation.  
      _correspondenceFinder->compute(*_referenceCloud, *_currentCloud, _T.inverse());
    
// std::cerr << "NumCorrespondence: " << _correspondenceFinder->correspondences().size() << std::endl;
      /************************************************************************
       *                            Alignment                                 *
       ************************************************************************/
      Eigen::Isometry3f invT = _T.inverse();
      for(int k = 0; k < _innerIterations; k++) {      
	invT.matrix().block<1, 4>(3, 0) << 0.0f, 0.0f, 0.0f, 1.0f;
	Matrix6f H;
	Vector6f b;

	_linearizer->setT(invT);
	_linearizer->update();
	H = _linearizer->H();
	b = _linearizer->b();
	H += Matrix6f::Identity() * 1000.0f;

	// Add the priors
	for(size_t j = 0; j < _priors.size(); j++) {
	  const SE3Prior *prior = _priors[j];
	  Vector6f priorError = prior->error(invT);
	  Matrix6f priorJacobian = prior->jacobian(invT);
	  Matrix6f priorInformationRemapped = prior->errorInformation(invT);

	  Matrix6f Hp = priorJacobian.transpose() * priorInformationRemapped * priorJacobian;
	  Vector6f bp = priorJacobian.transpose() * priorInformationRemapped * priorError;

	  H += Hp;
	  b += bp;
	}
      
	Vector6f dx = H.ldlt().solve(-b);
	Eigen::Isometry3f dT = v2t(dx);
	invT = dT * invT;
      }
      
      _T = invT.inverse();
      _T = v2t(t2v(_T));
      _T.matrix().block<1, 4>(3, 0) << 0.0f, 0.0f, 0.0f, 1.0f;
    }

    gettimeofday(&tvEnd, 0);
    double tStart = tvStart.tv_sec * 1000.0f + tvStart.tv_usec * 0.001f;
    double tEnd = tvEnd.tv_sec * 1000.0f + tvEnd.tv_usec * 0.001f;
    _totalTime = tEnd - tStart;
    _error = _linearizer->error();
    _inliers = _linearizer->inliers();

    _computeStatistics(_mean, _omega, _translationalEigenRatio, _rotationalEigenRatio);
    if (_rotationalEigenRatio > _rotationalMinEigenRatio || 
	_translationalEigenRatio > _translationalMinEigenRatio) {
      _solutionValid = false;
      if (_debug) {
	cerr << endl;
	cerr << "************** WARNING SOLUTION MIGHT BE INVALID (eigenratio failure) **************" << endl;
	cerr << "tr: " << _translationalEigenRatio << " rr: " << _rotationalEigenRatio << endl;
	cerr << "************************************************************************************" << endl;
      }
    } 
    else {
      _solutionValid = true;
      if (_debug) {
	cerr << "************** I FOUND SOLUTION VALID SOLUTION   (eigenratio ok) *******************" << endl;
	cerr << "tr: " << _translationalEigenRatio << " rr: " << _rotationalEigenRatio << endl;
	cerr << "************************************************************************************" << endl;
      }
    }
    if (_debug) {
      cout << "Solution statistics in (t, mq): " << endl;
      cout << "mean: " << _mean.transpose() << endl;
      cout << "Omega: " << endl;
      cout << _omega << endl;
    }
  }

  void Aligner::_computeStatistics(Vector6f &mean, Matrix6f &Omega, 
				   float &translationalRatio, float &rotationalRatio, bool usePriors) const {
    typedef SigmaPoint<Vector6f> SigmaPoint;
    typedef std::vector<SigmaPoint, Eigen::aligned_allocator<SigmaPoint> > SigmaPointVector;
  
    // Output init
    Matrix6f H;
    H.setZero();
    translationalRatio = std::numeric_limits<float>::max();
    rotationalRatio = std::numeric_limits<float>::max();

    Eigen::Isometry3f invT = _T.inverse();
    invT.matrix().block<1, 4>(3, 0) << 0.0f, 0.0f, 0.0f, 1.0f;
    _linearizer->setT(invT);
    _linearizer->update();
    H += _linearizer->H();

    // // Add the priors
    for(size_t j = 0; usePriors && j < _priors.size(); j++) {
      const SE3Prior *prior = _priors[j];
      Matrix6f priorJacobian = prior->jacobian(invT);
      Matrix6f priorInformationRemapped = prior->errorInformation(invT);
      Matrix6f Hp = priorJacobian.transpose() * priorInformationRemapped * priorJacobian;
      H += Hp;
    }

    JacobiSVD<Matrix6f> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Matrix6f localSigma = svd.solve(Matrix6f::Identity());
    SigmaPointVector sigmaPoints;
    Vector6f localMean = Vector6f::Zero();
    sampleUnscented(sigmaPoints, localMean, localSigma);
  
    Eigen::Isometry3f dT = _T;  // Transform from current to reference
    
    // Remap each of the sigma points to their original position
    //#pragma omp parallel 
    for (size_t i = 0; i < sigmaPoints.size(); i++) {
      SigmaPoint &p = sigmaPoints[i];
      p._sample = t2v(dT * v2t(p._sample).inverse());
    }
    // Reconstruct the gaussian 
    reconstructGaussian(mean, localSigma, sigmaPoints);

    // Compute the information matrix from the covariance
    Omega = localSigma.inverse();
    Omega = .5* (Omega + Omega.transpose());
    // Have a look at the svd of the rotational and the translational part;
    JacobiSVD<Matrix3f> partialSVD;
    partialSVD.compute(Omega.block<3, 3>(0, 0));
    translationalRatio = partialSVD.singularValues()(0) / partialSVD.singularValues()(2);
    
    partialSVD.compute(Omega.block<3, 3>(3, 3));
    rotationalRatio = partialSVD.singularValues()(0) / partialSVD.singularValues()(2);
  }

}
