#include "alignernn.h"

#include <sys/time.h>
#include <omp.h>

#include <opencv2/highgui/highgui.hpp>
#include "nicp/imageutils.h"

#include "unscented.h"
#include "bm_se3.h"

#include <cstdio>
#include "sphericalpointprojector.h"
#include "pinholepointprojector.h"
#include <opencv2/highgui/highgui.hpp>

using namespace std;

namespace nicp {

  void AlignerNN::align() {

    assert(_projector && "AlignerProjective: missing _projector");
    assert(_linearizer && "AlignerProjective: missing _linearizer");
    assert(_correspondenceFinder && "AlignerProjective: missing _correspondenceFinder");
    assert(_referenceCloud && "AlignerProjective: missing _referenceCloud");
    assert(_currentCloud && "AlignerProjective: missing _currentCloud");

    CorrespondenceFinderNN* cfnn = dynamic_cast<CorrespondenceFinderNN*>(_correspondenceFinder);
    assert(cfnn && "AlignerProjective: _correspondenceFinder is not of type CorrespondnceFinderNN");

    struct timeval tvStart, tvEnd;
    gettimeofday(&tvStart, 0);

    cfnn->init(*_referenceCloud, *_currentCloud);
    _T = _initialGuess;
    for(int i = 0; i < _outerIterations; i++) {
      /************************************************************************
       *                         Correspondence Computation                   *
       ************************************************************************/
      // Compute the indices of the current scene from the point of view of the sensor
      _T.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

      // Correspondences computation.
      cfnn->compute(*_referenceCloud, *_currentCloud, _T.inverse());

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
	H += Matrix6f::Identity() * _lambda;

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
	cout << endl;
	cout << "************** WARNING SOLUTION MIGHT BE INVALID (eigenratio failure) **************" << endl;
	cout << "tr: " << _translationalEigenRatio << " rr: " << _rotationalEigenRatio << endl;
	cout << "************************************************************************************" << endl;
      }
    }
    else {
      _solutionValid = true;
      if (_debug) {
	cout << "************** I FOUND SOLUTION VALID SOLUTION   (eigenratio ok) *******************" << endl;
	cout << "tr: " << _translationalEigenRatio << " rr: " << _rotationalEigenRatio << endl;
	cout << "************************************************************************************" << endl;
      }
    }
    if (_debug) {
      cout << "Solution statistics in (t, mq): " << endl;
      cout << "mean: " << _mean.transpose() << endl;
      cout << "Omega: " << endl;
      cout << _omega << endl;
    }
  }

}
