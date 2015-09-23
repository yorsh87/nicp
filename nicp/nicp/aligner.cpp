#include "aligner.h"

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
    _lambda = 1e3;
  };

  Aligner::Aligner(PointProjector* projector_,
		   Linearizer* linearizer_,
		   CorrespondenceFinder* correspondenceFinder_) {
    _projector = projector_;
    _linearizer = linearizer_;
    _correspondenceFinder = correspondenceFinder_;
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
    _lambda = 1e3;
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

    // Add the priors
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
