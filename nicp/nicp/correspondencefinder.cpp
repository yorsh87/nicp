#include "correspondencefinder.h"

#include <omp.h>

using namespace std;

namespace nicp {

  CorrespondenceFinder::CorrespondenceFinder() {
    _inlierDistanceThreshold = 1.0f;
    _squaredThreshold = _inlierDistanceThreshold * _inlierDistanceThreshold;
    _inlierNormalAngularThreshold = cosf(M_PI/6);
    _flatCurvatureThreshold = 0.2f;
    _inlierCurvatureRatioThreshold = 1.3f;
    _numCorrespondences = 0;
    _correspondences.clear();
    _demotedToGICP = false;
  }

}
