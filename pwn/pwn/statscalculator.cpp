#include "statscalculator.h"

namespace pwn {

  void StatsCalculator::compute(NormalVector &normals,
				StatsVector &statsVector,
				const PointVector &points,
				const IntImage &indexImage) {
    assert(indexImage.rows > 0 && indexImage.cols > 0 && "StatsCalculator: indexImage has zero size");
    assert(points.size() > 0 && "StatsCalculator: points has zero size");
    
    // HAKKE in order to avoid an annoying warning
    if(indexImage.rows > 0 && indexImage.cols > 0) {
      assert(1);
    } 
    
    if(statsVector.size() != points.size())
      statsVector.resize(points.size());
    if(normals.size() != points.size())
      normals.resize(points.size());
    Normal dummyNormal = Normal::Zero();
    std::fill(statsVector.begin(), statsVector.end(), Stats());
    std::fill(normals.begin(), normals.end(), dummyNormal);
  }

}
