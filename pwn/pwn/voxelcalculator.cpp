#include "voxelcalculator.h"

using namespace std;
using namespace Eigen;

namespace pwn {

  void VoxelCalculator::compute(Cloud &cloud, float res) {
    float oldRes = resolution();
    setResolution(res);
    compute(cloud);
    setResolution(oldRes);
  }

  void VoxelCalculator::compute(Cloud &cloud) {
    AccumulatorMap accumulatorMap;
    float inverseResolution = 1.0f / _resolution;

    for(size_t i = 0; i < cloud.points().size(); i++) {
      const Point &point = cloud.points()[i];

      IndexComparator s;
      s.indeces[0] = (int) (point[0] * inverseResolution);
      s.indeces[1] = (int) (point[1] * inverseResolution);
      s.indeces[2] = (int) (point[2] * inverseResolution);

      AccumulatorMap::iterator it = accumulatorMap.find(s);
      if(it == accumulatorMap.end()) {
	VoxelAccumulator voxelAccumulator;
	voxelAccumulator.accumulator = point;
	voxelAccumulator.numPoints = 1;
	voxelAccumulator.index = i;

	accumulatorMap.insert(make_pair(s, voxelAccumulator));
      }
      else {
	VoxelAccumulator &voxelAccumulator = it->second;
	voxelAccumulator.add(point);
      }
    }

    std::cout << "Voxelization resized the cloud from " << cloud.points().size() << " to ";
    // HAKKE
    // cloud.clear();
    Cloud tmpCloud;
    tmpCloud.clear();
    for(AccumulatorMap::iterator it = accumulatorMap.begin(); it != accumulatorMap.end(); it++) {
      VoxelAccumulator &voxelAccumulator = it->second;
      // HAKKE
      // Point average = voxelAccumulator.average();
      // cloud.points().push_back(average);
      tmpCloud.points().push_back(cloud.points()[voxelAccumulator.index]);
      tmpCloud.normals().push_back(cloud.normals()[voxelAccumulator.index]);
      tmpCloud.stats().push_back(cloud.stats()[voxelAccumulator.index]);
      if(cloud.pointInformationMatrix().size() == cloud.points().size() &&
	 cloud.normalInformationMatrix().size() == cloud.points().size()) {
	tmpCloud.pointInformationMatrix().push_back(cloud.pointInformationMatrix()[voxelAccumulator.index]);
	tmpCloud.normalInformationMatrix().push_back(cloud.normalInformationMatrix()[voxelAccumulator.index]);
      }
      // if(cloud.traversabilityVector().size() == cloud.points().size()) {
      // 	tmpCloud.traversabilityVector().push_back(cloud.traversabilityVector()[voxelAccumulator.index]);
      // }
      if(cloud.gaussians().size() == cloud.points().size()) {
	tmpCloud.gaussians().push_back(cloud.gaussians()[voxelAccumulator.index]);
      }
    }

    // HAKKE
    cloud.clear();
    cloud = tmpCloud;

    std::cout << cloud.points().size() << " points" << std::endl;
  }
}
