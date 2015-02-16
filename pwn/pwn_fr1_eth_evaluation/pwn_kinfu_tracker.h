#pragma once

#include <pcl/gpu/kinfu_large_scale/kinfu.h>

namespace pwn_fr1_eth_evaluation {

  class PWNKinfuTracker: public pcl::gpu::kinfuLS::KinfuTracker {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PWNKinfuTracker(const Eigen::Vector3f &volume_size, const float shifting_distance,
		    int rows = 480, int cols = 640): 
    pcl::gpu::kinfuLS::KinfuTracker(volume_size, shifting_distance, rows, cols) {}
    
    virtual bool processFrame(const KinfuTracker::DepthMap& depth_device) { return (*this)(depth_device); }
  };

}
