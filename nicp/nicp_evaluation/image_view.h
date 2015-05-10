#pragma once 

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/raycaster.h>
#include <pcl/visualization/image_viewer.h>

namespace nicp_fr1_eth_evaluation {

  class ImageView {
  public:
    ImageView();
    
    void showScene(pcl::gpu::kinfuLS::KinfuTracker* kinfu, 
		   const pcl::gpu::PtrStepSz<const pcl::gpu::kinfuLS::PixelRGB>& rgb24, 
		   bool registration, Eigen::Affine3f* pose_ptr = 0);
    void showDepth(const pcl::gpu::PtrStepSz<const unsigned short>& depth) { _viewerDepth.showShortImage (depth.data, depth.cols, depth.rows, 0, 5000, true); }

    void showGeneratedDepth(pcl::gpu::kinfuLS::KinfuTracker* kinfu, const Eigen::Affine3f& pose);
    void toggleImagePaint() { _paint_image = !_paint_image; }

    bool _paint_image;
    bool _accumulate_views;
    pcl::visualization::ImageViewer _viewerScene; 
    pcl::visualization::ImageViewer _viewerDepth;
    pcl::gpu::kinfuLS::KinfuTracker::View _view_device;
    pcl::gpu::kinfuLS::KinfuTracker::View _colors_device;
    std::vector<pcl::gpu::kinfuLS::PixelRGB> _view_host;
    pcl::gpu::kinfuLS::RayCaster::Ptr _raycaster_ptr;
    pcl::gpu::kinfuLS::KinfuTracker::DepthMap _generated_depth;
    std::vector<cv::Mat> _views;
  };

}
