#include "image_view.h"

namespace pcl {
  namespace gpu {
    namespace kinfuLS {
      void paint3DView(const KinfuTracker::View& rgb24, 
 		       KinfuTracker::View& view, float colors_weight = 0.5f);
      void mergePointNormal(const pcl::gpu::DeviceArray<PointXYZ>& cloud, 
			    const pcl::gpu::DeviceArray<Normal>& normals, 
			    pcl::gpu::DeviceArray<PointNormal>& output);
    }
  }
}

namespace pwn_fr1_eth_evaluation {

  ImageView::ImageView(): _paint_image(false), _accumulate_views(false) {
    _viewerScene.setWindowTitle("View3D from ray tracing");
    _viewerScene.setPosition(0, 0);
    _viewerDepth.setWindowTitle("Kinect Depth stream");
    _viewerDepth.setPosition(640, 0);
  }

  void ImageView::showScene(pcl::gpu::kinfuLS::KinfuTracker* kinfu, 
			    const pcl::gpu::PtrStepSz<const pcl::gpu::kinfuLS::PixelRGB>& rgb24, 
			    bool registration, Eigen::Affine3f* pose_ptr) {
    if(pose_ptr) {
      _raycaster_ptr->run(kinfu->volume(), *pose_ptr, 
			  kinfu->getCyclicalBufferStructure());
      _raycaster_ptr->generateSceneView(_view_device);
    }
    else { kinfu->getImage(_view_device); }

    if(_paint_image && registration && !pose_ptr) {
      _colors_device.upload(rgb24.data, rgb24.step, rgb24.rows, rgb24.cols);
      paint3DView(_colors_device, _view_device);
    }

    int cols;
    _view_device.download(_view_host, cols);
    _viewerScene.showRGBImage(reinterpret_cast<unsigned char*>(&_view_host[0]), 
			      _view_device.cols (), _view_device.rows());    

    if(_accumulate_views) {
      _views.push_back(cv::Mat());
      cv::cvtColor(cv::Mat(240, 320, CV_8UC3, (void*)&_view_host[0]), _views.back(), CV_RGB2GRAY);
    }
  }

  void ImageView::showGeneratedDepth(pcl::gpu::kinfuLS::KinfuTracker* kinfu, const Eigen::Affine3f& pose) {
    _raycaster_ptr->run(kinfu->volume(), pose, kinfu->getCyclicalBufferStructure());
    _raycaster_ptr->generateDepthImage(_generated_depth);        
    int c;
    std::vector<unsigned short> data;
    _generated_depth.download(data, c);    
    _viewerDepth.showShortImage(&data[0], _generated_depth.cols(), _generated_depth.rows(), 0, 5000, true);
  }

}
