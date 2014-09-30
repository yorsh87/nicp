#include "depthimageconverter.h"
#include "pinholepointprojector.h"

#include <iostream>
#include <fstream>

using namespace std;

namespace pwn {

  DepthImageConverter::DepthImageConverter(PointProjector *projector_,
					   StatsCalculator *statsCalculator_,
					   PointInformationMatrixCalculator *pointInformationMatrixCalculator_,
					   NormalInformationMatrixCalculator *normalInformationMatrixCalculator_) {
    _projector = projector_;
    _statsCalculator = statsCalculator_;
    _pointInformationMatrixCalculator = pointInformationMatrixCalculator_;
    _normalInformationMatrixCalculator = normalInformationMatrixCalculator_;
  }

  void DepthImageConverter::compute(Cloud &cloud,
				    const DepthImage &depthImage, 
				    const Eigen::Isometry3f &sensorOffset) {
    assert(_projector && "DepthImageConverter: missing _projector");
    assert(_statsCalculator && "DepthImageConverter: missing _statsCalculator");
    assert(_pointInformationMatrixCalculator && "DepthImageConverter: missing _pointInformationMatrixCalculator");
    assert(_normalInformationMatrixCalculator && "DepthImageConverter: missing _normalInformationMatrixCalculator");
    assert(depthImage.rows > 0 && depthImage.cols > 0 && "DepthImageConverter: depthImage has zero size");

    cloud.clear();
    _projector->setImageSize(depthImage.rows, depthImage.cols);
        
    // Resizing the temporaries
    if(depthImage.rows != _indexImage.rows || depthImage.cols != _indexImage.cols) {
      _indexImage.create(depthImage.rows, depthImage.cols);
    }

    // Unprojecting
    _projector->setTransform(Eigen::Isometry3f::Identity());
    _projector->unProject(cloud.points(), cloud.gaussians(), _indexImage, depthImage);
    
    _statsCalculator->compute(cloud.normals(),
			      cloud.stats(),
			      cloud.points(),
			      _indexImage);

    _pointInformationMatrixCalculator->compute(cloud.pointInformationMatrix(), cloud.stats(), cloud.normals());
    _normalInformationMatrixCalculator->compute(cloud.normalInformationMatrix(), cloud.stats(), cloud.normals());

    cloud.transformInPlace(sensorOffset);
  }

  void DepthImageConverter::compute(Cloud &cloud,
				    const DepthImage &depthImage, 
				    const RGBImage &rgbImage, 				    
				    const Eigen::Isometry3f &sensorOffset) {

    assert(_projector && "DepthImageConverter: missing _projector");
    assert(_statsCalculator && "DepthImageConverter: missing _statsCalculator");
    assert(_pointInformationMatrixCalculator && "DepthImageConverter: missing _pointInformationMatrixCalculator");
    assert(_normalInformationMatrixCalculator && "DepthImageConverter: missing _normalInformationMatrixCalculator");
    assert(depthImage.rows > 0 && depthImage.cols > 0 && "DepthImageConverter: depthImage has zero size");

    cloud.clear();
    _projector->setImageSize(depthImage.rows, depthImage.cols);
        
    // Resizing the temporaries
    if(depthImage.rows != _indexImage.rows || depthImage.cols != _indexImage.cols) {
      _indexImage.create(depthImage.rows, depthImage.cols);
    }

    // Unprojecting
    _projector->setTransform(Eigen::Isometry3f::Identity());
    _projector->unProject(cloud.points(), cloud.gaussians(), _indexImage, depthImage);
    
    _statsCalculator->compute(cloud.normals(),
			      cloud.stats(),
			      cloud.points(),
			      _indexImage);
    cloud.unprojectRGB(rgbImage,_indexImage);
    _pointInformationMatrixCalculator->compute(cloud.pointInformationMatrix(), cloud.stats(), cloud.normals());
    _normalInformationMatrixCalculator->compute(cloud.normalInformationMatrix(), cloud.stats(), cloud.normals());

    cloud.transformInPlace(sensorOffset);
  }

}
