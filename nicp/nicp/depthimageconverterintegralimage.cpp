#include "depthimageconverterintegralimage.h"
#include "statscalculatorintegralimage.h"
#include "sphericalpointprojector.h"
#include "cylindricalpointprojector.h"

#include <opencv2/highgui/highgui.hpp>

namespace nicp {

  DepthImageConverterIntegralImage::DepthImageConverterIntegralImage(PointProjector *projector_,
								     StatsCalculator *statsCalculator_,
								     PointInformationMatrixCalculator *pointInformationMatrixCalculator_,
								     NormalInformationMatrixCalculator *normalInformationMatrixCalculator_) :
    DepthImageConverter(projector_,
			statsCalculator_,
			pointInformationMatrixCalculator_,
			normalInformationMatrixCalculator_) {}

  void DepthImageConverterIntegralImage::compute(Cloud &cloud,
						 const DepthImage &depthImage,
						 const Eigen::Isometry3f &sensorOffset) {
    assert(_projector && "DepthImageConverterIntegralImage: missing _projector");
    assert(_statsCalculator && "DepthImageConverterIntegralImage: missing _statsCalculator");
    assert(_pointInformationMatrixCalculator && "DepthImageConverterIntegralImage: missing _pointInformationMatrixCalculator");
    assert(_normalInformationMatrixCalculator && "DepthImageConverterIntegralImage: missing _normalInformationMatrixCalculator");
    assert(depthImage.rows > 0 && depthImage.cols > 0 && "DepthImageConverterIntegralImage: depthImage has zero size");

    StatsCalculatorIntegralImage *statsCalculator = 0;
    statsCalculator = dynamic_cast<StatsCalculatorIntegralImage*>(_statsCalculator);
    assert(statsCalculator && "DepthImageConverterIntegralImage: _statsCalculator of non type StatsCalculatorIntegralImage");

    const float _normalWorldRadius = statsCalculator->worldRadius();
    cloud.clear();
    // _projector->setImageSize(depthImage.rows, depthImage.cols);

    // Resizing the temporaries
    if (depthImage.rows != _indexImage.rows || depthImage.cols != _indexImage.cols){
      _indexImage.create(depthImage.rows, depthImage.cols);
    }

    // Unprojecting
    // _projector->setTransform(Eigen::Isometry3f::Identity());
    _projector->unProject(cloud.points(), cloud.gaussians(), _indexImage, depthImage);

    // Computing the intervals
    _projector->projectIntervals(statsCalculator->intervalImage(), depthImage, _normalWorldRadius);

    // Compute stats
    // If it is a cylindrical or spherical projector suppress image radius for better normals
    SphericalPointProjector* sphericalProjector = 0;
    CylindricalPointProjector* cylindricalProjector = 0;
    bool suppressImageRadius = false;
    sphericalProjector = dynamic_cast<SphericalPointProjector*>(_projector);
    cylindricalProjector = dynamic_cast<CylindricalPointProjector*>(_projector);
    if(sphericalProjector || cylindricalProjector) { suppressImageRadius = true; }
    statsCalculator->compute(cloud.normals(),
			     cloud.stats(),
			     cloud.points(),
			     _indexImage,
			     suppressImageRadius);

    // Compute information matrices
    _pointInformationMatrixCalculator->compute(cloud.pointInformationMatrix(), cloud.stats(), cloud.normals());
    _normalInformationMatrixCalculator->compute(cloud.normalInformationMatrix(), cloud.stats(), cloud.normals());

    cloud.transformInPlace(sensorOffset);
  }

  void DepthImageConverterIntegralImage::compute(Cloud &cloud,
						 const DepthImage &depthImage,
						 const RGBImage &rgbImage,
						 const Eigen::Isometry3f &sensorOffset) {
    assert(_projector && "DepthImageConverterIntegralImage: missing _projector");
    assert(_statsCalculator && "DepthImageConverterIntegralImage: missing _statsCalculator");
    assert(_pointInformationMatrixCalculator && "DepthImageConverterIntegralImage: missing _pointInformationMatrixCalculator");
    assert(_normalInformationMatrixCalculator && "DepthImageConverterIntegralImage: missing _normalInformationMatrixCalculator");
    assert(depthImage.rows > 0 && depthImage.cols > 0 && "DepthImageConverterIntegralImage: depthImage has zero size");
    StatsCalculatorIntegralImage *statsCalculator = 0;
    statsCalculator = dynamic_cast<StatsCalculatorIntegralImage*>(_statsCalculator);
    assert(statsCalculator && "DepthImageConverterIntegralImage: _statsCalculator of non type StatsCalculatorIntegralImage");


    const float _normalWorldRadius = statsCalculator->worldRadius();
    cloud.clear();
    // _projector->setImageSize(depthImage.rows, depthImage.cols);

    // Resizing the temporaries
    if (depthImage.rows != _indexImage.rows || depthImage.cols != _indexImage.cols){
      _indexImage.create(depthImage.rows, depthImage.cols);
    }

    // Unprojecting
    _projector->setTransform(Eigen::Isometry3f::Identity());
    _projector->unProject(cloud.points(), cloud.gaussians(), _indexImage, depthImage);

    // Computing the intervals
    _projector->projectIntervals(statsCalculator->intervalImage(), depthImage, _normalWorldRadius);

    // Compute stats
    statsCalculator->compute(cloud.normals(),
			     cloud.stats(),
			     cloud.points(),
			     _indexImage);
    cloud.unprojectRGB(rgbImage,_indexImage);

    // Compute information matrices
    _pointInformationMatrixCalculator->compute(cloud.pointInformationMatrix(), cloud.stats(), cloud.normals());
    _normalInformationMatrixCalculator->compute(cloud.normalInformationMatrix(), cloud.stats(), cloud.normals());

    cloud.transformInPlace(sensorOffset);
  }

}
