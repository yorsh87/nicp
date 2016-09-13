#include "nicp_aligner_gui_main_window.h"

#include <stdio.h>
#include <dirent.h>
#include <set>
#include <sys/stat.h>
#include <sys/types.h>

#include <opencv2/highgui/highgui.hpp>

#include "nicp/imageutils.h"

#include "nicp_viewer/drawable_cloud.h"
#include "nicp_viewer/imageview.h"

namespace nicp_viewer {

  NICPAlignerGuiMainWindow::NICPAlignerGuiMainWindow(std::string directory, QWidget *parent, Qt::WindowFlags flags) : QMainWindow(parent, flags) {
    setupUi(this);

    _referenceScene = new QGraphicsScene();
    _currentScene = new QGraphicsScene();
    reference_graphicsView->setScene(_referenceScene);
    current_graphicsView->setScene(_currentScene);
    merge_pushButton->setEnabled(false);

    viewer->setAxisIsDrawn(true);

    // Init file list
    std::set<std::string> filenames = _readDirectory(directory);
    for(std::set<std::string>::const_iterator it = filenames.begin(); it != filenames.end(); ++it) {
      QString listItem(&(*it)[0]);
      if(listItem.endsWith(".pgm", Qt::CaseInsensitive) || listItem.endsWith(".png", Qt::CaseInsensitive)) {
	cloud_selection_listWidget->addItem(listItem);
      }
    }

    // Create class objects
    _depthScale = 0.001f;
    _sensorOffset = Eigen::Isometry3f::Identity();
    _sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

    _pinholeProjector = new nicp::PinholePointProjector();
    _cylindricalProjector = new nicp::CylindricalPointProjector();
    _sphericalProjector = new nicp::SphericalPointProjector();
    _depthImageConverterIntegralImage = new DepthImageConverterIntegralImage();
    _alignerProjective = new nicp::AlignerProjective();
    _alignerNN = new nicp::AlignerNN();
    _aligner = _alignerProjective;
    _selectProjector();
    _statsCalculatorIntegralImage = new nicp::StatsCalculatorIntegralImage();
    _pointInformationMatrixCalculator = new nicp::PointInformationMatrixCalculator();
    _normalInformationMatrixCalculator = new nicp::NormalInformationMatrixCalculator();
    _depthImageConverterIntegralImage->setProjector(_projector);
    _depthImageConverterIntegralImage->setStatsCalculator(_statsCalculatorIntegralImage);
    _depthImageConverterIntegralImage->setPointInformationMatrixCalculator(_pointInformationMatrixCalculator);
    _depthImageConverterIntegralImage->setNormalInformationMatrixCalculator(_normalInformationMatrixCalculator);

    _correspondenceFinderProjective = new nicp::CorrespondenceFinderProjective();
    _correspondenceFinderNN = new nicp::CorrespondenceFinderNN();
    _linearizer = new nicp::Linearizer();
    _linearizer->setAligner(_aligner);
    _alignerProjective->setProjector(_projector);
    _alignerProjective->setCorrespondenceFinder(_correspondenceFinderProjective);
    _alignerProjective->setLinearizer(_linearizer);
    _alignerNN->setProjector(_projector);
    _alignerNN->setCorrespondenceFinder(_correspondenceFinderNN);
    _alignerNN->setLinearizer(_linearizer);
    _aligner->setProjector(_projector);
    _aligner->setLinearizer(_linearizer);
    _correspondenceFinder = _aligner->correspondenceFinder();

    // Init class objects
    visualizationUpdate();
    statsUpdate();
    correspondencesUpdate();
    alignerUpdate();
    projectorsUpdate();

    _pngCounter = 0;
    _nicpCounter = 0;
  }

  NICPAlignerGuiMainWindow::~NICPAlignerGuiMainWindow() {}

  void NICPAlignerGuiMainWindow::projectorsUpdate() {
    _pinholeProjector->setImageSize(rows_spinBox->value(), cols_spinBox->value());
    _cylindricalProjector->setImageSize(rows_spinBox->value(), cols_spinBox->value());
    _sphericalProjector->setImageSize(rows_spinBox->value(), cols_spinBox->value());
    _pinholeProjector->setMinDistance(min_distance_doubleSpinBox->value());
    _cylindricalProjector->setMinDistance(min_distance_doubleSpinBox->value());
    _sphericalProjector->setMinDistance(min_distance_doubleSpinBox->value());
    _pinholeProjector->setMaxDistance(max_distance_doubleSpinBox->value());
    _cylindricalProjector->setMaxDistance(max_distance_doubleSpinBox->value());
    _sphericalProjector->setMaxDistance(max_distance_doubleSpinBox->value());

    Eigen::Matrix3f cameraMatrix;
    cameraMatrix <<
      fx_doubleSpinBox->value(), 0.0f, cx_doubleSpinBox->value(),
      0.0f, fy_doubleSpinBox->value(), cy_doubleSpinBox->value(),
      0.0f, 0.0f, 1.0f;
    _pinholeProjector->setCameraMatrix(cameraMatrix);
    _pinholeProjector->setBaseline(baseline_doubleSpinBox->value());
    _pinholeProjector->setAlpha(alpha_doubleSpinBox->value());

    _cylindricalProjector->setAngularFov(cyl_horizontal_fov_doubleSpinBox->value());
    float verticalFocalLength = (rows_spinBox->value() / 2.0f) / tanf(cyl_vertical_fov_doubleSpinBox->value());
    _cylindricalProjector->setVerticalFocalLength(verticalFocalLength);
    _cylindricalProjector->setVerticalCenter(vertical_center_spinBox->value());

    _sphericalProjector->setHorizontalFov(sph_horizontal_fov_doubleSpinBox->value());
    _sphericalProjector->setVerticalFov(sph_vertical_fov_doubleSpinBox->value());

    _pinholeProjector->scale(1.0f / scale_doubleSpinBox->value());
    _cylindricalProjector->scale(1.0f / scale_doubleSpinBox->value());
    _sphericalProjector->scale(1.0f / scale_doubleSpinBox->value());

    _correspondenceFinderProjective->setImageSize(rows_spinBox->value(), cols_spinBox->value());
  }

  void NICPAlignerGuiMainWindow::statsUpdate() {
    _statsCalculatorIntegralImage->setMinImageRadius(min_image_radius_spinBox->value());
    _statsCalculatorIntegralImage->setMaxImageRadius(max_image_radius_spinBox->value());
    _statsCalculatorIntegralImage->setMinPoints(min_points_spinBox->value());
    _statsCalculatorIntegralImage->setWorldRadius(world_radius_doubleSpinBox->value());
    _statsCalculatorIntegralImage->setCurvatureThreshold(curv_threshold_doubleSpinBox->value());
  }

  void NICPAlignerGuiMainWindow::correspondencesUpdate() {
    _correspondenceFinderProjective->setInlierNormalAngularThreshold(normal_angle_doubleSpinBox->value());
    _correspondenceFinderProjective->setFlatCurvatureThreshold(curv_flatness_doubleSpinBox->value());
    _correspondenceFinderProjective->setInlierCurvatureRatioThreshold(curv_ratio_doubleSpinBox->value());
    _correspondenceFinderProjective->setInlierDistanceThreshold(point_distance_doubleSpinBox->value());
    _correspondenceFinderNN->setInlierNormalAngularThreshold(normal_angle_doubleSpinBox->value());
    _correspondenceFinderNN->setFlatCurvatureThreshold(curv_flatness_doubleSpinBox->value());
    _correspondenceFinderNN->setInlierCurvatureRatioThreshold(curv_ratio_doubleSpinBox->value());
    _correspondenceFinderNN->setInlierDistanceThreshold(point_distance_doubleSpinBox->value());
    _pointInformationMatrixCalculator->setCurvatureThreshold(curv_flatness_doubleSpinBox->value());
    _normalInformationMatrixCalculator->setCurvatureThreshold(curv_flatness_doubleSpinBox->value());
  }

  void NICPAlignerGuiMainWindow::alignerUpdate() {
    _alignerProjective->setInnerIterations(inner_iter_spinBox->value());
    _alignerProjective->setOuterIterations(outer_iter_spinBox->value());
    _alignerProjective->setMinInliers(min_inliers_spinBox->value());
    _alignerNN->setInnerIterations(inner_iter_spinBox->value());
    _alignerNN->setOuterIterations(outer_iter_spinBox->value());
    _alignerNN->setMinInliers(min_inliers_spinBox->value());
    _linearizer->setInlierMaxChi2(max_chi2_doubleSpinBox->value());
  }

  void NICPAlignerGuiMainWindow::visualizationUpdate() {
    for(size_t i = 0; i < viewer->drawableList().size(); ++i) {
      nicp_viewer::DrawableCloud *dCloud = dynamic_cast<nicp_viewer::DrawableCloud*>(viewer->drawableList()[i]);
      nicp_viewer::GLParameterCloud *pCloud = dynamic_cast<nicp_viewer::GLParameterCloud*>(viewer->drawableList()[i]->parameter());
      if(!dCloud || ! pCloud) { continue; }
      pCloud->setStep(step_spinBox->value());
      pCloud->setPointSize(points_doubleSpinBox->value());
      pCloud->parameterNormals()->setNormalLength(normals_length_doubleSpinBox->value());
      pCloud->parameterNormals()->setLineWidth(normals_width_doubleSpinBox->value());
      pCloud->parameterCovariances()->setEllipsoidScale(covariances_doubleSpinBox->value());
      pCloud->parameterCorrespondences()->setLineWidth(correspondences_doubleSpinBox->value());
      dCloud->drawablePoints()->updatePointDrawList();
      dCloud->drawableNormals()->updateNormalDrawList();
      dCloud->drawableCovariances()->updateCovarianceDrawList();
      dCloud->drawableCorrespondences()->updateCorrespondenceDrawList();
    }
    viewer->updateGL();
  }

  void NICPAlignerGuiMainWindow::addCloud() {
    _selectProjector();
    QList<QListWidgetItem*> selectedItems = cloud_selection_listWidget->selectedItems();
    for(int i = 0; i < selectedItems.size(); ++i) {
      std::string filename = selectedItems[i]->text().toUtf8().constData();
      _rawDepth = cv::imread(filename, -1);
      if(_rawDepth.data == NULL) {
	std::cerr << "[WARNING] Impossible to load selected .pgm depth image" << std::endl;
	continue;
      }
      _referenceDepth = _currentDepth.clone();
      nicp::DepthImage_convert_16UC1_to_32FC1(_currentDepth, _rawDepth, _depthScale);
      nicp::DepthImage_scale(_scaledDepth, _currentDepth, scale_doubleSpinBox->value());
      _currentDepth = _scaledDepth.clone();
      nicp::Cloud *cloud = new Cloud();
      _depthImageConverterIntegralImage->compute(*cloud, _currentDepth, _sensorOffset);
      _clouds.push_back(cloud);

      Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
      if(_poses.size() > 0) { pose = _poses.back(); }
      pose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      _poses.push_back(pose);
      nicp_viewer::GLParameterCloud *rCloud = new nicp_viewer::GLParameterCloud();
      rCloud->parameterPoints()->setColor(Eigen::Vector4f(1.0f, 1.0f, 0.0f, 1.0f));
      rCloud->parameterNormals()->setColor(Eigen::Vector4f(1.0f, 0.5f, 0.0f, 1.0f));
      nicp_viewer::DrawableCloud *dCloud = new nicp_viewer::DrawableCloud(_poses.back(), rCloud, cloud);
      viewer->addDrawable(dCloud);
      _referenceScene->clear();
      _currentScene->clear();
      QImage referenceQImage;
      QImage *currentQImage = new QImage();
      nicp_viewer::DepthImageView depthImageView;
      depthImageView.computeColorMap((int)min_distance_doubleSpinBox->value() * 1000.0f, (int)max_distance_doubleSpinBox->value() * 1000.0f, 255);
      depthImageView.convertToQImage(referenceQImage, _referenceDepth);
      depthImageView.convertToQImage(*currentQImage, _currentDepth);
      _depths.push_back(currentQImage);
      _referenceScene->addPixmap(QPixmap::fromImage(referenceQImage));
      _currentScene->addPixmap(QPixmap::fromImage(*currentQImage));
      reference_graphicsView->fitInView(_referenceScene->itemsBoundingRect(), Qt::KeepAspectRatio);
      current_graphicsView->fitInView(_currentScene->itemsBoundingRect(), Qt::KeepAspectRatio);
      reference_graphicsView->show();
      current_graphicsView->show();
      visualizationUpdate();
      viewer->updateGL();
      std::cout << "[STATUS] Added new cloud of " << cloud->points().size() << " points from file " << filename << ", the drawable queue size now is: "
		<< viewer->drawableList().size() << std::endl;

    }
  }

  void NICPAlignerGuiMainWindow::clearLast() {
    if(viewer->drawableList().size() > 0) {
      nicp_viewer::Drawable *d = viewer->drawableList().back();
      viewer->popBack();
      delete d->parameter();
      delete d;

      nicp::Cloud *c = _clouds.back();
      _clouds.pop_back();
      delete c;

      QImage *qi = _depths.back();
      _depths.pop_back();
      delete qi;

      _poses.pop_back();
    }
    else { return; }
    _referenceScene->clear();
    _currentScene->clear();
    if(_depths.size() > 0) {
      if(_depths.size() > 1) { _referenceScene->addPixmap(QPixmap::fromImage(*_depths[_depths.size() - 2])); }
      _currentScene->addPixmap(QPixmap::fromImage(*_depths.back()));
      reference_graphicsView->fitInView(_referenceScene->itemsBoundingRect(), Qt::KeepAspectRatio);
      current_graphicsView->fitInView(_currentScene->itemsBoundingRect(), Qt::KeepAspectRatio);
      reference_graphicsView->show();
      current_graphicsView->show();
    }
    viewer->updateGL();
    std::cout << "[STATUS] Deleted last loaded depth images, the drawable queue size now is: "
	      << viewer->drawableList().size() << std::endl;
  }

  void NICPAlignerGuiMainWindow::clearAll() {
    while(viewer->drawableList().size() > 0) {
      nicp_viewer::Drawable *d = viewer->drawableList().back();
      viewer->popBack();
      delete d->parameter();
      delete d;

      nicp::Cloud *c = _clouds.back();
      _clouds.pop_back();
      delete c;

      QImage *qi = _depths.back();
      _depths.pop_back();
      delete qi;
    }
    _poses.clear();
    _referenceScene->clear();
    _currentScene->clear();
    viewer->updateGL();
    std::cout << "[STATUS] Deleted all previously loaded depth images, the drawable queue size now is: "
	      << viewer->drawableList().size() << std::endl;
  }

  void NICPAlignerGuiMainWindow::nicpSnapshot() {
    nicp::Cloud *globalCloud = new nicp::Cloud();
    for(size_t i = 0; i < _clouds.size(); ++i) { globalCloud->add(*_clouds[i], _poses[i]); }
    char buffer[1024];
    sprintf(buffer, "nicp_aligner_gui_snapshot_%03d.nicp", _nicpCounter);
    globalCloud->save(buffer, Eigen::Isometry3f::Identity(), 1, true);
    delete globalCloud;
    _nicpCounter++;
    std::cout << "[STATUS] Saved cloud on file " << buffer << std::endl;
  }

  void NICPAlignerGuiMainWindow::jpgSnapshot() {
    char buffer[1024];
    sprintf(buffer, "nicp_aligner_gui_snapshot_%03d.jpg", _pngCounter);
    viewer->setSnapshotQuality(100);
    viewer->saveSnapshot(QString(buffer), true);
    _pngCounter++;
    std::cout << "[STATUS] Saved viewer snapshot on file " << buffer << std::endl;
  }

  void NICPAlignerGuiMainWindow::optimize() {
    std::cerr << "sc: " << _statsCalculatorIntegralImage->minImageRadius() << " " << _statsCalculatorIntegralImage->maxImageRadius() << " "
	      << _statsCalculatorIntegralImage->minPoints() << " " << _statsCalculatorIntegralImage->worldRadius() << " "
	      << _statsCalculatorIntegralImage->curvatureThreshold() << std::endl;
    std::cerr << "im: " << _pointInformationMatrixCalculator->curvatureThreshold() << " " << _normalInformationMatrixCalculator->curvatureThreshold() << std::endl;
    std::cerr << "cfp: " << _correspondenceFinderProjective->imageRows() << " " << _correspondenceFinderProjective->imageCols() << " "
	      << _correspondenceFinderProjective->inlierDistanceThreshold() << " " << _correspondenceFinderProjective->inlierNormalAngularThreshold() << " "
	      << _correspondenceFinderProjective->inlierCurvatureRatioThreshold() << " " << _correspondenceFinderProjective->flatCurvatureThreshold() << std::endl;
    std::cerr << "cfnn: " << _correspondenceFinderNN->inlierDistanceThreshold() << " " << _correspondenceFinderNN->inlierNormalAngularThreshold() << " "
	      << _correspondenceFinderNN->inlierCurvatureRatioThreshold() << " " << _correspondenceFinderNN->flatCurvatureThreshold() << std::endl;
    std::cerr << "al: " << _aligner->innerIterations() << " " << _aligner->outerIterations() << " "
	      << _aligner->minInliers() << " " << _linearizer->inlierMaxChi2() << std::endl;
    if(viewer->drawableList().size() < 2) { return; }

    _selectProjector();
    nicp::Cloud *referenceCloud = _clouds[_clouds.size() - 2];
    nicp::Cloud *currentCloud = _clouds.back();
    _aligner->clearPriors();
    _aligner->setReferenceCloud(referenceCloud);
    _aligner->setCurrentCloud(currentCloud);
    if(step_by_step_checkBox->isChecked()) {
      _aligner->setOuterIterations(1);
      _aligner->setInitialGuess(_poses.back() * _poses[_poses.size() - 2].inverse());
    }
    else {
      _aligner->setOuterIterations(outer_iter_spinBox->value());
      _aligner->setInitialGuess(Eigen::Isometry3f::Identity());
    }
    _aligner->setSensorOffset(_sensorOffset);
    clock_t start, end;
    start = clock();
    _aligner->align();
    end = clock();
    std::cout << "[STATUS] Alignment time: " << (end - start) / 1000.0f << " ms" << std::endl;
    std::cout << "[STATUS] T: " << std::endl << _aligner->T().matrix() << std::endl;

    _poses.back() = _aligner->T() * _poses[_poses.size() - 2];
    nicp_viewer::DrawableCloud *dCloud = dynamic_cast<nicp_viewer::DrawableCloud*>(viewer->drawableList().back());
    if(dCloud) {
      dCloud->setTransformation(_poses.back());
      dCloud->clearDrawableObjects();
      dCloud->constructDrawableObjects();
      dCloud->drawableCorrespondences()->setReferencePointsTransformation((_poses[_poses.size() - 2].inverse() * _poses.back()).inverse());
      dCloud->drawableCorrespondences()->setReferencePoints(&referenceCloud->points());
      dCloud->drawableCorrespondences()->setCurrentPoints(&currentCloud->points());
      dCloud->drawableCorrespondences()->setCorrespondences(&_correspondenceFinder->correspondences());
      dCloud->drawableCorrespondences()->setNumCorrespondences(_correspondenceFinder->numCorrespondences());
      dCloud->drawableCorrespondences()->parameter()->setShow(true);
    }
    dCloud = dynamic_cast<nicp_viewer::DrawableCloud*>(viewer->drawableList()[viewer->drawableList().size() - 2]);
    if(dCloud) { dCloud->drawableCorrespondences()->parameter()->setShow(false); }
    std::cout << "[STATUS] Error: " << _aligner->error() << std::endl;
    std::cout << "[STATUS] Inliers: " << _aligner->inliers() << std::endl;
    _currentScene->clear();
    QImage currentQImage;
    nicp_viewer::DepthImageView div;
    div.computeColorMap((int)min_distance_doubleSpinBox->value() * 1000.0f, (int)max_distance_doubleSpinBox->value() * 1000.0f, 255);
    _projector->setTransform(_aligner->T().inverse() * _aligner->sensorOffset());
    _projector->project(_correspondenceFinderProjective->currentIndexImage(),
			_correspondenceFinderProjective->currentDepthImage(),
			currentCloud->points());
    div.convertToQImage(currentQImage, _correspondenceFinderProjective->currentDepthImage());
    _currentScene->addPixmap(QPixmap::fromImage(currentQImage));
    current_graphicsView->fitInView(_currentScene->itemsBoundingRect(), Qt::KeepAspectRatio);
    current_graphicsView->show();
    visualizationUpdate();
    viewer->updateGL();
  }

  void NICPAlignerGuiMainWindow::correspondences() {
    if(viewer->drawableList().size() < 2) { return; }
    initialGuess();
    _selectProjector();
    nicp::Cloud *referenceCloud = _clouds[_clouds.size() - 2];
    nicp::Cloud *currentCloud = _clouds.back();

    // Projective Correspondence Finder
    _projector->setTransform(_sensorOffset);
    _projector->project(_correspondenceFinderProjective->currentIndexImage(),
    			_correspondenceFinderProjective->currentDepthImage(),
    			currentCloud->points());
    _projector->setTransform(_sensorOffset);
    _projector->project(_correspondenceFinderProjective->referenceIndexImage(),
    			_correspondenceFinderProjective->referenceDepthImage(),
    			referenceCloud->points());
    _correspondenceFinder->compute(*referenceCloud, *currentCloud, (_poses[_poses.size() - 2].inverse() * _poses.back()).inverse());

    // NN Correspondence Finder
    // _correspondenceFinderNN->init(*referenceCloud, *currentCloud);
    // _correspondenceFinder->compute(*referenceCloud, *currentCloud, (_poses[_poses.size() - 2].inverse() * _poses.back()).inverse());

    nicp_viewer::Drawable *d = viewer->drawableList().back();
    nicp_viewer::DrawableCloud *dCloud = dynamic_cast<nicp_viewer::DrawableCloud*>(d);
    if(dCloud) {
      dCloud->setTransformation(_poses.back());
      dCloud->clearDrawableObjects();
      dCloud->constructDrawableObjects();
      dCloud->drawableCorrespondences()->setReferencePointsTransformation((_poses[_poses.size() - 2].inverse() * _poses.back()).inverse());
      dCloud->drawableCorrespondences()->setReferencePoints(&referenceCloud->points());
      dCloud->drawableCorrespondences()->setCurrentPoints(&currentCloud->points());
      dCloud->drawableCorrespondences()->setCorrespondences(&_correspondenceFinder->correspondences());
      dCloud->drawableCorrespondences()->setNumCorrespondences(_correspondenceFinder->numCorrespondences());
      dCloud->drawableCorrespondences()->parameter()->setShow(true);
      std::cout << "[STATUS] " << _correspondenceFinder->numCorrespondences() << " correspondences computed" << std::endl;
    }
    visualizationUpdate();
    viewer->updateGL();
  }

  void NICPAlignerGuiMainWindow::initialGuess() {
    if(viewer->drawableList().size() < 2) { return; }
    _poses.back() = _poses[_poses.size() - 2];
    nicp_viewer::Drawable *d = viewer->drawableList().back();
    nicp_viewer::DrawableCloud *dCloud = dynamic_cast<nicp_viewer::DrawableCloud*>(d);
    if(dCloud) {
      dCloud->setTransformation(_poses[_poses.size() - 2]);
      dCloud->drawableCorrespondences()->setReferencePointsTransformation((_poses[_poses.size() - 2].inverse() * _poses.back()).inverse());
      dCloud->drawableCorrespondences()->parameter()->setShow(true);

      _currentScene->clear();
      _currentScene->addPixmap(QPixmap::fromImage(*_depths.back()));
      current_graphicsView->fitInView(_currentScene->itemsBoundingRect(), Qt::KeepAspectRatio);
      current_graphicsView->show();
      std::cout << "[STATUS] Initial guess imposed" << std::endl;
    }
    viewer->updateGL();
  }

  void NICPAlignerGuiMainWindow::merge() { }

  std::set<string> NICPAlignerGuiMainWindow::_readDirectory(std::string directory) {
    DIR *dp;
    struct dirent *dirp;
    struct stat filestat;
    std::set<std::string> filenames;
    dp = opendir(directory.c_str());
    if(dp == NULL) { return filenames; }
    while((dirp = readdir(dp))) {
      std::string filepath = directory + "/" + dirp->d_name;
      if(stat(filepath.c_str(), &filestat)) { continue; }
      if(S_ISDIR(filestat.st_mode)) { continue; }
      filenames.insert(filepath);
    }
    closedir(dp);
    return filenames;
  }

  void NICPAlignerGuiMainWindow::_selectProjector() {
    if(pinhole_projector_radioButton->isChecked()) {
      _projector = _pinholeProjector;
      std::cout << "[STATUS] Using pinhole projector" << std::endl;
    }
    if(cylindrical_projector_radioButton->isChecked()) {
      _projector = _cylindricalProjector;
      std::cout << "[STATUS] Using cylindrical projector" << std::endl;
    }
    if(spherical_projector_radioButton->isChecked()) {
      _projector = _sphericalProjector;
      std::cout << "[STATUS] Using spherical projector" << std::endl;
    }

    _depthImageConverterIntegralImage->setProjector(_projector);
    _aligner->setProjector(_projector);
  }

}
