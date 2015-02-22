#include "pwn_aligner_gui_main_window.h"

#include <stdio.h>
#include <dirent.h>
#include <set>
#include <sys/stat.h>
#include <sys/types.h>

#include <opencv2/highgui/highgui.hpp>

#include "pwn/imageutils.h"

#include "pwn_viewer/drawable_cloud.h"
#include "pwn_viewer/imageview.h"

namespace pwn_viewer {

  PwnAlignerGuiMainWindow::PwnAlignerGuiMainWindow(std::string directory, QWidget *parent, Qt::WindowFlags flags) : QMainWindow(parent, flags) { 
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
    _depthScale = 0.001;
    _sensorOffset = Eigen::Isometry3f::Identity();
    _sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

    _pinholeProjector = new pwn::PinholePointProjector();
    _cylindricalProjector = new pwn::CylindricalPointProjector();
    _sphericalProjector = new pwn::SphericalPointProjector();
    _depthImageConverterIntegralImage = new DepthImageConverterIntegralImage();
    _aligner = new pwn::Aligner();   
    _selectProjector();
    _statsCalculatorIntegralImage = new pwn::StatsCalculatorIntegralImage();
    _pointInformationMatrixCalculator = new pwn::PointInformationMatrixCalculator(); 
    _normalInformationMatrixCalculator = new pwn::NormalInformationMatrixCalculator(); 
    _depthImageConverterIntegralImage->setProjector(_projector);
    _depthImageConverterIntegralImage->setStatsCalculator(_statsCalculatorIntegralImage);
    _depthImageConverterIntegralImage->setPointInformationMatrixCalculator(_pointInformationMatrixCalculator);
    _depthImageConverterIntegralImage->setNormalInformationMatrixCalculator(_normalInformationMatrixCalculator);

    _correspondenceFinder = new pwn::CorrespondenceFinder(); 
    _linearizer = new pwn::Linearizer(); 
    _linearizer->setAligner(_aligner);
    _aligner->setProjector(_projector);
    _aligner->setCorrespondenceFinder(_correspondenceFinder);
    _aligner->setLinearizer(_linearizer);
  
    // Init class objects
    visualizationUpdate();
    statsUpdate();
    correspondencesUpdate();
    alignerUpdate();
    projectorsUpdate();

    _pngCounter = 0;
    _pwnCounter = 0;
  }

  PwnAlignerGuiMainWindow::~PwnAlignerGuiMainWindow() {}

  void PwnAlignerGuiMainWindow::projectorsUpdate() { 
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

    _correspondenceFinder->setImageSize(rows_spinBox->value(), cols_spinBox->value());
  }

  void PwnAlignerGuiMainWindow::statsUpdate() { 
    _statsCalculatorIntegralImage->setMinImageRadius(min_image_radius_spinBox->value());
    _statsCalculatorIntegralImage->setMaxImageRadius(max_image_radius_spinBox->value());
    _statsCalculatorIntegralImage->setMinPoints(min_points_spinBox->value());
    _statsCalculatorIntegralImage->setWorldRadius(world_radius_doubleSpinBox->value());
    _statsCalculatorIntegralImage->setCurvatureThreshold(curv_threshold_doubleSpinBox->value());
  }

  void PwnAlignerGuiMainWindow::correspondencesUpdate() { 
    _correspondenceFinder->setInlierNormalAngularThreshold(cosf(normal_angle_doubleSpinBox->value()));
    _correspondenceFinder->setFlatCurvatureThreshold(curv_flatness_doubleSpinBox->value());
    _correspondenceFinder->setInlierCurvatureRatioThreshold(curv_ratio_doubleSpinBox->value());
    _correspondenceFinder->setInlierDistanceThreshold(point_distance_doubleSpinBox->value());
    _pointInformationMatrixCalculator->setCurvatureThreshold(curv_flatness_doubleSpinBox->value());
    _normalInformationMatrixCalculator->setCurvatureThreshold(curv_flatness_doubleSpinBox->value());
  }

  void PwnAlignerGuiMainWindow::alignerUpdate() { 
    _aligner->setInnerIterations(inner_iter_spinBox->value());
    _aligner->setOuterIterations(outer_iter_spinBox->value());
    _aligner->setMinInliers(min_inliers_spinBox->value());  
    _linearizer->setInlierMaxChi2(max_chi2_doubleSpinBox->value());
  }

  void PwnAlignerGuiMainWindow::visualizationUpdate() { 
    for(size_t i = 0; i < viewer->drawableList().size(); ++i) {
      pwn_viewer::DrawableCloud *dCloud = dynamic_cast<pwn_viewer::DrawableCloud*>(viewer->drawableList()[i]);
      pwn_viewer::GLParameterCloud *pCloud = dynamic_cast<pwn_viewer::GLParameterCloud*>(viewer->drawableList()[i]->parameter());
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

  void PwnAlignerGuiMainWindow::addCloud() { 
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
      pwn::DepthImage_convert_16UC1_to_32FC1(_currentDepth, _rawDepth, _depthScale);
      pwn::DepthImage_scale(_scaledDepth, _currentDepth, scale_doubleSpinBox->value());
      _currentDepth = _scaledDepth.clone();
      pwn::Cloud *cloud = new Cloud();
      _depthImageConverterIntegralImage->compute(*cloud, _currentDepth, _sensorOffset);
      _clouds.push_back(cloud);

      Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
      if(_poses.size() > 0) { pose = _poses.back(); }
      pose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;      
      _poses.push_back(pose);
      pwn_viewer::GLParameterCloud *rCloud = new pwn_viewer::GLParameterCloud();
      rCloud->parameterPoints()->setColor(Eigen::Vector4f(1.0f, 1.0f, 0.0f, 1.0f));
      rCloud->parameterNormals()->setColor(Eigen::Vector4f(0.0f, 1.0f, 1.0f, 1.0f));
      pwn_viewer::DrawableCloud *dCloud = new pwn_viewer::DrawableCloud(_poses.back(), rCloud, cloud);
      viewer->addDrawable(dCloud);
      _referenceScene->clear();
      _currentScene->clear();
      QImage referenceQImage;
      QImage *currentQImage = new QImage();
      pwn_viewer::DepthImageView depthImageView;
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

  void PwnAlignerGuiMainWindow::clearLast() {
    if(viewer->drawableList().size() > 0) {
      pwn_viewer::Drawable *d = viewer->drawableList().back();    
      viewer->popBack();
      delete d->parameter();
      delete d;	

      pwn::Cloud *c = _clouds.back();    
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

  void PwnAlignerGuiMainWindow::clearAll() { 
    while(viewer->drawableList().size() > 0) {
      pwn_viewer::Drawable *d = viewer->drawableList().back();    
      viewer->popBack();
      delete d->parameter();
      delete d;	

      pwn::Cloud *c = _clouds.back();    
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

  void PwnAlignerGuiMainWindow::pwnSnapshot() {  
    pwn::Cloud *globalCloud = new pwn::Cloud();
    for(size_t i = 0; i < _clouds.size(); ++i) { globalCloud->add(*_clouds[i], _poses[i]); }
    char buffer[1024];
    sprintf(buffer, "pwn_aligner_gui_snapshot_%03d.pwn", _pwnCounter);
    globalCloud->save(buffer, Eigen::Isometry3f::Identity(), 1, true);
    delete globalCloud;
    _pwnCounter++;
    std::cout << "[STATUS] Saved cloud on file " << buffer << std::endl;
  }

  void PwnAlignerGuiMainWindow::jpgSnapshot() { 
    char buffer[1024];
    sprintf(buffer, "pwn_aligner_gui_snapshot_%03d.jpg", _pngCounter);
    viewer->setSnapshotQuality(100);
    viewer->saveSnapshot(QString(buffer), true);
    _pngCounter++;
    std::cout << "[STATUS] Saved viewer snapshot on file " << buffer << std::endl;
  }

  void PwnAlignerGuiMainWindow::optimize() { 
    std::cerr << "sc: " << _statsCalculatorIntegralImage->minImageRadius() << " " << _statsCalculatorIntegralImage->maxImageRadius() << " " 
	      << _statsCalculatorIntegralImage->minPoints() << " " << _statsCalculatorIntegralImage->worldRadius() << " "
	      << _statsCalculatorIntegralImage->curvatureThreshold() << std::endl;
    std::cerr << "im: " << _pointInformationMatrixCalculator->curvatureThreshold() << " " << _normalInformationMatrixCalculator->curvatureThreshold() << std::endl;
    std::cerr << "cf: " << _correspondenceFinder->imageRows() << " " << _correspondenceFinder->imageCols() << " " 
	      << _correspondenceFinder->inlierDistanceThreshold() << " " << _correspondenceFinder->inlierNormalAngularThreshold() << " "
	      << _correspondenceFinder->inlierCurvatureRatioThreshold() << " " << _correspondenceFinder->flatCurvatureThreshold() << std::endl;
    std::cerr << "al: " << _aligner->innerIterations() << " " << _aligner->outerIterations() << " " 
	      << _aligner->minInliers() << " " << _linearizer->inlierMaxChi2() << std::endl;
    if(viewer->drawableList().size() < 2) { return; }

    _selectProjector();
    pwn::Cloud *referenceCloud = _clouds[_clouds.size() - 2];  
    pwn::Cloud *currentCloud = _clouds.back();  
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
    pwn_viewer::DrawableCloud *dCloud = dynamic_cast<pwn_viewer::DrawableCloud*>(viewer->drawableList().back());
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
    dCloud = dynamic_cast<pwn_viewer::DrawableCloud*>(viewer->drawableList()[viewer->drawableList().size() - 2]);
    if(dCloud) { dCloud->drawableCorrespondences()->parameter()->setShow(false); }
    std::cout << "[STATUS] Error: " << _aligner->error() << std::endl;
    std::cout << "[STATUS] Inliers: " << _aligner->inliers() << std::endl;
    _currentScene->clear();
    QImage currentQImage;
    pwn_viewer::DepthImageView div;
    div.computeColorMap((int)min_distance_doubleSpinBox->value() * 1000.0f, (int)max_distance_doubleSpinBox->value() * 1000.0f, 255);
    _projector->setTransform(_aligner->T().inverse() * _aligner->sensorOffset());
    _projector->project(_correspondenceFinder->currentIndexImage(),
			_correspondenceFinder->currentDepthImage(),
			currentCloud->points());
    div.convertToQImage(currentQImage, _correspondenceFinder->currentDepthImage());
    _currentScene->addPixmap(QPixmap::fromImage(currentQImage));
    current_graphicsView->fitInView(_currentScene->itemsBoundingRect(), Qt::KeepAspectRatio);
    current_graphicsView->show();
    visualizationUpdate();
    viewer->updateGL();
  }

  void PwnAlignerGuiMainWindow::correspondences() { 
    if(viewer->drawableList().size() < 2) { return; }
    initialGuess();
    _selectProjector();
    pwn::Cloud *referenceCloud = _clouds[_clouds.size() - 2];  
    pwn::Cloud *currentCloud = _clouds.back();  
    _projector->setTransform(_sensorOffset);
    _projector->project(_correspondenceFinder->currentIndexImage(),
			_correspondenceFinder->currentDepthImage(),
			currentCloud->points());        
    _projector->setTransform(_sensorOffset);
    _projector->project(_correspondenceFinder->referenceIndexImage(),
			_correspondenceFinder->referenceDepthImage(),
			referenceCloud->points());
    _correspondenceFinder->compute(*referenceCloud, *currentCloud, (_poses[_poses.size() - 2].inverse() * _poses.back()).inverse());

    pwn_viewer::Drawable *d = viewer->drawableList().back();
    pwn_viewer::DrawableCloud *dCloud = dynamic_cast<pwn_viewer::DrawableCloud*>(d);
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
      std::cout << "[STATUS] Correspondences computed" << std::endl;
    }
    visualizationUpdate();
    viewer->updateGL();
  }

  void PwnAlignerGuiMainWindow::initialGuess() {
    if(viewer->drawableList().size() < 2) { return; }
    _poses.back() = _poses[_poses.size() - 2];
    pwn_viewer::Drawable *d = viewer->drawableList().back();
    pwn_viewer::DrawableCloud *dCloud = dynamic_cast<pwn_viewer::DrawableCloud*>(d);
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

  void PwnAlignerGuiMainWindow::merge() { }

  std::set<string> PwnAlignerGuiMainWindow::_readDirectory(std::string directory) {
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

  void PwnAlignerGuiMainWindow::_selectProjector() {
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
