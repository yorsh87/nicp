#include <iostream>
#include <fstream>
#include <unistd.h>

#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QListWidget>
#include <QCheckBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>

#include <opencv2/highgui/highgui.hpp>

#include "nicp/imageutils.h"
#include "nicp/pinholepointprojector.h"
#include "nicp/statscalculatorintegralimage.h"
#include "nicp/depthimageconverterintegralimage.h"

#include "nicp_qglviewer.h"
#include "drawable_points.h"
#include "gl_parameter_points.h"
#include "drawable_normals.h"
#include "gl_parameter_normals.h"
#include "drawable_covariances.h"
#include "gl_parameter_covariances.h"
#include "drawable_uncertainty.h"
#include "gl_parameter_uncertainty.h"

using namespace Eigen;
using namespace std;
using namespace nicp_viewer;

int main(int argc, char **argv) {
  // Input handling
  if(argc < 2 || 
     string(argv[1]) == "-h" || string(argv[1]) == "-help" || 
     string(argv[1]) == "--h" || string(argv[1]) == "--help") {
    std::cout << "USAGE: ";
    std::cout << "nicp_cloud_prop_viewer cloudFilename.pgm" << std::endl;
    std::cout << "   cloudFilename{.pgm,.nicp} \t-->\t input depth image or point cloud filename" << std::endl;
    return 0;
  }  
  string filename(argv[1]);
  if(filename.substr(filename.size() - 3) != "pgm" && filename.substr(filename.size() - 3) != "nicp") {
    std::cerr << "ERROR: input filename " << filename << " of unknown extension ... quitting!" << std::endl;
    return 0;
  }

  // Create GUI
  QApplication application(argc, argv);
  QWidget *mainWindow = new QWidget();
  mainWindow->setWindowTitle("nicp_cloud_prop_viewer");
  QVBoxLayout *base_layout = new QVBoxLayout();
  mainWindow->setLayout(base_layout);
  QHBoxLayout *viewer_layout = new QHBoxLayout();
  base_layout->addItem(viewer_layout);
  QHBoxLayout *settings_layout = new QHBoxLayout();
  base_layout->addItem(settings_layout);
  QGridLayout *properties_layout = new QGridLayout();  
  settings_layout->addItem(properties_layout);
  base_layout->setStretch(0, 1);
  base_layout->setStretch(1, 0);

  NICPQGLViewer *nicp_qglviewer = new NICPQGLViewer(mainWindow);
  viewer_layout->addWidget(nicp_qglviewer);
  nicp_qglviewer->init();
  nicp_qglviewer->setAxisIsDrawn(true);

  QLabel *stepLabel = new QLabel("Step", mainWindow);
  QSpinBox *stepSpinBox = new QSpinBox(mainWindow);
  stepSpinBox->setSingleStep(1);
  stepSpinBox->setValue(1);
  QCheckBox *stepCheckBox = new QCheckBox(mainWindow);
  stepCheckBox->setChecked(true);
  properties_layout->addWidget(stepLabel, 0, 0, Qt::AlignCenter);
  properties_layout->addWidget(stepSpinBox, 1, 0, Qt::AlignCenter);
  properties_layout->addWidget(stepCheckBox, 2, 0, Qt::AlignCenter);

  QLabel *pointsLabel = new QLabel("Points", mainWindow);
  QDoubleSpinBox *pointsSpinBox = new QDoubleSpinBox(mainWindow);
  pointsSpinBox->setSingleStep(0.01f);
  pointsSpinBox->setValue(1.0f);
  pointsSpinBox->setDecimals(2);
  QCheckBox *pointsCheckBox = new QCheckBox(mainWindow);
  pointsCheckBox->setChecked(true);
  properties_layout->addWidget(pointsLabel, 0, 1, Qt::AlignCenter);
  properties_layout->addWidget(pointsSpinBox, 1, 1, Qt::AlignCenter);
  properties_layout->addWidget(pointsCheckBox, 2, 1, Qt::AlignCenter);
  
  QLabel *normalsLabel = new QLabel("Normals", mainWindow);
  QDoubleSpinBox *normalsSpinBox = new QDoubleSpinBox(mainWindow);
  normalsSpinBox->setSingleStep(0.01f);
  normalsSpinBox->setValue(0.03f);
  normalsSpinBox->setDecimals(2);
  QCheckBox *normalsCheckBox = new QCheckBox(mainWindow);
  normalsCheckBox->setChecked(false);
  properties_layout->addWidget(normalsLabel, 0, 2, Qt::AlignCenter);
  properties_layout->addWidget(normalsSpinBox, 1, 2, Qt::AlignCenter);
  properties_layout->addWidget(normalsCheckBox, 2, 2, Qt::AlignCenter);

  QLabel *covariancesLabel = new QLabel("Covariances", mainWindow);
  QDoubleSpinBox *covariancesSpinBox = new QDoubleSpinBox(mainWindow);
  covariancesSpinBox->setSingleStep(0.01f);
  covariancesSpinBox->setValue(0.02f);
  covariancesSpinBox->setDecimals(2);
  QCheckBox *covariancesCheckBox = new QCheckBox(mainWindow);
  covariancesCheckBox->setChecked(false);
  properties_layout->addWidget(covariancesLabel, 0, 3, Qt::AlignCenter);
  properties_layout->addWidget(covariancesSpinBox, 1, 3, Qt::AlignCenter);
  properties_layout->addWidget(covariancesCheckBox, 2, 3, Qt::AlignCenter);

  QLabel *uncertaintyLabel = new QLabel("Uncertainty", mainWindow);
  QDoubleSpinBox *uncertaintySpinBox = new QDoubleSpinBox(mainWindow);
  uncertaintySpinBox->setSingleStep(0.01f);
  uncertaintySpinBox->setValue(1.0f);
  uncertaintySpinBox->setDecimals(2);
  QCheckBox *uncertaintyCheckBox = new QCheckBox(mainWindow);
  uncertaintyCheckBox->setChecked(false);
  properties_layout->addWidget(uncertaintyLabel, 0, 4, Qt::AlignCenter);
  properties_layout->addWidget(uncertaintySpinBox, 1, 4, Qt::AlignCenter);
  properties_layout->addWidget(uncertaintyCheckBox, 2, 4, Qt::AlignCenter);

  QPushButton *snapshotButton = new QPushButton("Snapshot", mainWindow);
  properties_layout->addWidget(snapshotButton, 1, 5, Qt::AlignCenter);
  snapshotButton->setFixedSize(snapshotButton->size());

  // Read input cloud
  Matrix3f cameraMatrix;
  cameraMatrix <<
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f,   0.0f,   1.0f;
  Isometry3f sensorOffset = Isometry3f::Identity();
  sensorOffset.translation() = Vector3f(0.0f, 0.0f, 0.0f);
  Quaternionf quaternion = Quaternionf(0.5f, -0.5f, 0.5f, -0.5f);
  sensorOffset.linear() = quaternion.toRotationMatrix();
  sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

  int imageRows = 480, imageCols = 640;
  PinholePointProjector pointProjector;
  pointProjector.setCameraMatrix(cameraMatrix);
  pointProjector.setImageSize(imageRows, imageCols);
  pointProjector.setMaxDistance(2.0f);

  float curvatureThreshold = 1.0f;
  StatsCalculatorIntegralImage statsCalculator;
  statsCalculator.setCurvatureThreshold(curvatureThreshold);

  PointInformationMatrixCalculator pointInformationMatrixCalculator;
  NormalInformationMatrixCalculator normalInformationMatrixCalculator;  
  pointInformationMatrixCalculator.setCurvatureThreshold(curvatureThreshold);
  normalInformationMatrixCalculator.setCurvatureThreshold(curvatureThreshold);

  DepthImageConverterIntegralImage converter(&pointProjector, &statsCalculator,
					     &pointInformationMatrixCalculator, 
					     &normalInformationMatrixCalculator);

  RawDepthImage rawDepthImage;
  DepthImage depthImage;
  Cloud pointCloud;
  
  if(filename.substr(filename.size() - 3) == "pgm") {
    rawDepthImage = cv::imread(filename, -1);
    if(rawDepthImage.data == NULL) {
      std::cerr << "ERROR: impossible to load input .pgm depth image " << filename << " ... quitting!"<<endl;
      return 0;
    } 
    DepthImage_convert_16UC1_to_32FC1(depthImage, rawDepthImage, 0.001f);
    if (argc == 3){
      cv::Mat rawRgbImage = cv::imread(argv[2], -1);
      converter.compute(pointCloud, depthImage, rawRgbImage, sensorOffset); 
    } else {
      converter.compute(pointCloud, depthImage, sensorOffset); 
    }
  }
  else {
    Eigen::Isometry3f tmp;
    pointCloud.load(tmp, filename.c_str());
  }

  // Manage GUI
  mainWindow->show();
  nicp_qglviewer->show();
  GLParameterPoints pointsParameter(1.0f, Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f));
  pointsParameter.setShow(true);
  DrawablePoints pointsDrawable(Eigen::Isometry3f::Identity(), (GLParameter*)&pointsParameter, &pointCloud.points(), &pointCloud.normals());
  if (pointCloud.rgbs().size())
    pointsDrawable.setRGBs(&(pointCloud.rgbs()));
  GLParameterNormals normalsParameter(1.0f, Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f), 0.03f);
  DrawableNormals normalsDrawable(Eigen::Isometry3f::Identity(), (GLParameter*)&normalsParameter, &pointCloud.points(), &pointCloud.normals());
  normalsParameter.setShow(false);
  GLParameterCovariances covariancesParameter(1.0f, Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f), Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f), curvatureThreshold, 0.02f);
  DrawableCovariances covariancesDrawable(Eigen::Isometry3f::Identity(), (GLParameter*)&covariancesParameter, &pointCloud.stats());
  covariancesParameter.setShow(false);
  GLParameterUncertainty uncertaintyParameter(1.0f, Eigen::Vector4f(1.0f, 0.25f, 0.0f, 1.0f), 1.0f);
  DrawableUncertainty uncertaintyDrawable(Eigen::Isometry3f::Identity(), (GLParameter*)&uncertaintyParameter, &pointCloud.gaussians());
  uncertaintyParameter.setShow(false);
  nicp_qglviewer->addDrawable(&pointsDrawable);
  nicp_qglviewer->addDrawable(&normalsDrawable);
  nicp_qglviewer->addDrawable(&covariancesDrawable);
  nicp_qglviewer->addDrawable(&uncertaintyDrawable);
  int snapshotCounter = 0;
  while(mainWindow->isVisible()) {
    bool update = false;

    // Check step
    if(!stepCheckBox->isChecked()) {
      pointsCheckBox->setChecked(false);
      normalsCheckBox->setChecked(false);
      covariancesCheckBox->setChecked(false);      
      uncertaintyCheckBox->setChecked(false);
      pointsParameter.setShow(false);
      normalsParameter.setShow(false);
      covariancesParameter.setShow(false);
      uncertaintyParameter.setShow(false);
      update = true;
    }
    if((int)stepSpinBox->value() != (int)pointsParameter.step()) {
      pointsDrawable.setStep(stepSpinBox->value());
      normalsDrawable.setStep(stepSpinBox->value());
      covariancesDrawable.setStep(stepSpinBox->value());
      uncertaintyDrawable.setStep(stepSpinBox->value());
      update = true;
    }

    // Points
    if((float)pointsSpinBox->value() != (float)pointsParameter.pointSize()) {
      pointsDrawable.setPointSize(pointsSpinBox->value());
      update = true;
    }
    if(pointsCheckBox->isChecked() != pointsParameter.show()) {
      pointsParameter.setShow(pointsCheckBox->isChecked());
      update = true;
    }

    // Normals
    if((float)normalsSpinBox->value() != (float)normalsParameter.normalLength()) {
      normalsDrawable.setNormalLength(normalsSpinBox->value());
      update = true;
    }
    if(normalsCheckBox->isChecked() != normalsParameter.show()) {
      normalsParameter.setShow(normalsCheckBox->isChecked());
      update = true;
    }

    // Covariances
    if((float)covariancesSpinBox->value() != (float)covariancesParameter.ellipsoidScale()) {
      covariancesDrawable.setEllipsoidScale(covariancesSpinBox->value());
      update = true;
    }
    if(covariancesCheckBox->isChecked() != covariancesParameter.show()) {
      covariancesParameter.setShow(covariancesCheckBox->isChecked());
      update = true;
    }
    
    // Uncertainty
    if((float)uncertaintySpinBox->value() != (float)uncertaintyParameter.ellipsoidScale()) {
      uncertaintyDrawable.setEllipsoidScale(uncertaintySpinBox->value());
      update = true;
    }
    if(uncertaintyCheckBox->isChecked() != uncertaintyParameter.show()) {
      uncertaintyParameter.setShow(uncertaintyCheckBox->isChecked());
      update = true;
    }

    // Snapshot
    if(snapshotButton->isDown()) {
      char snapshotName[1024];
      sprintf(snapshotName, "snapshot-%02d.png", snapshotCounter++);
      nicp_qglviewer->setSnapshotFormat(QString("PNG"));
      nicp_qglviewer->setSnapshotQuality(100);
      nicp_qglviewer->saveSnapshot(QString(snapshotName), true); 
      usleep(100000);
    }
    
    if(update)
      nicp_qglviewer->updateGL();    
    application.processEvents();
  }

  return 0;
}
