#include <iostream>
#include <fstream>
#include <sys/time.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/highgui/highgui.hpp>

#include <QApplication>
#include <QMainWindow>
#include <QHBoxLayout>

#include "pwn/bm_se3.h"
#include "pwn/imageutils.h"
#include "pwn/pinholepointprojector.h"
#include "pwn/depthimageconverterintegralimage.h"
#include "pwn/statscalculatorintegralimage.h"
#include "pwn/aligner.h"
#include "pwn/merger.h"

#include <pwn_viewer/pwn_qglviewer.h>
#include <pwn_viewer/drawable_points.h>

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace pwn;
using namespace pwn_viewer;

class StandardCamera: public qglviewer::Camera {
public:
  StandardCamera(): _standard(true) {}
  
  float zNear() const {
    if(_standard) { return 0.001f; } 
    else { return Camera::zNear(); } 
  }

  float zFar() const {  
    if(_standard) { return 10000.0f; } 
    else { return Camera::zFar(); }
  }

  bool standard() const { return _standard; }  
  void setStandard(bool s) { _standard = s; }

protected:
  bool _standard;
};


class PWNTrackerAppViewer: public PWNQGLViewer {
public:
  PWNTrackerAppViewer(QWidget *parent = 0, 
		      const QGLWidget *shareWidget = 0, 
		      Qt::WFlags flags = 0): PWNQGLViewer(parent, shareWidget, flags) {
    _spin = _spinOnce = false;
    _needRedraw = true;
    _currentCloud = 0;
    _referenceScene = 0;
    _drawableCurrentCloud = 0;
    _drawableReferenceScene = 0;
  }
  ~PWNTrackerAppViewer() {}

  virtual void keyPressEvent(QKeyEvent* e) {
    PWNQGLViewer::keyPressEvent(e);
    // Start tracking
    if((e->key() == Qt::Key_T)) { 
      std::cout << "[INFO]: starting tracking" << std::endl;
      _spin = true; 
    }
    // Align next depth image
    else if((e->key() == Qt::Key_N)) { 
      std::cout << "[INFO]: aligning next depth image" << std::endl;
      _spinOnce = true; 
    }
    // Stop tracking
    else if((e->key() == Qt::Key_P)) { 
      std::cout << "[INFO]: stop tracking" << std::endl;
      _spin = _spinOnce = false; 
    }
    else {}
  }

  virtual void init() {
    PWNQGLViewer::init();
    setBackgroundColor(QColor::fromRgb(255, 255, 255));
    setMouseBinding(Qt::ControlModifier, Qt::LeftButton, RAP_FROM_PIXEL);
    qglviewer::Camera* oldcam = camera();
    qglviewer::Camera* cam = new StandardCamera();
    setCamera(cam);
    cam->setPosition(qglviewer::Vec(0.0f, 0.0f, 0.0f));
    cam->setUpVector(qglviewer::Vec(0.0f, -1.0f, 0.0f));
    cam->lookAt(qglviewer::Vec(0.0f, 0.0f, 1.0f));
    setAxisIsDrawn(true);
    delete oldcam;
  }
 
  virtual void draw() { 
    PWNQGLViewer::draw(); 
    _needRedraw = false;
  }

  void updateReferenceScene(Cloud* referenceScene_, Eigen::Isometry3f transform_) {
    if(!_drawableReferenceScene) { 
      _drawableReferenceScene = new DrawablePoints(transform_, 
						   new GLParameterPoints(1.0f, Eigen::Vector4f(0.5f, 0.5f, 0.5f, 1.0f)), 
						   &referenceScene_->points(), &referenceScene_->normals());      
      addDrawable(_drawableReferenceScene);
    }
    else { 
      _drawableReferenceScene->setTransformation(transform_);
      DrawablePoints* dp = dynamic_cast<DrawablePoints*>(_drawableReferenceScene);
      if(dp) { dp->updatePointDrawList(); }
    }
    _needRedraw = true;
  }

  void updateCurrentCloud(Cloud* currentCloud_, Eigen::Isometry3f transform_) {
    if(!_drawableCurrentCloud) {
      _drawableCurrentCloud = new DrawablePoints(transform_, 
						 new GLParameterPoints(2.0f, Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f)), 
						 &currentCloud_->points(), &currentCloud_->normals());      
      addDrawable(_drawableCurrentCloud);
    }
    else { 
      _drawableCurrentCloud->setTransformation(transform_);      
      DrawablePoints* dp = dynamic_cast<DrawablePoints*>(_drawableCurrentCloud);
      if(dp) { 
	dp->setPointsAndNormals(&currentCloud_->points(), &currentCloud_->normals());
	dp->updatePointDrawList(); 
      }
    }
    _needRedraw = true;
  }

  void resetReferenceScene(Cloud* referenceScene_) {
    _referenceScene = 0;
    _needRedraw = true;
  }  

  void addCloud(Cloud* cloud_, Eigen::Isometry3f transform_) {    
    size_t size = _drawableList.size();
    if(size > 0) {
      GLParameterPoints* pp = dynamic_cast<GLParameterPoints*>(_drawableList[size - 1]->parameter());
      if(pp) { 
	pp->setColor(Eigen::Vector4f(0.5f, 0.5f, 0.5f, 1.0f)); 
	pp->setPointSize(1.0f);
      }
      DrawablePoints* dp = dynamic_cast<DrawablePoints*>(_drawableList[size - 1]);
      if(dp) { dp->updatePointDrawList(); }
    }
    Drawable* d = new DrawablePoints(transform_, 
				     new GLParameterPoints(2.0f, Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f)), 
				     &cloud_->points(), &cloud_->normals());
    addDrawable(d);
    _needRedraw = true;
  }  

  inline bool needRedraw() const { return _needRedraw; }
  inline bool spin() const { return _spin; }
  inline bool spinOnce() { 
    bool ret = _spinOnce;
    _spinOnce = false; 
    return ret; 
  }

protected:  
  bool _spin;
  bool _spinOnce;
  bool _needRedraw;
  Cloud* _referenceScene;
  Cloud* _currentCloud;
  Drawable* _drawableReferenceScene;
  Drawable* _drawableCurrentCloud;
};

class PWNTrackerApp {
public:
  PWNTrackerApp(const std::string& configurationFile, PWNTrackerAppViewer* viewer_ = 0) { 
    _viewer = viewer_;
    if(_viewer) { std::cout << "[INFO] g  enabled viewer " << _viewer << std::endl; }
    else { std::cout << "[INFO]: g  disabled viewer " << _viewer << std::endl; }

    _seq = 0;
    _rows = 480; 
    _cols = 640;
    _imageScaling = 1;
    _depthScaling = 0.001f;
    _breakingAngle = M_PI / 2.0f;
    _breakingDistance = 1.0f; 
    _breakingInlierRatio = 0.75f; 
    _K << 
    525.0f,   0.0f, 319.5f,
      0.0f, 525.0f, 239.5f,
      0.0f,   0.0f,   1.0f;

    _deltaT.setIdentity();
    _deltaT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _globalT.setIdentity();
    _globalT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _localT.setIdentity();
    _localT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

    _currentCloud = new Cloud();
    _referenceCloud = new Cloud();
    _referenceScene = new Cloud();

    init(configurationFile); 
  }
  ~PWNTrackerApp() {}

  void init(const std::string& configurationFile) {
    // Fill input parameter map
    map<string, std::vector<float> > inputParameters;
    bool fillInputParameters =  fillInputParametersMap(inputParameters, configurationFile);
    if(!fillInputParameters) {
      std::cerr << "[ERROR]: error occurred reading input parameters... quitting" << std::endl;
      exit(-1);
    }
    
    setInputParameters(inputParameters);
  }

  void setInputParameters(map<string, std::vector<float> >& inputParameters) {
    map<string, std::vector<float> >::iterator it;

    // General
    if((it = inputParameters.find("depthScaling")) != inputParameters.end()) _depthScaling = ((*it).second)[0];
    if((it = inputParameters.find("imageScaling")) != inputParameters.end()) _imageScaling = ((*it).second)[0];
    Vector3f initialTranslation = Vector3f(0.0f, 0.0f, 0.0f);
    Quaternionf initialRotation = Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
    if((it = inputParameters.find("startingPose")) != inputParameters.end()) {
      if((*it).second.size() == 7) {
	initialTranslation.x() = ((*it).second)[0];
	initialTranslation.y() = ((*it).second)[1];
	initialTranslation.z() = ((*it).second)[2];
	initialRotation.x() = ((*it).second)[3];
	initialRotation.y() = ((*it).second)[4];
	initialRotation.z() = ((*it).second)[5];
	initialRotation.w() = ((*it).second)[6];
      }
      else { std::cerr << "[WARNING]: startingPose has bad formatting from file, expecting 7 values but " << (*it).second.size() << " where given... keeping default values" << std::endl; }
    }
    initialRotation.normalize();
    _globalT.translation() = initialTranslation;
    _globalT.linear() = initialRotation.toRotationMatrix();
    _globalT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    if((it = inputParameters.find("breakingAngle")) != inputParameters.end()) _breakingAngle = ((*it).second)[0];
    if((it = inputParameters.find("breakingDistance")) != inputParameters.end()) _breakingDistance = ((*it).second)[0];
    if((it = inputParameters.find("breakingInlierRatio")) != inputParameters.end()) _breakingInlierRatio = ((*it).second)[0];
    std::cout << "[INFO]: g  depth scaling " << _depthScaling << std::endl;
    std::cout << "[INFO]: g  image scaling " << _imageScaling << std::endl;
    std::cout << "[INFO]: g  starting pose " << t2v(_globalT).transpose() << std::endl;
    std::cout << "[INFO]: g  breaking angle " << _breakingAngle << std::endl;
    std::cout << "[INFO]: g  breaking distance " << _breakingDistance << std::endl;
    std::cout << "[INFO]: g  breaking inlier ratio " << _breakingInlierRatio << std::endl;

    // Point projector
    if((it = inputParameters.find("K")) != inputParameters.end()) {
      if((*it).second.size() == 4) {
	_K(0, 0) = ((*it).second)[0];
	_K(1, 1) = ((*it).second)[1];
	_K(0, 2) = ((*it).second)[2];
	_K(1, 2) = ((*it).second)[3];
      }
      else { std::cerr << "[WARNING]: K has bad formatting from file, expecting 4 values but " << (*it).second.size() << " where given... keeping default values" << std::endl; }
    }
    if((it = inputParameters.find("minDistance")) != inputParameters.end()) _projector.setMinDistance(((*it).second)[0]);
    if((it = inputParameters.find("maxDistance")) != inputParameters.end()) _projector.setMaxDistance(((*it).second)[0]);
    if((it = inputParameters.find("imageSize")) != inputParameters.end()) {
      if((*it).second.size() == 2) {
	_rows = ((*it).second)[0];
	_cols = ((*it).second)[1];
      }
      else { std::cerr << "[WARNING]: imageSize has bad formatting from file, expecting 2 values but " << (*it).second.size() << " where given... keeping default values" << std::endl; }
    }
    _projector.setTransform(Eigen::Isometry3f::Identity());
    _projector.setCameraMatrix(_K);
    _projector.setImageSize(_rows, _cols);
    _projector.scale(1.0f / (float)_imageScaling);    
    std::cout << "[INFO]: pp offset " << t2v(_projector.transform()).transpose() << std::endl;    
    std::cout << "[INFO]: pp minimum distance " << _projector.minDistance() << std::endl;    
    std::cout << "[INFO]: pp maximum distance " << _projector.maxDistance() << std::endl;
    std::cout << "[INFO]: pp K " << std::endl << _projector.cameraMatrix() << std::endl;
    std::cout << "[INFO]: pp image size " << _projector.imageRows() << " --- " << _projector.imageCols() << std::endl;

    // Stats calculator and information matrix calculators
    if((it = inputParameters.find("minImageRadius")) != inputParameters.end()) _statsCalculator.setMinImageRadius(((*it).second)[0]);
    if((it = inputParameters.find("maxImageRadius")) != inputParameters.end()) _statsCalculator.setMaxImageRadius(((*it).second)[0]);
    if((it = inputParameters.find("minPoints")) != inputParameters.end()) _statsCalculator.setMinPoints(((*it).second)[0]);
    if((it = inputParameters.find("statsCalculatorCurvatureThreshold")) != inputParameters.end()) _statsCalculator.setCurvatureThreshold(((*it).second)[0]);
    if((it = inputParameters.find("worldRadius")) != inputParameters.end()) _statsCalculator.setWorldRadius(((*it).second)[0]);    
    if((it = inputParameters.find("informationMatrixCurvatureThreshold")) != inputParameters.end()) {
      _pointInformationMatrixCalculator.setCurvatureThreshold(((*it).second)[0]);
      _normalInformationMatrixCalculator.setCurvatureThreshold(((*it).second)[0]);
    }
    std::cout << "[INFO]: sc minimum image radius " << _statsCalculator.minImageRadius() << std::endl;
    std::cout << "[INFO]: sc maximum image radius " << _statsCalculator.maxImageRadius() << std::endl;
    std::cout << "[INFO]: sc minimum points " << _statsCalculator.minPoints() << std::endl;
    std::cout << "[INFO]: sc curvature threshold " << _statsCalculator.curvatureThreshold() << std::endl;
    std::cout << "[INFO]: sc world radius " << _statsCalculator.worldRadius() << std::endl;
    std::cout << "[INFO]: ic curvature threshold " << _normalInformationMatrixCalculator.curvatureThreshold() << std::endl;

    // Correspondence finder
    if((it = inputParameters.find("inlierDistanceThreshold")) != inputParameters.end()) _correspondenceFinder.setInlierDistanceThreshold(((*it).second)[0]);
    if((it = inputParameters.find("inlierNormalAngularThreshold")) != inputParameters.end()) _correspondenceFinder.setInlierNormalAngularThreshold(((*it).second)[0]);
    if((it = inputParameters.find("inlierCurvatureRatioThreshold")) != inputParameters.end()) _correspondenceFinder.setInlierCurvatureRatioThreshold(((*it).second)[0]);
    if((it = inputParameters.find("correspondenceFinderCurvatureThreshold")) != inputParameters.end()) _correspondenceFinder.setFlatCurvatureThreshold(((*it).second)[0]);
    _correspondenceFinder.setImageSize(_projector.imageRows(), _projector.imageCols());
    std::cout << "[INFO]: cf inlier distance threshold " << _correspondenceFinder.inlierDistanceThreshold() << std::endl;
    std::cout << "[INFO]: cf inlier normal angular threshold " << _correspondenceFinder.inlierNormalAngularThreshold() << std::endl;
    std::cout << "[INFO]: cf inlier curvature ratio threshold " << _correspondenceFinder.inlierCurvatureRatioThreshold() << std::endl;
    std::cout << "[INFO]: cf curvature threshold " << _correspondenceFinder.flatCurvatureThreshold() << std::endl;
    std::cout << "[INFO]: cf image size " << _correspondenceFinder.imageRows() << " --- " << _correspondenceFinder.imageCols() << std::endl;

    // Linearizer
    if((it = inputParameters.find("inlierMaxChi2")) != inputParameters.end()) _linearizer.setInlierMaxChi2(((*it).second)[0]);
    if((it = inputParameters.find("robustKernel")) != inputParameters.end()) _linearizer.setRobustKernel(((*it).second)[0]);    
    _linearizer.setAligner(&_aligner);
    std::cout << "[INFO]: l  inlier maximum chi2 " << _linearizer.inlierMaxChi2() << std::endl;
    if(_linearizer.robustKernel()) { std::cout << "[INFO]: l  robust kernel enabled" << std::endl; }
    else { std::cout << "[INFO]: l  robust kernel disabled" << std::endl; }
    std::cout << "[INFO]: l  aligner " << _linearizer.aligner() << std::endl;

    // Aligner
    if((it = inputParameters.find("outerIterations")) != inputParameters.end()) _aligner.setOuterIterations(((*it).second)[0]);    
    if((it = inputParameters.find("innerIterations")) != inputParameters.end()) _aligner.setInnerIterations(((*it).second)[0]);
    if((it = inputParameters.find("minInliers")) != inputParameters.end()) _aligner.setMinInliers(((*it).second)[0]);
    if((it = inputParameters.find("translationalMinEigenRatio")) != inputParameters.end()) _aligner.setTranslationalMinEigenRatio(((*it).second)[0]);
    if((it = inputParameters.find("rotationalMinEigenRatio")) != inputParameters.end()) _aligner.setRotationalMinEigenRatio(((*it).second)[0]);
    _aligner.setProjector(&_projector);
    _aligner.setCorrespondenceFinder(&_correspondenceFinder);
    _aligner.setLinearizer(&_linearizer);
    std::cout << "[INFO]: a  outer iterations " << _aligner.outerIterations() << std::endl;
    std::cout << "[INFO]: a  inner iterations " << _aligner.innerIterations() << std::endl;
    std::cout << "[INFO]: a  minimum inliers " << _aligner.minInliers() << std::endl;
    std::cout << "[INFO]: a  translation minimum eigen ratio " << _aligner.translationalMinEigenRatio() << std::endl;
    std::cout << "[INFO]: a  rotational minimum eigen ratio " << _aligner.rotationalMinEigenRatio() << std::endl;
    std::cout << "[INFO]: a  projector " << _aligner.projector() << std::endl;
    std::cout << "[INFO]: a  correspondence finder " << _aligner.correspondenceFinder() << std::endl;
    std::cout << "[INFO]: a  linearizer " << _aligner.linearizer() << std::endl;

    // Depth image converter
    _converter = DepthImageConverterIntegralImage(&_projector, &_statsCalculator,
						  &_pointInformationMatrixCalculator,
						  &_normalInformationMatrixCalculator);

    // Merger
    if((it = inputParameters.find("depthThreshold")) != inputParameters.end()) _merger.setMaxPointDepth(((*it).second)[0]);
    if((it = inputParameters.find("normalThreshold")) != inputParameters.end()) _merger.setNormalThreshold(((*it).second)[0]);
    if((it = inputParameters.find("distanceThreshold")) != inputParameters.end()) _merger.setDistanceThreshold(((*it).second)[0]);
    _merger.setDepthImageConverter(&_converter);
    std::cout << "[INFO]: m  depth threshold " << _merger.maxPointDepth() << std::endl;
    std::cout << "[INFO]: m  normal threshold " << _merger.normalThreshold() << std::endl;
    std::cout << "[INFO]: m  distance threshold " << _merger.distanceThreshold() << std::endl;
    std::cout << "[INFO]: m  converter " << _merger.depthImageConverter() << std::endl;
  }

  bool fillInputParametersMap(map<string, std::vector<float> >& inputParameters, 
			      const string& configurationFilename) {
    ifstream is(configurationFilename.c_str());
    if(!is) {
      std::cerr << "[ERROR] impossible to open configuration file " << configurationFilename << std::endl;
      return false;
    }

    while(is.good()) {
      // Get a line from the configuration file
      char buf[1024];
      is.getline(buf, 1024);
      istringstream iss(buf);
    
      // Add the parameter to the map
      string parameter; 
      if(!(iss >> parameter)) continue;
      if(parameter[0] == '#' || parameter[0] == '/') continue;
      float value;
      std::vector<float> values;
      while((iss >> value)) { values.push_back(value); }
      if(values.size() > 0) { inputParameters.insert(pair<string, std::vector<float> >(parameter, values)); }
    }
  
    return true;
  }

  inline double get_time() {
    struct timeval ts;
    gettimeofday(&ts, 0);
    return ts.tv_sec + ts.tv_usec * 1e-6;
  }

  Eigen::Isometry3f globalT() const { return _globalT; }

  double spinOnce(Eigen::Isometry3f& deltaT, const std::string& depthFilename) { 
    // Generate new current cloud
    _rawDepth = imread(depthFilename, -1);
    DepthImage_convert_16UC1_to_32FC1(_depth, _rawDepth, _depthScaling);
    DepthImage_scale(_scaledDepth, _depth, _imageScaling);
    // imwrite("current.png", _scaledDepth * 50);
    _scaledIndeces.create(_scaledDepth.rows, _scaledDepth.cols);
    _converter.compute(*_currentCloud, _scaledDepth);

    if(_viewer) { _viewer->updateReferenceScene(_referenceScene, _globalT); }
    
    // Align the new cloud
    _aligner.setInitialGuess(deltaT);
    _aligner.setReferenceCloud(_referenceScene);
    _aligner.setCurrentCloud(_currentCloud);
    _tBegin = get_time();
    _aligner.align();  
    _tEnd = get_time();
    _deltaT = _aligner.T();      
    _globalT = _globalT * _deltaT;
    deltaT = _deltaT;

    if(_viewer) { _viewer->updateCurrentCloud(_currentCloud, _globalT); }

    // Update structures    
    _referenceScene->add(*_currentCloud, _deltaT);
    _referenceScene->transformInPlace(_deltaT.inverse());
    _merger.merge(_referenceScene);

    // _converter.projector()->project(_scaledIndeces, _scaledDepth, _referenceScene->points());    
    // imwrite("reference.png", _scaledDepth * 50);
    // _converter.compute(*_referenceScene, _scaledDepth);
    
    _seq++;
    
    return _tEnd - _tBegin; 
  }

protected:
  double _tBegin, _tEnd; 
  
  int _seq;
  int _rows, _cols;
  int _imageScaling;
  float _depthScaling;
  float _breakingAngle, _breakingDistance, _breakingInlierRatio;   
  Matrix3f _K;

  Eigen::Isometry3f _deltaT, _globalT, _localT;

  RawDepthImage _rawDepth;
  DepthImage _depth, _scaledDepth;
  IndexImage _scaledIndeces;
  Cloud* _referenceScene;
  Cloud* _referenceCloud;
  Cloud* _currentCloud;

  PinholePointProjector _projector;
  StatsCalculatorIntegralImage _statsCalculator;  
  PointInformationMatrixCalculator _pointInformationMatrixCalculator;
  NormalInformationMatrixCalculator _normalInformationMatrixCalculator;  
  DepthImageConverterIntegralImage _converter;
  CorrespondenceFinder _correspondenceFinder;
  Linearizer _linearizer;
  Aligner _aligner;  
  Merger _merger;

  PWNTrackerAppViewer* _viewer;
};

int main(int argc, char** argv) {
  /*********************************************************************************
   *                               INPUT HANDLING                                  *
   *********************************************************************************/
  if(argc < 4) {
    std::cout << "Usage: pwn_icra_evaluator <configuration.txt> <associations.txt> <odometry.txt>" << std::endl
	      << "\tconfiguration.txt\t-->\tinput text configuration filename" << std::endl
	      << "\tassociations.txt\t-->\tfiel containing a set of depth images associations for alignment in the format: " << std::endl
	      << "\t\t\t\t\ttimestampDepthImage depthImageFilename" << std::endl
	      << "\todometry.txt\t\t-->\toutput text filename containing the computed visual odometry" << std::endl;
    return 0;
  }
  std::string configurationFile = std::string(argv[1]);
  std::string associationsFile = std::string(argv[2]);
  std::string odometryFile = std::string(argv[3]);
  std::cout << "[INFO]: configuration file " << configurationFile << std::endl;
  std::cout << "[INFO]: associations file " << associationsFile << std::endl;
  std::cout << "[INFO]: odometry file " << odometryFile << std::endl;

  ifstream is(associationsFile.c_str());
  if(!is) {
    std::cerr << "[ERROR]: impossible to open depth images associations file " << associationsFile << std::endl;
    return -1;
  }
  ofstream os(odometryFile.c_str());
  if(!os) {
    std::cerr << "[ERROR]: impossible to open odometry file " << odometryFile << std::endl;
    return -1;
  }

  /*********************************************************************************
   *                            INIT GUI AND TRACKER                               *
   *********************************************************************************/
  QApplication application(argc, argv);
  QWidget* mainWindow = new QWidget();
  mainWindow->setWindowTitle("nicp_tracker_app");
  QHBoxLayout* viewerLayout = new QHBoxLayout();
  mainWindow->setLayout(viewerLayout);
  PWNTrackerAppViewer* viewer = new PWNTrackerAppViewer(mainWindow);
  viewerLayout->addWidget(viewer);
  viewer->init();
  mainWindow->show();
  viewer->show();
  // mainWindow->showMaximized();
  PWNTrackerApp tracker(configurationFile, viewer); 

  /*********************************************************************************
   *                                 MANAGE GUI                                    *
   *********************************************************************************/
  Eigen::Isometry3f globalT;
  globalT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  while(mainWindow->isVisible()) {        
    application.processEvents();
    if(viewer->needRedraw()) { viewer->updateGL(); }
    else { usleep(10000); }        
    while(is.good() && (viewer->spinOnce() || viewer->spin())) {
      char buf[4096];
      is.getline(buf, 4096);
      istringstream iss(buf);
      string timestamp, depthFilename;
      if(!(iss >> timestamp >> depthFilename)) { continue; }
      if(timestamp[0] == '#') { continue; }
      std::cout << "---------------------------------------------------------------------------- " << std::endl;
      std::cout << "[INFO]: new frame " << depthFilename << std::endl;
      Eigen::Isometry3f deltaT = Eigen::Isometry3f::Identity();
      deltaT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      double time = tracker.spinOnce(deltaT, depthFilename);
      globalT = tracker.globalT();
      std::cout << "[INFO]: delta  T " << t2v(deltaT).transpose() << std::endl;
      std::cout << "[INFO]: global T " << t2v(globalT).transpose() << std::endl;
      std::cout << "[INFO]: computation time " << time << " ms" << std::endl;
      Quaternionf globalRotation = Quaternionf(globalT.linear());
      globalRotation.normalize();
      os << timestamp << " " 
	 << globalT.translation().x() << " "  << globalT.translation().y() << " " << globalT.translation().z() << " " 
	 << globalRotation.x() << " " << globalRotation.y() << " " << globalRotation.z() << " " << globalRotation.w() 
	 << std::endl;
      application.processEvents();
      if(viewer->needRedraw()) { viewer->updateGL(); }
    }
  }

  return 0;
}
