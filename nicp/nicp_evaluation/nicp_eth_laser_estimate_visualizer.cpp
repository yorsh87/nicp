#include <iostream>
#include <fstream>
#include <string> 
#include <sys/time.h>

#include <GL/gl.h>
#include <GL/glut.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/highgui/highgui.hpp>

#include <QApplication>
#include <QMainWindow>
#include <QHBoxLayout>

#include "nicp/bm_se3.h"
#include "nicp/imageutils.h"
#include "nicp/sphericalpointprojector.h"
#include "nicp/depthimageconverterintegralimage.h"
#include "nicp/statscalculatorintegralimage.h"

#include <nicp_viewer/nicp_qglviewer.h>
#include <nicp_viewer/drawable_points.h>

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace nicp;
using namespace nicp_viewer;

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

class GLUWrapper {
public:
  static GLUquadricObj* getQuadradic() {
    static GLUWrapper inst;
    return inst._quadratic;
  }
protected:
  GLUWrapper() {
    _quadratic = gluNewQuadric();
    gluQuadricNormals(_quadratic, GLU_SMOOTH);
  }
  ~GLUWrapper() {
    gluDeleteQuadric(_quadratic);
  }

  GLUquadricObj *_quadratic;;
};

class NICPTrackerAppViewer: public NICPQGLViewer {
public:
  NICPTrackerAppViewer(QWidget *parent = 0,
		      const string& algorithm_ = "",
		      const QGLWidget *shareWidget = 0, 
		      Qt::WFlags flags = 0): NICPQGLViewer(parent, shareWidget, flags) {
    _spin = _spinOnce = false;
    _needRedraw = true;
    _seq = 0;
    _algorithm = algorithm_;
    _currentTransform = Eigen::Isometry3f::Identity();
    _currentCloud = 0;
    _drawableCurrentCloud = 0;
    _drawableReferenceScene = 0;
  }
  ~NICPTrackerAppViewer() {}

  virtual void keyPressEvent(QKeyEvent* e) {
    NICPQGLViewer::keyPressEvent(e);
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
    NICPQGLViewer::init();
    setBackgroundColor(QColor::fromRgb(100, 100, 100));
    setMouseBinding(Qt::ControlModifier, Qt::LeftButton, RAP_FROM_PIXEL);
    qglviewer::Camera* oldcam = camera();
    qglviewer::Camera* cam = new StandardCamera();
    setCamera(cam);
    cam->setPosition(qglviewer::Vec(0.0f, 0.0f, 0.0f));
    cam->setUpVector(qglviewer::Vec(0.0f, -1.0f, 0.0f));
    cam->lookAt(qglviewer::Vec(0.0f, 0.0f, 1.0f));
    delete oldcam;
  }
 
  void drawSphere(GLfloat radius) {
    glNormal3f(0.0f, 0.0f, -1.0f); 
    gluSphere(GLUWrapper::getQuadradic(), radius, 32, 32);
  }

  void drawPyramid(GLfloat pyrH, GLfloat pyrW) {
    glBegin(GL_TRIANGLES);
    glNormal3f(0.0f, 0.0f, -1.0f); 
    glVertex3f(pyrW, pyrW, pyrH);
    glVertex3f(pyrW, -pyrW, pyrH);
    glVertex3f(-pyrW, -pyrW, pyrH);
    glVertex3f(-pyrW, -pyrW, pyrH);
    glVertex3f(-pyrW, pyrW, pyrH);
    glVertex3f(pyrW, pyrW, pyrH);
    glEnd(); 
    
    glBegin(GL_TRIANGLES);
    glNormal3f(0.0f, 0.0f, -1.0f); 
    glVertex3f(pyrW, -pyrW, pyrH);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(pyrW, pyrW, pyrH);
    glVertex3f(pyrW, pyrW, pyrH);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(-pyrW, pyrW, pyrH);
    glVertex3f(-pyrW, pyrW, pyrH);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(-pyrW, -pyrW, pyrH);
    glVertex3f(-pyrW, -pyrW, pyrH);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(pyrW, -pyrW, pyrH);
    glEnd(); 
  }

  void drawPyramidWireframe(float pyrH, float pyrW) {
    glLineWidth(3.0f);
    glBegin(GL_LINE_LOOP);
    glNormal3f(0.0f, 0.0f, -1.0f); 
    glVertex3f(pyrW, -pyrW, pyrH);
    glVertex3f(pyrW, pyrW, pyrH);
    glVertex3f(-pyrW, pyrW, pyrH);
    glVertex3f(-pyrW, -pyrW, pyrH);
    glEnd();

    glBegin(GL_LINES);
    glNormal3f(0.0f, 0.0f, -1.0f); 
    glVertex3f(pyrW, -pyrW, pyrH);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(pyrW, pyrW, pyrH);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(-pyrW, pyrW, pyrH);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(-pyrW, -pyrW, pyrH);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glEnd();
  }

  virtual void draw() { 
    NICPQGLViewer::draw(); 

    glColor4f(0.0, 0.0f, 1.0f, 1.0f);
    drawAxis(0.2f);

    float radius = 0.02f; 
    // Draw current estimated camera frame
    if(_estimatedPoses.size() > 0) {
      glPushMatrix();
      glColor4f(1.0, 0.0f, 0.0f, 1.0f);
      glMultMatrixf(_estimatedPoses[_estimatedPoses.size() - 1].data());
      float pyrH = 0.1f;
      float pyrW = 0.05f;
      drawPyramidWireframe(pyrH, pyrW);
      glPopMatrix();
    }

    // Draw current groundtruth camera frame
    if(_groundtruthPoses.size() > 0) {
      glPushMatrix();
      glColor4f(0.0, 1.0f, 0.0f, 1.0f);
      glMultMatrixf(_groundtruthPoses[_groundtruthPoses.size() - 1].data());
      float pyrH = 0.1f;
      float pyrW = 0.05f;
      drawPyramidWireframe(pyrH, pyrW);
      glPopMatrix();
    }

    // Draw estimated trajectory
    for(size_t i = 0; i < _estimatedPoses.size(); ++i) {
      glPushMatrix();
      glColor4f(1.0, 0.0f, 0.0f, 1.0f);
      glMultMatrixf(_estimatedPoses[i].data());
      drawSphere(radius);
      glPopMatrix();
    }

    // Draw groundtruth trajectory
    for(size_t i = 0; i < _groundtruthPoses.size(); ++i) {
      glPushMatrix();
      glColor4f(0.0, 1.0f, 0.0f, 1.0f);
      glMultMatrixf(_groundtruthPoses[i].data());
      drawSphere(radius);
      glPopMatrix();
    }    
    
    if(_algorithm != "" && _spin) {
      char buffer[1024];
      sprintf(buffer, "%s_%05d.png", _algorithm.c_str(), _seq);
      this->setSnapshotFormat(QString("PNG"));
      this->setSnapshotQuality(10);
      this->saveSnapshot(QString(buffer), true);
      _seq++;
    }

    _needRedraw = false;
  }

  void updateReferenceScene(Cloud* referenceScene_, Eigen::Isometry3f transform_) {
    if(!_drawableReferenceScene) { 
      _drawableReferenceScene = new DrawablePoints(transform_, 
						   new GLParameterPoints(1.0f, Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f)), 
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
      delete _currentCloud;
      _currentCloud = 0;
      _drawableCurrentCloud->setTransformation(transform_);      
      DrawablePoints* dp = dynamic_cast<DrawablePoints*>(_drawableCurrentCloud);
      if(dp) { 
	dp->setPointsAndNormals(&currentCloud_->points(), &currentCloud_->normals());
	dp->updatePointDrawList(); 
      }
    }
    _currentCloud = currentCloud_;
    _estimatedPoses.push_back(transform_);
    _currentTransform = transform_;
    
    _needRedraw = true;
  }

  void resetReferenceScene() { 
    DrawablePoints* dp = dynamic_cast<DrawablePoints*>(_drawableReferenceScene);    
    GLParameterPoints* pp = dynamic_cast<GLParameterPoints*>(_drawableReferenceScene->parameter());    
    if(dp && pp) {
      pp->setColor(Eigen::Vector4f(0.8f, 0.5f, 0.5f, 1.0f));
      dp->updatePointDrawList(); 
    }
    _drawableReferenceScene = 0;
    _needRedraw = true; 
  }  

  void addGroundtruthPose(const Eigen::Isometry3f pose) { 
    _groundtruthPoses.push_back(pose);
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
  int _seq;
  string _algorithm;
  Eigen::Isometry3f _currentTransform;
  std::vector<Eigen::Isometry3f> _groundtruthPoses;
  std::vector<Eigen::Isometry3f> _estimatedPoses;
  Cloud* _currentCloud;
  Drawable* _drawableReferenceScene;
  Drawable* _drawableCurrentCloud;
};

class NICPTrackerApp {
public:
  NICPTrackerApp(const std::string& configurationFile, NICPTrackerAppViewer* viewer_) { 
    _viewer = viewer_;
    
    _seq = 0;
    _rows = 480; 
    _cols = 640;
    _imageScaling = 1;
    _depthScaling = 0.001f;
    _K << 
    525.0f,   0.0f, 319.5f,
      0.0f, 525.0f, 239.5f,
      0.0f,   0.0f,   1.0f;

    _offsetT.setIdentity();
    _offsetT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _previousEstimateT.setIdentity();
    _previousEstimateT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _sensorOffset.setIdentity();
    _sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    
    _currentCloud = new Cloud();
    _referenceScene = new Cloud();

    init(configurationFile); 
  }

  ~NICPTrackerApp() {}

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
    Vector3f sensorOffsetTranslation = Vector3f(0.0f, 0.0f, 0.0f);
    Quaternionf sensorOffsetRotation = Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
    if((it = inputParameters.find("sensorOffset")) != inputParameters.end()) {
      if((*it).second.size() == 7) {
	sensorOffsetTranslation.x() = ((*it).second)[0];
	sensorOffsetTranslation.y() = ((*it).second)[1];
	sensorOffsetTranslation.z() = ((*it).second)[2];
	sensorOffsetRotation.x() = ((*it).second)[3];
	sensorOffsetRotation.y() = ((*it).second)[4];
	sensorOffsetRotation.z() = ((*it).second)[5];
	sensorOffsetRotation.w() = ((*it).second)[6];
      }
      else { std::cerr << "[WARNING]: startingPose has bad formatting from file, expecting 7 values but " << (*it).second.size() << " where given... keeping default values" << std::endl; }
    }    
    sensorOffsetRotation.normalize();
    _sensorOffset.translation() = sensorOffsetTranslation;
    _sensorOffset.linear() = sensorOffsetRotation.toRotationMatrix();
    _sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    std::cout << "[INFO]: g  depth scaling " << _depthScaling << std::endl;
    std::cout << "[INFO]: g  image scaling " << _imageScaling << std::endl;
    std::cout << "[INFO]: g  sensor offset " << t2v(_sensorOffset).transpose() << std::endl;

    // Point projector
    if((it = inputParameters.find("horizontalFov")) != inputParameters.end()) _projector.setHorizontalFov(((*it).second)[0]);
    if((it = inputParameters.find("verticalFov")) != inputParameters.end()) _projector.setVerticalFov(((*it).second)[0]);
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
    _projector.setImageSize(_rows, _cols);
    _projector.scale(1.0f / (float)_imageScaling);    
    std::cout << "[INFO]: pp offset " << t2v(_projector.transform()).transpose() << std::endl;    
    std::cout << "[INFO]: pp horizontal fov " << _projector.horizontalFov() << std::endl;    
    std::cout << "[INFO]: pp vertical fov " << _projector.verticalFov() << std::endl;    
    std::cout << "[INFO]: pp minimum distance " << _projector.minDistance() << std::endl;    
    std::cout << "[INFO]: pp maximum distance " << _projector.maxDistance() << std::endl;
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

    // Depth image converter
    _converter = DepthImageConverterIntegralImage(&_projector, &_statsCalculator,
						  &_pointInformationMatrixCalculator,
						  &_normalInformationMatrixCalculator);
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
  
  void setStartingPose(const Eigen::Isometry3f startingPose) { _offsetT = startingPose; }

  void spinOnce(Eigen::Isometry3f& estimateT, const std::string& depthFilename) { 
    // Load new current cloud
    _rawDepth = imread(depthFilename, -1);
    if(_imageScaling > 1) {
      DepthImage_convert_16UC1_to_32FC1(_depth, _rawDepth, _depthScaling);
      DepthImage_scale(_scaledDepth, _depth, _imageScaling);
    }
    else { DepthImage_convert_16UC1_to_32FC1(_scaledDepth, _rawDepth, _depthScaling); }
    _scaledIndeces.create(_scaledDepth.rows, _scaledDepth.cols);
    _currentCloud = new Cloud();
    _converter.compute(*_currentCloud, _scaledDepth, _sensorOffset);
            
    Eigen::Isometry3f deltaT = _previousEstimateT.inverse() * estimateT;
    std::cout << "[INFO]: delta  T " << t2v(deltaT).transpose() << std::endl;    
    _referenceScene->transformInPlace(deltaT.inverse());
    _referenceScene->add(*_currentCloud);

    _seq++;
    _previousEstimateT = estimateT;

    // Update visualization structures
    _viewer->updateCurrentCloud(_currentCloud, estimateT); 
    _viewer->updateReferenceScene(_referenceScene, estimateT);
    _viewer->resetReferenceScene(); 
  }

protected:
  int _seq;
  int _rows, _cols;
  int _imageScaling;
  float _depthScaling;
  Matrix3f _K;

  Eigen::Isometry3f _offsetT, _previousEstimateT, _sensorOffset;

  RawDepthImage _rawDepth;
  DepthImage _depth, _scaledDepth, _referenceScaledDepth;
  IndexImage _scaledIndeces, _referenceScaledIndeces;
  Cloud* _referenceScene;
  Cloud* _currentCloud;

  SphericalPointProjector _projector;
  StatsCalculatorIntegralImage _statsCalculator;  
  PointInformationMatrixCalculator _pointInformationMatrixCalculator;
  NormalInformationMatrixCalculator _normalInformationMatrixCalculator;  
  DepthImageConverterIntegralImage _converter;
  
  NICPTrackerAppViewer* _viewer;
};

Eigen::Isometry3f getT(std::string TFilename, std::string depthTimestamp) {
  ifstream is(TFilename.c_str());
  if(!is) {
    std::cerr << "[ERROR]: impossible to open file " << TFilename << std::endl;
    exit(-1);
  }
  
  Eigen::Quaternionf q, bestQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
  Eigen::Vector3f t, bestT(0.0f, 0.0f, 0.0f);  
  double dDepthTimestamp = 0;
  std::istringstream i(depthTimestamp);
  i >> dDepthTimestamp;
  double tMin = std::numeric_limits<double>::max();
  while(is.good()) {
    char buf[4096];
    is.getline(buf, 4096);
    std::istringstream iss(buf);
    std::string timestamp;
    if(!(iss >> timestamp)) { continue; }
    if(timestamp[0] == '#' || timestamp[0] == '/') { continue; }
    if(!(iss >> t.x() >> t.y() >> t.z() >> q.x() >> q.y() >> q.z() >> q.w())) { continue; }
    std::istringstream ii(timestamp);
    double dTimestamp;
    ii >> dTimestamp;
    if(fabs(dTimestamp - dDepthTimestamp) < tMin) {
      bestQuaternion = q;
      bestT = t;
      tMin = fabs(dTimestamp - dDepthTimestamp);
    }
  }  
  Eigen::Isometry3f T;
  bestQuaternion.normalize();
  T.linear() = bestQuaternion.toRotationMatrix();
  T.translation() = bestT;
  T.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  return T;
}

int main(int argc, char** argv) {
  /*********************************************************************************
   *                               INPUT HANDLING                                  *
   *********************************************************************************/
  if(argc < 7) {
    std::cout << "Usage: nicp_eth_laser_estimate_visualizer <configuration.txt> <associations.txt> <odometry.txt> <groundtruth.txt> <algorithm> <save_snapshots>" << std::endl
	      << "\tconfiguration.txt\t-->\tinput text configuration filename" << std::endl
	      << "\tassociations.txt\t-->\tfiel containing a set of depth images associations for alignment in the format: " << std::endl
	      << "\t\t\t\t\ttimestampDepthImage depthImageFilename" << std::endl
	      << "\todometry.txt\t\t-->\toutput text filename containing the computed visual odometry" << std::endl
	      << "\tground_truth.txt\t\t-->\tgroundtruth trajectory file" << std::endl    
	      << "\talgorithm\t\t-->\talgorithm used to compute the estimate. Options are:" << std::endl    
	      << "\t\t\t\t\tgicp\tndt\tnicp" << std::endl
	      << "\tsave_snapshots\t\t-->\t 1 if you want to save snapshots, 0 otherwise" << std::endl;    
    return 0;
  }  
  std::string configurationFile = std::string(argv[1]);
  std::string associationsFile = std::string(argv[2]);
  std::string odometryFile = std::string(argv[3]);
  std::string groundtruthFile = std::string(argv[4]);
  std::string algorithm = std::string(argv[5]);
  bool save_snapshots = atoi(argv[6]);
  std::cout << "[INFO]: configuration file " << configurationFile << std::endl;
  std::cout << "[INFO]: associations file " << associationsFile << std::endl;
  std::cout << "[INFO]: odometry file " << odometryFile << std::endl;
  std::cout << "[INFO]: groundtruth file " << groundtruthFile << std::endl;
  
  ifstream is_ass(associationsFile.c_str());
  if(!is_ass) {
    std::cerr << "[ERROR]: impossible to open depth images associations file " << associationsFile << std::endl;
    return -1;
  }

  /*********************************************************************************
   *                            INIT GUI AND TRACKER                               *
   *********************************************************************************/
  QApplication application(argc, argv);
  QWidget* mainWindow = new QWidget();
  mainWindow->setWindowTitle("Estimate Visualizer");
  QHBoxLayout* viewerLayout = new QHBoxLayout();
  mainWindow->setLayout(viewerLayout);
  NICPTrackerAppViewer* viewer;
  if(save_snapshots) { viewer = new NICPTrackerAppViewer(mainWindow, algorithm); }
  else { viewer = new NICPTrackerAppViewer(mainWindow); }
  viewerLayout->addWidget(viewer);
  NICPTrackerApp* tracker;
  viewer->init();
  mainWindow->show();
  viewer->show();
  tracker = new NICPTrackerApp(configurationFile, viewer); 
  
  /*********************************************************************************
   *                                 MANAGE GUI                                    *
   *********************************************************************************/
  int counter = 0;
  Eigen::Isometry3f offsetT;
  while(mainWindow->isVisible()) {        
    application.processEvents();
    if(viewer->needRedraw()) { viewer->updateGL(); }
    else { usleep(10000); }        
    while(is_ass.good() && (viewer->spinOnce() || viewer->spin())) {
      char buf[4096];
      is_ass.getline(buf, 4096);
      istringstream iss(buf);
      string timestamp, depthFilename;
      if(!(iss >> timestamp >> depthFilename)) { continue; }
      if(timestamp[0] == '#') { continue; }
      std::cout << "---------------------------------------------------------------------------- " << std::endl;
      std::cout << "[INFO]: new frame " << depthFilename << std::endl;
      Eigen::Isometry3f groundtruthT = Eigen::Isometry3f::Identity();
      groundtruthT = getT(groundtruthFile, timestamp);  
      Eigen::Isometry3f estimateT = Eigen::Isometry3f::Identity();
      estimateT = getT(odometryFile, timestamp);  
      if(counter == 0) { offsetT = groundtruthT; }  
      viewer->addGroundtruthPose(offsetT.inverse() * groundtruthT);
      tracker->spinOnce(estimateT, depthFilename);      
      std::cout << "[INFO]: estimate T " << t2v(estimateT).transpose() << std::endl;
      std::cout << "[INFO]: ground   T " << t2v(groundtruthT).transpose() << std::endl;
      counter++;
      application.processEvents();
      if(viewer->needRedraw()) { viewer->updateGL(); }
    }    
  }

  return 0;
}
