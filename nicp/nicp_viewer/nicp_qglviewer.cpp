#include <GL/gl.h>

#include "nicp_qglviewer.h"

#include "opengl_primitives.h"

using namespace Eigen;

namespace nicp_viewer {

  class StandardCamera : public qglviewer::Camera {
  public:
    StandardCamera() : _standard(true) {}
  
    float zNear() const {
      if(_standard) 
	return 0.001f; 
      else 
	return Camera::zNear(); 
    }

    float zFar() const {  
      if(_standard) 
	return 10000.0f; 
      else 
	return Camera::zFar();
    }

    bool standard() const { return _standard; }
  
    void setStandard(bool s) { _standard = s; }

  protected:
    bool _standard;
  };

  NICPQGLViewer::NICPQGLViewer(QWidget *parent, const QGLWidget *shareWidget, Qt::WFlags flags) : QGLViewer(parent, shareWidget, flags), _last_key_event(QEvent::None, 0, Qt::NoModifier) {
    _last_key_event_processed = true;
    
    _ellipsoidDrawList = 0;
    _numDrawLists = 2;
  }

  void NICPQGLViewer::updateCameraPosition(Eigen::Isometry3f pose) {
    qglviewer::Camera *oldcam = camera();
    qglviewer::Camera *cam = new StandardCamera();
    setCamera(cam);

    Eigen::Vector3f position = pose*Vector3f(-2.0f, 0.0f, 1.0f);
    cam->setPosition(qglviewer::Vec(position[0], position[1], position[2]));
    Eigen::Vector3f upVector = pose.linear()*Vector3f(0.0f, 0.0f, 0.5f);
    upVector.normalize();
    cam->setUpVector(qglviewer::Vec(upVector[0], upVector[1], upVector[2]), true);
  
    Eigen::Vector3f lookAt = pose*Vector3f(4.0f, 0.0f, 0.0f);  
    cam->lookAt(qglviewer::Vec(lookAt[0], lookAt[1], lookAt[2]));  

    delete oldcam;
  }

  void NICPQGLViewer::setKinectFrameCameraPosition() {
    qglviewer::Camera *oldcam = camera();
    qglviewer::Camera *cam = new StandardCamera();
    setCamera(cam);

    cam->setPosition(qglviewer::Vec(0.0f, 0.0f, -1.0f));
    cam->setUpVector(qglviewer::Vec(0.0f, -1.0f, 0.0f));
    cam->lookAt(qglviewer::Vec(0.0f, 0.0f, 0.0f));  

    delete oldcam;
  }

  void NICPQGLViewer::init() {
    // Init QGLViewer.
    QGLViewer::init();
    // Set background color light yellow.
    // setBackgroundColor(QColor::fromRgb(255, 255, 194));

    // Set some default settings.
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND); 
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_FLAT);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Don't save state.
    setStateFileName(QString::null);

    // Mouse bindings.
    setMouseBinding(Qt::RightButton, CAMERA, ZOOM);
    setMouseBinding(Qt::MidButton, CAMERA, TRANSLATE);

    // Replace camera.
    qglviewer::Camera *oldcam = camera();
    qglviewer::Camera *cam = new StandardCamera();
    setCamera(cam);
    cam->setPosition(qglviewer::Vec(-1.0f, 0.0f, 0.0f));
    cam->setUpVector(qglviewer::Vec(0.0f, 0.0f, 1.0f));
    cam->lookAt(qglviewer::Vec(0.0f, 0.0f, 0.0f));
    delete oldcam;

    // Create draw lists.
    _ellipsoidDrawList = glGenLists(_numDrawLists);
    _pyramidDrawList = glGenLists(_numDrawLists);

    // Compile draw lists.
    // Ellipsoid.
    glNewList(_ellipsoidDrawList, GL_COMPILE);
    drawSphere(1.0f);
    glEndList();
    // Pyramid.
    glNewList(_pyramidDrawList, GL_COMPILE);
    drawPyramid(0.5f, 0.5f);
    glEndList();
  }

  // Function containing the draw commands.
  void NICPQGLViewer::draw() {
    QGLViewer::draw();
  
    // Draw the vector of drawable objects.
    for(size_t i = 0; i < _drawableList.size(); i++) {
      if(_drawableList[i]) {
	_drawableList[i]->draw();
      }
    }
  }

  // Function to add a drawable objects to the viewer.
  void NICPQGLViewer::addDrawable(Drawable *d) {
    // Set the viewer to the input drawable object.
    d->setViewer(this);
    // Add the input object to the vector.
    _drawableList.push_back(d);
  }

  QKeyEvent* NICPQGLViewer::lastKeyEvent() {
    if(_last_key_event_processed) { return 0; }
    return &_last_key_event;
  }
  
  void NICPQGLViewer::keyEventProcessed() { _last_key_event_processed = true; }

  void NICPQGLViewer::keyPressEvent(QKeyEvent* e) {
    QGLViewer::keyPressEvent(e);
    _last_key_event = *e;
    _last_key_event_processed = false;
  }
  
}
