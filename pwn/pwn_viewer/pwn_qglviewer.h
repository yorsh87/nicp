#pragma once

#include <QGLViewer/qglviewer.h>
#include <vector>
#include "drawable.h"

#include <iostream>

using namespace std;

namespace pwn_viewer {

  class PWNQGLViewer : public QGLViewer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PWNQGLViewer(QWidget *parent = 0, const QGLWidget *shareWidget = 0, Qt::WFlags flags = 0);
    virtual ~PWNQGLViewer() {}

    virtual void init();
    virtual void draw();

    virtual void addDrawable(Drawable *d);
    inline void popFront() { _drawableList.erase(_drawableList.begin()); }
    inline void popBack() { _drawableList.pop_back(); }
    inline void erase(int index) { _drawableList.erase(_drawableList.begin() + index); }
    inline void clearDrawableList() { _drawableList.clear(); }
  
    inline GLuint ellipsoidDrawList() { return _ellipsoidDrawList; }
    inline GLuint pyramidDrawList() { return _ellipsoidDrawList; }
    inline std::vector<Drawable*>& drawableList() { return _drawableList; }   
  
    void updateCameraPosition(Eigen::Isometry3f pose);
    void setKinectFrameCameraPosition();

    inline const std::vector<Drawable*>& drawableList() const { return _drawableList; }   

  protected:
    int _numDrawLists;
    GLuint _ellipsoidDrawList;
    GLuint _pyramidDrawList;
    std::vector<Drawable*> _drawableList;  
  };

}
