#pragma once

#include <GL/gl.h>
#include <QObject>

namespace pwn_viewer {

  class GLParameter : public QObject {
    Q_OBJECT;
 
  public:
    GLParameter();
    virtual ~GLParameter() {}

    virtual void applyGLParameter() = 0;

    inline int step() { return _step; }
    inline void setStep(int step_) { _step = step_; }  

    inline bool show() const { return _show; }
    inline void setShow(bool show_) { _show = show_; }
  
  protected:
    int _step;
    bool _show;
  };

}
