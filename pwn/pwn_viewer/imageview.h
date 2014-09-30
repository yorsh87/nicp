#pragma once

#include <qimage.h>
#include <qlabel.h>

#include "pwn/definitions.h"

using namespace pwn;

namespace pwn_viewer {

  struct DepthImageView {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    DepthImageView();
    ~DepthImageView() {}

    void computeColorMap(int cmin, int cmax, unsigned char alpha);
  
    inline unsigned int color(unsigned short idx) const { return _colorMap[idx]; }
  
    void convertToQImage(QImage &img, const RawDepthImage &m) const;
    void convertToQImage(QImage &img, const DepthImage &m) const;
  
  protected:
    unsigned int _colorMap[0xffff];
  };

  struct RGBImageView {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    RGBImageView();
    ~RGBImageView() {}
    void convertToQImage(QImage &img, const RGBImage &m) const;
  };


}
