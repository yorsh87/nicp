#include "imageview.h"

namespace pwn_viewer {

  DepthImageView::DepthImageView() {
    computeColorMap (0, 255, 0xff);
  }

  void DepthImageView::computeColorMap(int cmin, int cmax, unsigned char alpha){
    unsigned int* color = _colorMap;
    float scale = 255.0f / (float)(cmax - cmin);
    for(int i = 0; i < 0xffff; i++) {
      unsigned char c;
      if(i < cmin) {
	c = 0x00;
      } 
      else if(i > cmax) {
	c = 0xff;
      } 
      else {
	c = (unsigned char) (scale*(i - cmin));
      }
      *color++ = ((unsigned int) alpha << 24) | c << 16 | c << 8 | c;
    }
  }

  void DepthImageView::convertToQImage(QImage &img, const RawDepthImage &m) const {
    if(img.size().height() != m.rows || img.size().width() != m.cols || img.format() != QImage::Format_ARGB32)
      img = QImage(m.cols, m.rows, QImage::Format_ARGB32);
    for(int r = 0; r < m.rows; ++r) {
      for(int c = 0; c < m.cols; ++c) {
	img.setPixel(c, r, color(m(r, c)));
      }
    }
  }

  void DepthImageView::convertToQImage(QImage &img, const DepthImage &m) const {
    if(img.size().height() != m.rows || img.size().width() != m.cols || img.format() != QImage::Format_ARGB32)
      img = QImage(m.cols, m.rows, QImage::Format_ARGB32);
    for(int r = 0; r < m.rows; ++r) {
      for(int c = 0; c < m.cols; ++c) {
	img.setPixel(c, r, color(1000.0f * m(r, c)));
      }
    }
  }


  RGBImageView::RGBImageView() {
  }


  void RGBImageView::convertToQImage(QImage &img, const RGBImage &m) const {
    if(img.size().height() != m.cols || img.size().width() != m.rows || img.format() != QImage::Format_ARGB32)
      img = QImage(m.rows, m.cols, QImage::Format_ARGB32);
    for(int j = 0; j < m.cols; j++) {
      for(int i = 0; i < m.rows; i++) {
	const cv::Vec3b& c = m(i,j);
	unsigned int color = c[0] << 16 | c[1] << 8 | c[2];
	img.setPixel(i, j, color);
      }
    }
  }


}
