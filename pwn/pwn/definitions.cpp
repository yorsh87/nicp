#include "definitions.h"

namespace pwn {

  void scaleDepthImage(DepthImage& dest, const DepthImage& src, int step){
    int rows = src.rows/step;
    int cols = src.cols/step;
    dest = DepthImage(rows,cols,0.0f);
    for (int c = 0; c<dest.cols; c++){
      for (int r = 0; r<dest.rows; r++){
	float acc=0;
	int np=0;
	int sc = c*step;
	int sr = r*step;
	for (int i=0; i<step; i++){
	  for (int j=0; j<step; j++){
	    if (sc + i < src.cols&&
		sr + j < src.rows) {
	      acc += src.at<float>(sr+j,sc+i);
	      np += src.at<float>(sr+j,sc+i) > 0;
	    }
	  }
	}
	if (np)
	  dest.at<float>(r,c) = acc/np;
      }
    }
  }

}
