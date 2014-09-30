#include "pointintegralimage.h"

#include <omp.h>

namespace pwn {

  void PointIntegralImage::compute(const IntImage &indices, const PointVector &points) {
    assert(points.size() > 0 && "PointIntegralImage: points has zero size");
    assert(indices.rows > 0 && indices.cols > 0 && "PointIntegralImage: indices has zero size");
    
    if(cols() != indices.cols || rows() != indices.rows)
      resize(indices.cols, indices.rows);
    clear();

    // Fill the accumulators with the points
#pragma omp parallel for
    for(int c = 0; c < cols(); c++) {
      const int* rowptr =  &indices(c,0);
      for(int r = 0; r < rows(); r++) {
	const int &index = rowptr[r]; 
	PointAccumulator &acc = coeffRef(r, c);
	if(index < 0)
	  continue;
	const Point &point = points[index];
	acc.operator += (point);
      }
    }
    
    // Fill by column
#pragma omp parallel for
    for(int c = 0; c < cols(); c++) {
      for(int r = 1; r < rows(); r++) {
	coeffRef(r, c) += coeffRef(r - 1, c);
      }
    }

    // Fill by row
#pragma omp parallel for
    for(int r = 0; r < rows(); r++) {
      for(int c = 1; c < cols(); c++) {
	coeffRef(r, c) += coeffRef(r, c - 1);
      }
    }
  }

  void PointIntegralImage::clear() {
    int s = rows() * cols();
    PointAccumulator *p = data();
    for(int i = 0; i < s; i++, p++)
      p->clear();
  }

  PointAccumulator PointIntegralImage::getRegion(int xmin, int xmax, int ymin, int ymax) {
    assert(rows() > 0 && cols() > 0 && "PointIntegralImage: this has zero size");
    
    PointAccumulator pa;
    xmin = _clamp(xmin - 1, 0, rows() - 1);
    xmax = _clamp(xmax - 1, 0, rows() - 1);
    ymin = _clamp(ymin - 1, 0, cols() - 1);
    ymax = _clamp(ymax - 1, 0, cols() - 1);
    pa = coeffRef(xmax, ymax); // Total
    pa += coeffRef(xmin, ymin);  // Upper right
    pa -= coeffRef(xmin, ymax);  // Upper rectangle
    pa -= coeffRef(xmax, ymin);  // Rightmost rectangle
    return pa;
  }

  inline int PointIntegralImage::_clamp(int v, int min, int max) {
    v = (v < min) ? min : v;
    v = (v > max) ? max : v;
    return v;
  }

}
