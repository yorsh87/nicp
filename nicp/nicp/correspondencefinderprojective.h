#pragma once

#include "correspondencefinder.h"

namespace nicp {

  /** \class CorrespondenceFinderProjective correspondencefinderprojective.h "correspondencefinderprojective.h"
   *  \brief Class that can be used to find correspondences between two reprojected point clouds.
   *
   *  This class has the objective to find correspondences between two point clouds reprojected into
   *  their depth images. A correspondence has to satisfy some constraint like maximum angle between
   *  normals, maximum distance between the points and so on. The algorithm just pick points in the same
   *  position inside the repsective depth images and make a comparison to see if the two points are
   *  a correspondence, if they satisfy all the constraints then the pair of points is added to the vector
   *  of correspondences.
   */
  class CorrespondenceFinderProjective : public CorrespondenceFinder {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Empty constructor.
     *  This constructor creates a CorrespondenceFinder with default values for all its attributes.
     */
    CorrespondenceFinderProjective();

    /**
     *  Destructor.
     */
    virtual ~CorrespondenceFinderProjective() {}

    /**
     *  Method that returns a constant reference to the index image of the point cloud to align.
     *  @return a constant reference to the index image of the point cloud to align.
     */
    inline const IntImage& currentIndexImage() const {return _currentIndexImage;}

    /**
     *  Method that returns a reference to the index image of the point cloud to align.
     *  @return a reference to the index image of the point cloud to align.
     */
    inline IntImage& currentIndexImage() {return _currentIndexImage;}

    /**
     *  Method that returns a constant reference to the reference index image.
     *  @return a constant reference to the reference index image.
     */
    inline const IntImage& referenceIndexImage() const {return _referenceIndexImage;}

    /**
     *  Method that returns a reference to the reference index image.
     *  @return a reference to the reference index image.
     */
    inline IntImage& referenceIndexImage() {return _referenceIndexImage;}

    /**
     *  Method that returns a constant reference to the depth image of the point cloud to align.
     *  @return a constant reference to the depth image of the point cloud to align.
     */
    inline const DepthImage& currentDepthImage() const {return _currentDepthImage;}

    /**
     *  Method that returns a reference to the depth image of the point cloud to align.
     *  @return a reference to the depth image of the point cloud to align.
     */
    inline DepthImage& currentDepthImage() {return _currentDepthImage;}

    /**
     *  Method that returns a constant reference to the reference depth image.
     *  @return a constant reference to the reference depth image.
     */
    inline const DepthImage& referenceDepthImage() const {return _referenceDepthImage;}

    /**
     *  Method that returns a reference to the reference depth image.
     *  @return a reference to the reference depth image.
     */
    inline DepthImage& referenceDepthImage() {return _referenceDepthImage;}

    /**
     *  Method that returns a constant reference to the RGB image of the point cloud to align.
     *  @return a constant reference to the RGB image of the point cloud to align.
     */
    inline const RGBImage& currentRGBImage() const {return _currentRGBImage;}

    /**
     *  Method that returns a reference to the RGB image of the point cloud to align.
     *  @return a reference to the RGB image of the point cloud to align.
     */
    inline RGBImage& currentRGBImage() {return _currentRGBImage;}

    /**
     *  Method that returns a constant reference to the reference RGB image.
     *  @return a constant reference to the reference RGB image.
     */
    inline const RGBImage& referenceRGBImage() const {return _referenceRGBImage;}

    /**
     *  Method that returns a reference to the reference RGB image.
     *  @return a reference to the reference RGB image.
     */
    inline RGBImage& referenceRGBImage() {return _referenceRGBImage;}

    /**
     *  Method that returns the number of rows of the index and depth images used by the CorrespondenceFinder.
     *  @return the number of rows of the index and depth images used by the CorrespondenceFinder.
     *  @see imageCols()
     *  @see setImageSize()
     */
    inline int imageRows() const { return _rows; }

    /**
     *  Method that returns the number of columns of the index and depth images used by the CorrespondenceFinder.
     *  @return the number of columns of the index and depth images used by the CorrespondenceFinder.
     *  @see imageRows()
     *  @see setImageSize()
     */
    inline int imageCols() const { return _cols; }

    /**
     *  Method that set the size of the index and depth images used by the CorrespondenceFinder.
     *  @param rows_ is an int value used to set the number of rows of the index and depth images used by the CorrespondenceFinder.
     *  @param cols_ is an int value used to set the number of columns of the index and depth images used by the CorrespondenceFinder.
     *  @see imageRows()
     *  @see imageCols()
     */
    inline void setImageSize(const int rows_, const int cols_) {
        if(_rows != rows_ || _cols != cols_) {
            _rows = rows_;
            _cols = cols_;
            _referenceIndexImage.create(_rows, _cols);
            _currentIndexImage.create(_rows, _cols);
            _referenceRGBImage.create(_rows, _cols);
            _currentRGBImage.create(_rows, _cols);
        }
    }

    /**
     *  This method computes the vector of correspondece of the two point cloud given in input.
     *  @param referenceScene is a reference to the first point cloud to use to compute the Correspondence.
     *  @param currentScene is a reference to the second point cloud to use to compute the Correspondence.
     *  @param T is an isometry that is applied to the first point cloud before to compute the Correspondence.
     */
    virtual void compute(const Cloud &referenceScene, const Cloud &currentScene, Eigen::Isometry3f T);

    inline int rowSearchRegion() const { return _rowSearchRegion;}
    inline int colSearchRegion() const { return _colSearchRegion;}

    inline void setRowSearchRegion(int rsr) { _rowSearchRegion = rsr; }
    inline void setColSearchRegion(int csr) { _colSearchRegion = csr; }

  protected:
    void _computeSingle(const Cloud &referenceScene, const Cloud &currentScene, Eigen::Isometry3f T);
    void _computeMulti(const Cloud &referenceScene, const Cloud &currentScene, Eigen::Isometry3f T);

    int _rows; /**< Number of rows of the index images and depth images used by the CorrespondenceFinder. */
    int _cols; /**< Number of columns of the index images and depth images used by the CorrespondenceFinder. */

    IntImage _referenceIndexImage; /**< Reference index image. */
    IntImage _currentIndexImage; /**< Index image of the point cloud to align. */
    DepthImage _referenceDepthImage; /**< Reference depth image. */
    DepthImage _currentDepthImage; /**< Depth image of the point cloud to align. */
    RGBImage _referenceRGBImage;
    RGBImage _currentRGBImage;
    int _rowSearchRegion;
    int _colSearchRegion;
  };

}
