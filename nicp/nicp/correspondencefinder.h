#pragma once

#include "cloud.h"

namespace nicp {

  /** \struct Correspondence correspondencefinder.h "correspondencefinder.h"
   *  \brief Class that can be used to represent a correspondence using indeces.
   *  
   *  This class is a structure thought to contain a correspondece, in particular this
   *  correspondence is represented using two indeces pointing to the elements forming
   *  the correspondence.
   */
  struct Correspondence {
    /**
     *  Constructor.
     *  This constructor creates a Correspondence with the indeces given in input, if no index is
     *  given then both the indeces are imposed to be -1.
     */
    Correspondence(int referenceIndex_ = -1, int currentIndex_ = -1) {
      referenceIndex = referenceIndex_;
      currentIndex = currentIndex_;
    }

    int referenceIndex; /**< Index of the correspondece in the reference depth image. */
    int currentIndex; /**< Index of the correspondece in the depth image to align. */
  };

  /** \typedef CorrespondenceVector
   * \brief A vector of Correspondence.
   */
  typedef std::vector<Correspondence> CorrespondenceVector;

  /** \class CorrespondenceFinder correspondencefinder.h "correspondencefinder.h"
   *  \brief Class that can be used to find correspondences between two reprojected point clouds.
   *  
   *  This class has the objective to find correspondences between two point clouds reprojected into
   *  their depth images. A correspondence has to satisfy some constarint like maximum angle between
   *  normals, maximum distance between the points and so on. The algorithm just pick points in the same
   *  position inside the repsective depth images and make a comparison to see if the two points are 
   *  a correspondece, if they satisfy all the constraints then the pair of points is added to the vector
   *  of correspondences.
   */
  class CorrespondenceFinder {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
    /**
     *  Empty constructor.
     *  This constructor creates a CorrespondenceFinder with default values for all its attributes.
     */
    CorrespondenceFinder();

    /**
     *  Destructor.
     */
    virtual ~CorrespondenceFinder() {}

    /**
     *  Method that returns a constant reference to the computed vector of Correspondece.
     *  @return a constant reference to the computed vector of Correspondece.
     */    
    inline const CorrespondenceVector& correspondences() const { return _correspondences; }

    /**
     *  Method that returns a reference to the computed vector of Correspondece.
     *  @return a reference to the computed vector of Correspondece.
     */
    inline CorrespondenceVector& correspondences() { return _correspondences; }
   
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
     *  Method that returns a constant reference to the depth image of the point cloud to align.
     *  @return a constant reference to the depth image of the point cloud to align.
     */    
    inline const RGBImage& currentRGBImage() const {return _currentRGBImage;}

    /**
     *  Method that returns a reference to the depth image of the point cloud to align.
     *  @return a reference to the depth image of the point cloud to align.
     */
    inline RGBImage& currentRGBImage() {return _currentRGBImage;}
        
    /**
     *  Method that returns a constant reference to the reference depth image.
     *  @return a constant reference to the reference depth image.
     */    
    inline const RGBImage& referenceRGBImage() const {return _referenceRGBImage;}
    
    /**
     *  Method that returns a reference to the reference depth image.
     *  @return a reference to the reference depth image.
     */
    inline RGBImage& referenceRGBImage() {return _referenceRGBImage;}

    /**
     *  Method that returns the squared maximum distance threshold of a Correspondence.
     *  @return the squared maximum distance threshold of a Correspondence set to the CorrespondenceFinder.
     *  @see inlierDistanceThreshold()
     *  @see setInlierDistanceThreshold()
     */
    inline float squaredThreshold() const { return _squaredThreshold; }
    
    /**
     *  Method that returns the maximum distance threshold of a Correspondence.
     *  @return the maximum distance threshold of a Correspondence set to the CorrespondenceFinder.
     *  @see setInlierDistanceThreshold()
     *  @see squaredThreshold()
     */
    inline float inlierDistanceThreshold() const { return _inlierDistanceThreshold; }

    /**
     *  Method that set the the maximum distance threshold of a Correspondence to the one given in input.
     *  @param inlierDistanceThreshold_ is a float value used to update the maximum distance threshold of a Correspondence. 
     *  @see inlierDistanceThreshold()
     *  @see squaredThreshold()
     */    
    inline void setInlierDistanceThreshold(const float inlierDistanceThreshold_) {
      _inlierDistanceThreshold = inlierDistanceThreshold_;
      _squaredThreshold = _inlierDistanceThreshold * _inlierDistanceThreshold;
    } 

    /**
     *  Method that returns the maximum curvature coefficient of a Correspondence.
     *  @return the maximum curvature coefficient of a Correspondence set to the CorrespondenceFinder.
     *  @see setFlatCurvatureThreshold()
     */
    inline float flatCurvatureThreshold() const { return _flatCurvatureThreshold; }
    
    /**
     *  Method that set the the maximum curvature coefficient of a Correspondence to the one given in input.
     *  @param flatCurvatureThreshold_ is a float value used to update the maximum curvature coefficient of a Correspondence. 
     *  @see flatCurvatureThreshold()
     */
    inline void setFlatCurvatureThreshold(const float flatCurvatureThreshold_) { _flatCurvatureThreshold = flatCurvatureThreshold_; }  

    /**
     *  Method that returns the maximum curvature coefficient ratio of a Correspondence.
     *  @return the maximum curvature coefficient ratio of a Correspondence set to the CorrespondenceFinder.
     *  @see setInlierCurvatureRatioThreshold()
     */
    inline float inlierCurvatureRatioThreshold() const { return _inlierCurvatureRatioThreshold; }
    
    /**
     *  Method that set the the maximum curvature coefficient ratio of a Correspondence to the one given in input.
     *  @param inlierCurvatureRatioThreshold_ is a float value used to update the maximum curvature coefficient ratio of a Correspondence. 
     *  @see inlierCurvatureRatioThreshold()
     */
    inline void setInlierCurvatureRatioThreshold(const float inlierCurvatureRatioThreshold_) { _inlierCurvatureRatioThreshold = inlierCurvatureRatioThreshold_; }

    /**
     *  Method that returns the maximum angle of the normals of a Correspondence.
     *  @return the maximum angle of the normals of a Correspondence set to the CorrespondenceFinder.
     *  @see setInlierNormalAngularThreshold()
     */
    inline float inlierNormalAngularThreshold() const { return _inlierNormalAngularThreshold; }
    
    /**
     *  Method that set the the maximum angle of the normals of a Correspondence to the one given in input.
     *  @param inlierNormalAngularThreshold_ is a float value used to update the maximum angle of the normals of a Correspondence. 
     *  @see inlierNormalAngularThreshold()
     */
    inline void setInlierNormalAngularThreshold(const float inlierNormalAngularThreshold_) { _inlierNormalAngularThreshold = inlierNormalAngularThreshold_; }

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
     *  Method that returns the number of Correspondence found.
     *  @return the number of Correspondence found.
     *  @see correspondences()
     */
    inline int numCorrespondences() const { return _numCorrespondences; }
    
    /**
     *  This method computes the vector of correspondece of the two point cloud given in input.
     *  @param referenceScene is a reference to the first point cloud to use to compute the Correspondence. 
     *  @param currentScene is a reference to the second point cloud to use to compute the Correspondence. 
     *  @param T is an isometry that is applied to the first point cloud before to compute the Correspondence.
     */
    void compute(const Cloud &referenceScene, const Cloud &currentScene, Eigen::Isometry3f T);

    inline int rowSearchRegion() const { return _rowSearchRegion;}
    inline int colSearchRegion() const { return _colSearchRegion;}

    inline void setRowSearchRegion(int rsr) { _rowSearchRegion = rsr; }
    inline void setColSearchRegion(int csr) { _colSearchRegion = csr; }
    
    inline bool demotedToGICP_() const { return _demotedToGICP; }
    inline void setDemotedToGICP(bool demotedToGICP_) { _demotedToGICP = demotedToGICP_; }

  protected:
    void _computeSingle(const Cloud &referenceScene, const Cloud &currentScene, Eigen::Isometry3f T);
    void _computeMulti(const Cloud &referenceScene, const Cloud &currentScene, Eigen::Isometry3f T);


    float _inlierNormalAngularThreshold; /**< Maximum angle between the normals of two points in order to be considered a correspondence. */
    float _flatCurvatureThreshold; /**< Maximum curvature coefficient for the two points in order to be considered a correspondence. */
    float _inlierCurvatureRatioThreshold; /**< Maximum curvature coefficient ratio of two points in order to be considered a correspondence. */
    float _inlierDistanceThreshold; /**< Maximum distance between two points in order to be considered a correspondence. */
    float _squaredThreshold; /**< Squared maximum distance between two points in order to be considered a correspondence. */
  
    int _numCorrespondences; /**< Number of Correspondence found. */
    CorrespondenceVector _correspondences; /**< A vector of Correspondece. */

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
    bool _demotedToGICP;
  };

}
