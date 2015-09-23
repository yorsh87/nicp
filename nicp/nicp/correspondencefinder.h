#pragma once

#include "cloud.h"

namespace nicp {

  /** \struct Correspondence correspondencefinder.h "correspondencefinder.h"
   *  \brief Class that can be used to represent a correspondence using indices.
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
   *  \brief Class interface that can be used to build correspondence finder to make data association between two point clouds.
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
     *  Method that returns the number of Correspondence found.
     *  @return the number of Correspondence found.
     *  @see correspondences()
     */
    inline int numCorrespondences() const { return _numCorrespondences; }

    /**
     *  This method computes the vector of correspondece of the two point cloud given in input.
     *  @param referenceScene is a reference to the first point cloud to use to compute the Correspondence.
     *  @param currentScene is a reference to the second point cloud to use to compute the Correspondence.
     *  @param T is an isometry that is applied to the reference point cloud before to compute the Correspondence.
     */
    virtual void compute(const Cloud &referenceScene, const Cloud &currentScene, Eigen::Isometry3f T) = 0;

    /**
     *  Method that returns true if the correspondence finder is demoted to behave like GICP,
     *  in this case the normals are not used for outliers rejection.
     *  @return the status relative to GICP behavior of the correspondence finder.
     *  @see setDemotedToGICP()
     */
    inline bool demotedToGICP_() const { return _demotedToGICP; }
    /**
     *  Method set/unset the correspondence finder to behave like GICP,
     *  in this case the normals are not used for outliers rejection.
     *  @param demotedToGICP_ is the input bool value used to set/unset GICP behavior.
     *  @see demotedToGICP()
     */
    inline void setDemotedToGICP(bool demotedToGICP_) { _demotedToGICP = demotedToGICP_; }

  protected:
    float _inlierNormalAngularThreshold; /**< Maximum angle between the normals of two points in order to be considered a correspondence. */
    float _flatCurvatureThreshold; /**< Maximum curvature coefficient for the two points in order to be considered a correspondence. */
    float _inlierCurvatureRatioThreshold; /**< Maximum curvature coefficient ratio of two points in order to be considered a correspondence. */
    float _inlierDistanceThreshold; /**< Maximum distance between two points in order to be considered a correspondence. */
    float _squaredThreshold; /**< Squared maximum distance between two points in order to be considered a correspondence. */

    int _numCorrespondences; /**< Number of Correspondence found. */
    CorrespondenceVector _correspondences; /**< A vector of Correspondece. */

    bool _demotedToGICP; /**< If true normals will not be used for outliers rejection. */
  };

}
