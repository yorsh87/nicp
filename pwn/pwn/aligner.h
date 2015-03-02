#pragma once

#include "linearizer.h"
#include "pointprojector.h"
#include "cloud.h"
#include "correspondencefinder.h"
#include "se3_prior.h"

namespace pwn {

  /** \class Aligner aligner.h "aligner.h"
   *  \brief Class for point cloud alignment.
   *  
   *  This class allows to compute the homogeneous transformation that brings
   *  a point cloud to superpose an other reference point cloud.
   */
  class Aligner {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Empty constructor.
     *  This constructor creates an Aligner with default values for all its attributes.
     *  All the pointers to objects implementing an algorithm have to be set since
     *  this constructor sets them to zero.
     */
    Aligner();

    /**
     *  Destructor.
     */
    virtual ~Aligner() {}

    /**
     *  Method that returns a pointer to the point projector used by the Aligner.
     *  @return a pointer to the Aligner's point projector.
     *  @see setProjector()
     */
    inline PointProjector* projector() { return _projector; }

    /**
     *  Method that set the point projector used by the aligner to the one given in input.
     *  @param projector_ is a pointer to the point projector used to update the Aligner's point projector. 
     *  @see projector()
     */
    inline void setProjector(PointProjector *projector_) { _projector = projector_; }
    
    /**
     *  Method that returns a pointer to the reference point cloud.
     *  @return a pointer to the reference point cloud.
     *  @see setReferenceCloud()
     */
    inline const Cloud* referenceCloud() const { return _referenceCloud; }
    
    /**
     *  Method that set the reference cloud to the one given in input.
     *  @param referenceCloud_ is a pointer to the point cloud used to update the reference point cloud. 
     *  @see referenceCloud()
     */    
    inline void setReferenceCloud(Cloud *referenceCloud_) { 
      _referenceCloud = referenceCloud_; 
      //clearPriors();
    }
    
    /**
     *  Method that returns a pointer to the point cloud to align.
     *  @return a pointer to the point cloud to align.
     *  @see setCurrentCloud()
     */
    inline const Cloud* currentCloud() const { return _currentCloud; }

    /**
     *  Method that set the point cloud to align to the one given in input.
     *  @param currentCloud_ is a pointer to the point cloud used to update the point cloud to align. 
     *  @see currentCloud()
     */    
    inline void setCurrentCloud(Cloud *currentCloud_) { 
      _currentCloud = currentCloud_; 
      //clearPriors();
    }

    /**
     *  Method that returns the number of linear iterations setted to the Aligner.
     *  @return the number of linear iterations set to the Aligner.
     *  @see setOuterIterations()
     */
    inline int outerIterations() const { return _outerIterations; }

    /**
     *  Method that set the number of linear iterations the one given in input.
     *  @param outerIterations_ is an int value used to update the number of linear iterations of the Aligner. 
     *  @see outerIterations()
     */    
    inline void setOuterIterations(const int outerIterations_) { _outerIterations = outerIterations_; }

    /**
     *  Method that returns the number of nonlinear iterations setted to the Aligner.
     *  @return the number of nonlinear iterations set to the Aligner.
     *  @see setInnerIterations()
     */
    inline int innerIterations() const { return _innerIterations; }
    
    /**
     *  Method that set the number of nonlinear iterations the one given in input.
     *  @param innerIterations_ is an int value used to update the number of nonlinear iterations of the Aligner. 
     *  @see innerIterations()
     */    
    inline void setInnerIterations(const int innerIterations_) { _innerIterations = innerIterations_; }

    /**
     *  Method that returns the final transformation computed by the aligner that super pose the cloud to
     *  align to the reference one.
     *  @return an isometry transformation representing the alignment between the two given point clouds.
     */
    inline const Eigen::Isometry3f& T() const { return _T; }
    
    /**
     *  Method that returns the initial guess transformation setted to the aligner between the cloud to
     *  align and the reference one.
     *  @return an isometry transformation representing the initial guess between the two given point clouds.
     *  @see setInitialGuess()
     */
    inline const Eigen::Isometry3f& initialGuess() const { return _initialGuess; }	
    
    /**
     *  Method that set the initial guess transformation the one given in input.
     *  @param initialGuess_ is an isometry transformation used to update the initial guess setted to the Aligner. 
     *  @see initialGuess()
     */    
    inline void setInitialGuess(const Eigen::Isometry3f initialGuess_) { 
      _initialGuess = initialGuess_; 
      _initialGuess.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f; 
    }

    /**
     *  Method that returns the sensor offset applied to the input point clouds. Use this method if you
     *  setted the same sensor offset for both the point clouds.
     *  @return an isometry transformation representing the sensor offset setted for the two given point clouds.
     *  @see setSensorOffset()
     */
    inline const Eigen::Isometry3f& sensorOffset() const { return _referenceSensorOffset; }
    
    /**
     *  Method that set the sensor offset for both the point clouds.
     *  @param sensorOffset_ is an isometry transformation used to update the sensor offset of the two given point
     *  clouds. 
     *  @see sensorOffset()
     */    
    inline void setSensorOffset(const Eigen::Isometry3f sensorOffset_) { 
      setReferenceSensorOffset(sensorOffset_);
      setCurrentSensorOffset(sensorOffset_);
    }

    /**
     *  Method that returns the sensor offset applied to the reference cloud. Use this method if you
     *  setted a different sensor offset for the two point clouds.
     *  @return an isometry transformation representing the sensor offset setted for the reference cloud.
     *  @see setReferenceSensorOffset()
     */
    inline const Eigen::Isometry3f& referenceSensorOffset() const { return _referenceSensorOffset; }
   
    /**
     *  Method that set the sensor offset for the reference cloud.
     *  @param referenceSensorOffset_ is an isometry transformation used to update the sensor offset of the reference
     *  cloud. 
     *  @see referenceSensorOffset()
     */    
    inline void setReferenceSensorOffset(const Eigen::Isometry3f referenceSensorOffset_) { 
      _referenceSensorOffset = referenceSensorOffset_; 
      _referenceSensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    }

    /**
     *  Method that returns the sensor offset applied to the cloud to align. Use this method if you
     *  setted a different sensor offset for the two point clouds.
     *  @return an isometry transformation representing the sensor offset setted for the cloud to align.
     *  @see setCurrentSensorOffset()
     */
    inline const Eigen::Isometry3f& currentSensorOffset() const { return _currentSensorOffset; }

    /**
     *  Method that set the sensor offset for the cloud to align.
     *  @param currentSensorOffset_ is an isometry transformation used to update the sensor offset of the cloud
     *  to align. 
     *  @see currentSensorOffset()
     */    
    inline void setCurrentSensorOffset(const Eigen::Isometry3f currentSensorOffset_) { 
      _currentSensorOffset = currentSensorOffset_; 
      _currentSensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    }

    /**
     *  Method that returns a pointer to the Linearizer used by the Aligner.
     *  @return a pointer to the Aligner's Linearizer.
     *  @see setLinearizer()
     */
    inline Linearizer* linearizer() { return _linearizer; }

    /**
     *  Method that set the Linearizer used by the Aligner to the one given in input.
     *  @param linearizer_ is a pointer to the linearizer used to update the Aligner's linearizer. 
     *  @see linearizer()
     */
    inline void setLinearizer(Linearizer* linearizer_) { 
      _linearizer = linearizer_; 
      if( _linearizer) 
	_linearizer->setAligner(this); 
    }

    /**
     *  Method that returns a bool value that if it's true means that the debug mode is active and so
     *  more information are printed on the terminal, otherwise the debug mode is disabled.
     *  @return a bool value representing the fact that the debug mode is enabled or disabled.
     *  @see setDebug()
     */
    inline bool debug() const { return _debug; }

    /**
     *  Method that set the debug mode based on the value given in input.
     *  @param debug_ is an bool value used to change the debug mode between enabled or disabled. 
     *  @see debug()
     */    
    inline void setDebug(const bool debug_) { _debug = debug_; }

    /**
     *  Method that returns the minimum number of inliers to consider the computed final transformation valid.
     *  @return an int value representing the minimum number of inliers needed in order to consider the computed
     *  transformation valid.
     *  @see setMinInliers()
     */
    inline int minInliers() const { return _minInliers; }

    /**
     *  Method that set the the minimum number of inliers necessary to consider the computed final transformation
     *  valid to the value given in input.
     *  @param minInliers_ is an int value used to update the aligner's minimum number of inliers. 
     *  @see minInliers()
     */    
    inline void setMinInliers(const int minInliers_) { _minInliers = minInliers_; }

    /**
     *  Method that returns the minimum translational ratio for which if the translation ratio of the computed 
     *  final transformation is above this value, the transformation is not considered valid.
     *  @return a float value representing the minimum allowed translation ratio.
     *  @see setTranslationalMinEigenRatio()
     */
    inline float translationalMinEigenRatio() { return _translationalMinEigenRatio; }

    /**
     *  Method that set the the minimum translation ratio, necessary to consider the computed final transformation
     *  valid, to the value given in input.
     *  @param translationalMinEigenRatio_ is a float value used to update the aligner's minimum translational 
     *  ratio. 
     *  @see translationalMinEigenRatio()
     */    
    inline void setTranslationalMinEigenRatio(const float translationalMinEigenRatio_) { _translationalMinEigenRatio = translationalMinEigenRatio_; }
    
    /**
     *  Method that returns the minimum rotational ratio for which if the rotational ratio of the computed 
     *  final transformation is above this value, the transformation is not considered valid.
     *  @return a float value representing the minimum allowed rotational ratio.
     *  @see setRotationalMinEigenRatio()
     */
    inline float rotationalMinEigenRatio() { return _rotationalMinEigenRatio; }

    /**
     *  Method that set the the minimum rotational ratio, necessary to consider the computed final transformation
     *  valid, to the value given in input.
     *  @param rotationalMinEigenRatio_ is a float value used to update the aligner's minimum rotational
     *  ratio. 
     *  @see rotationalMinEigenRatio()
     */
    inline void setRotationalMinEigenRatio(const float rotationalMinEigenRatio_) { _rotationalMinEigenRatio = rotationalMinEigenRatio_; }
    
    /**
     *  Method that returns the translational ratio associated to the computed final transformation.
     *  @return a float value representing the translational ratio associated to the computed final transformation.
     *  @see rotationalEigenRatio()
     */    
    inline float translationalEigenRatio() { return _translationalEigenRatio; }

    /**
     *  Method that return the rotational ratio associated to the computed final transformation.
     *  @return a float value representing the rotational ratio associated to the computed final transformation.
     *  @see translationalEigenRatio()
     */    
    inline float rotationalEigenRatio() { return _rotationalEigenRatio; }

    inline float lambda() { return _lambda; }
    inline void setLambda(const float lambda_) { _lambda = lambda_; }

    /**
     *  Method that returns a pointer to the correspondence finder used by the Aligner.
     *  @return a pointer to the aligner's correspondence finder.
     *  @see setCorrespondenceFinder()
     */
    inline CorrespondenceFinder* correspondenceFinder() { return _correspondenceFinder; }
    
    /**
     *  Method that set the correspondence finder used by the aligner to the one given in input.
     *  @param correspondenceFinder_ is a pointer to the correspondence finder used to update the aligner's 
     *  correspondenceFinder. 
     *  @see correspondenceFinder()
     */
    inline void setCorrespondenceFinder(CorrespondenceFinder* correspondenceFinder_) { _correspondenceFinder = correspondenceFinder_; }

    /**
     *  This method computes the final transformation that brings the cloud to align to superpose the reference
     *  cloud.
     */
    virtual void align();
    
    /**
     *  Method that returns the infomration matrix 6x6 associated to the computed final transformation.
     *  @return a constant reference to the information matrix associated to the computed final transformation.
     */
    const Matrix6f& omega() const { return _omega; }

    /**
     *  @returns false if the solution is invalid
     */
    inline bool solutionValid() {return _solutionValid;}

    /**
     *  Method that returns the error associated to the computed final transformation.
     *  @return a float value representing the error associated to the computed final transformation.
     */
    inline float error() const { return _error; }

    /**
     *  Method that returns the number of inliers found with the computed final transformation.
     *  @return an int value containing the number of inliers found with the computed final transformation.
     */
    inline int inliers() const { return _inliers; }

    /**
     *  Method that returns the total time needed to compute the final transformation.
     *  @return a double value containing the total time needed to compute the final transformation.
     */
    inline double totalTime() const { return _totalTime; }
  
    /**
     *  This method allows to add a relative prior for the alignment given the associated mean and information
     *  matrix.
     *  @param mean is a three element vector representing the mean of the relative prior.
     *  @param informationMatrix is a 6x6 matrix representing the information matrix of the relative prior.
     *  @see addAbsolutePrior()
     *  @see clearPriors()
     */    
    void addRelativePrior(const Eigen::Isometry3f &mean, const Matrix6f &informationMatrix);

    /**
     *  This method allows to add an absolute prior for the alignment given the associated transformation, mean 
     *  and information matrix.
     *  @param referenceTransform is an isometry transformation containing the reference transformation of the 
     *  absolute prior.
     *  @param mean is a three element vector representing the mean of the relative prior.
     *  @param informationMatrix is a 6x6 matrix representing the information matrix of the relative prior.
     *  @see addRelativePrior()
     *  @see clearPriors()
     */    
    void addAbsolutePrior(const Eigen::Isometry3f &referenceTransform, const Eigen::Isometry3f &mean, const Matrix6f &informationMatrix);

    /**
     *  This method allows to clear the vector containing all the added priors.
     *  @see addRelativePrior()
     *  @see addAbsolutePrior()
     */    
    void clearPriors();

  protected:
    /**
     *  This method computes the translational ratio and the rotational ratio associated to the computed final
     *  transformation.
     *  @param mean is a three element vector representing the mean associated to the computed final
     *  transformation.
     *  @param Omega is a 6x6 matrix representing the information matrix associated to the computed final
     *  transformation.
     *  @param translationalRatio is a float containing the computed translational ratio associated to the 
     *  computed final transformation.
     *  @param rotationalRatio is a float containing the computed rotational ratio associated to the 
     *  computed final transformation.
     *  @see translationallEigenRatio()
     *  @see rotationalEigenRatio()
     */    
    void _computeStatistics(Vector6f &mean, Matrix6f &Omega, 
			    float &translationalRatio, float &rotationalRatio, bool usePriors = false) const;

    PointProjector *_projector; /**< Pointer to the point projector used by the Aligner to reproject points. */
    Linearizer *_linearizer; /**< Pointer to the linearizer used by the Aligner to linearize the error function. */
    CorrespondenceFinder *_correspondenceFinder; /**< Pointer to the correspondence finder used by the Aligner to find correspondences between the reprojected point clouds. */

    Cloud *_referenceCloud; /**< Pointer to the reference point cloud. */
    Cloud *_currentCloud; /**< Pointer to the point cloud to align. */
  
    bool _debug; /**< Bool value that if it is true additional informations will be printed on the terminal. */
    int _outerIterations; /**< Number of linear iterations. */
    int _innerIterations; /**< Number of nonlinear iterations. */
    int _minInliers; /**< Minimum number of inliers to consider the alignment valid. */

    Eigen::Isometry3f _T; /**< Alignment transformation expressing the transformation that bring the _currentCloud to superpose the _referenceCloud. */
    Eigen::Isometry3f _initialGuess; /**< Initial guess expressing a possible starting transformation that bring the _currentCloud to superpose the _referenceCloud. */
    Eigen::Isometry3f _referenceSensorOffset; /**< Sensor offset to apply to the _referenceCloud. */
    Eigen::Isometry3f _currentSensorOffset; /**< Sensor offset to apply to the _currentCloud. */
    
    Matrix6f _omega; /**< Information matrix 6x6 associated to the resulting alignment transformation. */
    Vector6f _mean; /**< Mean vector 6x1 associated to the resulting alignment transformation. */

    int _inliers; /**< Number of inliers found applying the resulting alignment transformation. */
    double _totalTime; /**< Total time needed to find an alignment. */
    float _error; /**< Error associated to the resulting alignment transformation. */
    float _lambda;
    float  _translationalEigenRatio; /**< Ratio of the translational part of the resulting alignment transformation. */
    float _rotationalEigenRatio; /**< Ratio of the rotational part of the resulting alignment transformation. */
    float _translationalMinEigenRatio; /**< Minimum translational ratio for which above it the resulting transformation is not considered valid. */
    float _rotationalMinEigenRatio; /**< Minimum rotational ratio for which above it the resulting transformation is not considered valid. */

    std::vector<SE3Prior*> _priors; /**< Vector of SE3 priors to apply during the alignment. */

    bool _solutionValid;
  };

}
