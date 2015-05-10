#pragma once

#include "stats.h"
#include "informationmatrix.h"

namespace nicp {

  /** \class InformationMatrixCalculator informationmatrixcalculator.h "informationmatrixcalculator.h"
   *  \brief Class interface for information matrix computation.
   *  
   *  This class provides an easy interface for information matrix computation. This class should be
   *  extended in order to define a specific InformationMatrixCalculator that fits your needs.
   */
  class InformationMatrixCalculator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
    /**
     *  Empty constructor.
     *  This constructor creates an InformationMatrixCalculator with default values for all its attributes.
     */
    InformationMatrixCalculator() {
      _flatInformationMatrix.setZero();
      _nonFlatInformationMatrix.setZero();
      _flatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
      _nonFlatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
      _curvatureThreshold = 0.0f;
      _scale = 1.0f;
    }

    /**
     *  Destructor.
     */
    virtual ~InformationMatrixCalculator() {}

    /**
     *  Method that returns the base information matrix used for points lying on a flat surface.
     *  @return the base 4x4 information matrix used for points lying on a flat surface.
     *  @see setFlatInformationMatrix()
     */
    inline InformationMatrix flatInformationMatrix() const { return _flatInformationMatrix; }

    /**
     *  Method that sets the base information matrix used for points lying on a flat surface to the one given in input.
     *  @param flatInformationMatrix_ is the base 4x4 information matrix for points lying on a flat surface used to update the one set to the InformationMatrixCalculator.
     *  @see flatInformationMatrix()
     */
    inline void setFlatInformationMatrix(const InformationMatrix flatInformationMatrix_) { _flatInformationMatrix = flatInformationMatrix_; }

    /**
     *  Method that returns the base information matrix used for points lying on an high curvature surface.
     *  @return the base 4x4 information matrix used for points lying on an high curvature surface.
     *  @see setNonFlatInformationMatrix()
     */
    inline InformationMatrix nonFlatInformationMatrix() const { return _nonFlatInformationMatrix; }
    
    /**
     *  Method that sets the base information matrix used for points lying on an high curvature surface to the one given in input.
     *  @param nonFlatInformationMatrix_ is the base 4x4 information matrix for points lying on an high curvature flat surface used to update the one set to the InformationMatrixCalculator.
     *  @see nonFlatInformationMatrix()
     */
    inline void setNonFlatInformationMatrix(const InformationMatrix nonFlatInformationMatrix_) { _nonFlatInformationMatrix = nonFlatInformationMatrix_; }

    /**
     *  Method that returns the curvature threshold used by the InformationMatrixCalculator.
     *  @return the curvature threshold used by the InformationMatrixCalculator.
     *  @see setCurvatureThreshold()
     */
    inline float curvatureThreshold() const { return _curvatureThreshold; }
    
    /**
     *  Method that sets the the curvature threshold used by the InformationMatrixCalculator to the one given in input.
     *  @param curvatureThreshold_ is the curvature threshold used to update the one set to the InformationMatrixCalculator.
     *  @see curvatureThreshold()
     */
    inline void setCurvatureThreshold(const float curvatureThreshold_) { _curvatureThreshold = curvatureThreshold_; }

    /**
     *  Method computes the InformationMatrixVector given the point properties and normals.
     *  @param informationMatrix is a reference to the InformationMAtrixVector that will contains the computed information matrices.
     *  @param stats is the vector containing the properties of the points.
     *  @param imageNormals is the vector of normals associated to the point cloud.
     */
    virtual void compute(InformationMatrixVector &informationMatrix,
			 const StatsVector &stats,
			 const NormalVector &imageNormals) = 0;

    inline float scale() const { return _scale; }
    inline void setScale(float scale_) { 
      _scale = scale_; 
      std::cerr << "Scalei: " << _scale << std::endl;
    }

  
  protected:
    float _curvatureThreshold; /**< Threshold valued for which points are considered lying on a flat surface or not. */
    InformationMatrix _flatInformationMatrix; /**< Base information matrix for point lying on a flat surface. */
    InformationMatrix _nonFlatInformationMatrix; /**< Base information matrix for point lying on an high curvature surface. */
    float _scale;
  };

  /** \class PointInformationMatrixCalculator informationmatrixcalculator.h "informationmatrixcalculator.h"
   *  \brief Class for computation of information matrices of points.
   *  
   *  This class allows to compyte the information matrices associated to the points of a point cloud.
   */
  class PointInformationMatrixCalculator : virtual public InformationMatrixCalculator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Empty constructor.
     *  This constructor creates a PointInformationMatrixCalculator with default values for all its attributes.
     */
    PointInformationMatrixCalculator() : InformationMatrixCalculator() {
      _flatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1000.0f, 0.001f, 0.001f));
      _nonFlatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1.0f, 0.1f, 0.1f));
      _curvatureThreshold = 0.02f;
    }

    /**
     *  Destructor.
     */
    virtual ~PointInformationMatrixCalculator() {}

    /**
     *  Method that computes the InformationMatrixVector associated to the points given the point properties and normals.
     *  @param informationMatrix is a reference to the InformationMAtrixVector that will contains the computed information matrices.
     *  @param statsVector is the vector containing the properties of the points.
     *  @param imageNormals is the vector of normals associated to the point cloud.
     */
    virtual void compute(InformationMatrixVector &informationMatrix,
			 const StatsVector &statsVector,
			 const NormalVector &imageNormals);
  };

  /** \class NormalInformationMatrixCalculator informationmatrixcalculator.h "informationmatrixcalculator.h"
   *  \brief Class for computation of information matrices of normals.
   *  
   *  This class allows to compyte the information matrices associated to the normals of a point cloud.
   */
  class NormalInformationMatrixCalculator : virtual public InformationMatrixCalculator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Empty constructor.
     *  This constructor creates a NormalInformationMatrixCalculator with default values for all its attributes.
     */
    NormalInformationMatrixCalculator() : InformationMatrixCalculator() {
       _flatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1000.0f, 0.001f, 0.001f));
       // _flatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(100000.0f, 0.00001f, 100000.0f));
      _nonFlatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(0.1f, 1.0f, 1.0f));
      _curvatureThreshold = 0.02f;
    }

    /**
     *  Destructor.
     */
    virtual ~NormalInformationMatrixCalculator() {}

    /**
     *  Method that computes the InformationMatrixVector associated to the normals given the point properties and normals.
     *  @param informationMatrix is a reference to the InformationMAtrixVector that will contains the computed information matrices.
     *  @param statsVector is the vector containing the properties of the points.
     *  @param imageNormals is the vector of normals associated to the point cloud.
     */
    virtual void compute(InformationMatrixVector &informationMatrix,
			 const StatsVector &statsVector,
			 const NormalVector &imageNormals);
  };

}
