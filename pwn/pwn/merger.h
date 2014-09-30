#pragma once

#include "pinholepointprojector.h"
#include "depthimageconverter.h"
#include "cloud.h"

namespace pwn {

  /** \class Merger merger.h "merger.h"
   *  \brief Class for point cloud merging.
   *  
   *  This class allows to merge points in a point cloud. This is particularly useful
   *  when multiple point clouds are added togheter in a single one. This creates an excess
   *  of points that are not necessary and slow the computation. The merging process will reduct
   *  the points erasing the ones in surplus.
   */
  class Merger {
  public:  
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Empty constructor.
     *  This constructor creates a Merger with default values for all its attributes.
     *  All the pointers to objects implementing an algorithm have to be set since
     *  this constructor sets them to zero.
     */
    Merger();

    /**
     *  Destructor.
     */
    virtual ~Merger() {}
  
    /**
     *  Method that returns the distance threshold used by the Merger.
     *  @return the distance threshold used by the Merger.
     *  @see setDistanceThreshold()
     */
    inline float distanceThreshold() const { return _distanceThreshold; }
    
    /**
     *  Method that set the distance threshold used by the Merger to the one given input.
     *  @param distanceThreshold_ is a float value used to update the distance threshold used by the Merger.
     *  @see distanceThreshold()
     */
    inline void setDistanceThreshold(float distanceThreshold_) { _distanceThreshold = distanceThreshold_; }

    /**
     *  Method that returns the normal threshold used by the Merger.
     *  @return the normal threshold used by the Merger.
     *  @see setNormalThreshold()
     */
    inline float normalThreshold() const { return _normalThreshold; }	

    /**
     *  Method that set the normal threshold used by the Merger to the one given input.
     *  @param normalThreshold_ is a float value used to update the normal threshold used by the Merger.
     *  @see normalThreshold()
     */
    inline void setNormalThreshold(float normalThreshold_) { _normalThreshold = normalThreshold_; }

    /**
     *  Method that returns the depth threshold used by the Merger.
     *  @return the depth threshold used by the Merger.
     *  @see setMaxPointDepth()
     */
    inline float maxPointDepth() const { return _maxPointDepth; }

    /**
     *  Method that set the depth threshold used by the Merger to the one given input.
     *  @param maxPointDepth_ is a float value used to update the depth threshold used by the Merger.
     *  @see maxPointDepth()
     */
    inline void setMaxPointDepth(float maxPointDepth_) { _maxPointDepth = maxPointDepth_; }

    /**
     *  Method that returns the DepthImageConverter used by the Merger.
     *  @return the DepthImageConverter used by the Merger.
     *  @see setDepthImageConverter()
     */
    inline DepthImageConverter* depthImageConverter() const { return _depthImageConverter; }
    
    /**
     *  Method that set the DepthImageConverter used by the Merger to the one given input.
     *  @param depthImageConverter_ is a pointer used to update the depthImageConverter used by the Merger.
     *  @see depthImageConverter()
     */
    inline void setDepthImageConverter(DepthImageConverter *depthImageConverter_) { _depthImageConverter = depthImageConverter_; }

    /**
     *  Method that returns the image size set to the Merger.
     *  @return the image size set to the Merger.
     *  @see setImageSize()
     */
    inline Eigen::Vector2i imageSize() const { return Eigen::Vector2i(_indexImage.rows, _indexImage.cols); }  

    /**
     *  Method that set the the image size of the Merger to the one given input.
     *  @param r is an int value representing the number of rows of the image.
     *  @param c is an int value representing the number of cols of the image.
     *  @see imageSize()
     */
    inline void setImageSize(int r, int c) {
      _indexImage.create(r, c);
      _depthImage.create(r, c);
    }

    /**
     *  Method that computes the merged point cloud given a transformation pose and the point cloud to merge.
     *  @param cloud is a pointer to the cloud to merge.
     *  @param transform is the pose transofrmation to use in the merging process.
     */
    void merge(Cloud *cloud, Eigen::Isometry3f transform = Eigen::Isometry3f::Identity());
    
  protected:
    float _distanceThreshold; /**< Distance threshold for which points over it are not collapsed. */
    float _normalThreshold; /**< Normal threshold for which points with normals over it are not collapsed. */
    float _maxPointDepth; /**< Depth threshold for which points over it are not collapsed. */

    DepthImageConverter *_depthImageConverter; /**< Pointer to the DepthImageConverter. */

    DepthImage _depthImage; /**< DepthImage for inner computations. */
    IntImage _indexImage; /**< IndexImage for inner computations. */
    std::vector<int> _collapsedIndices; /**< Vector of collapsed point indeces to merge. */
  };

}
