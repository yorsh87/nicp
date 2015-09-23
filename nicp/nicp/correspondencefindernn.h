#pragma once

#include <flann/flann.hpp>

#include "correspondencefinder.h"

namespace nicp {

    /** \class CorrespondenceFinderNN correspondencefindernn.h "correspondencefindernn.h"
     *  \brief Class that can be used to find correspondences between two point clouds.
     *  Data association is computed via nearest neighbor.
     *
     *  This class has the objective to find correspondences between two point clouds using a
     *  nearest neighbor approach. A correspondence has to satisfy some constraint like
     *  maximum angle between normals, maximum distance between the points and so on.
     */
    class CorrespondenceFinderNN : public CorrespondenceFinder {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        /**
         *  Empty constructor.
         *  This constructor creates a CorrespondenceFinder with default values for all its attributes.
         */
        CorrespondenceFinderNN();

        /**
         *  Destructor.
         */
        virtual ~CorrespondenceFinderNN();

        /**
         *  Method that returns the value of the flag normal scaling.
         *  @return the value of the normal scaling flag.
         *  @see setNormalScaling()
         */
        float normalScaling() const { return _normal_scaling; }

        /**
         *  Method that set the normal scaling flag.
         *  @param normal_scaling_ new normal scaling flag value.
         *  @see normalScaling()
         */
        void setNormalScaling(float normal_scaling_) { _normal_scaling =  normal_scaling_; }

        /**
         *  This method initialize the correspondence finder and should be called by the aligner
         *  before starting the alignment. See AlignerNN to have an idea of how to use it.
         *  @param referenceScene is a reference to the first point cloud to use to compute the Correspondence.
         *  @param currentScene is a reference to the second point cloud to use to compute the Correspondence.
         */
        void init(const Cloud &referenceScene, const Cloud &currentScene);

        /**
         *  This method computes the vector of correspondece of the two point cloud given in input.
         *  @param referenceScene is a reference to the first point cloud to use to compute the Correspondence.
         *  @param currentScene is a reference to the second point cloud to use to compute the Correspondence.
         *  @param T is an isometry that is applied to the first point cloud before to compute the Correspondence.
         */
        virtual void compute(const Cloud &referenceScene, const Cloud &currentScene, Eigen::Isometry3f T);

      protected:
        void _model2linear(std::vector<float>& dest, const Cloud& src, float nscale);
        void _model2linear(std::vector<float>& dest, const Cloud& src, float nscale, const Eigen::Isometry3f& T);

        int _knn;
        float _normal_scaling; /**< scale the weight of the normals in the data association. */
        flann::Matrix<float> _reference_matrix;
        flann::Matrix<float> _current_matrix;
        flann::Matrix<int> _indices_matrix;
        flann::Matrix<float> _distances_matrix;
        flann::Index<flann::L2<float> > *_index;

        std::vector<float> _reference_points;
        std::vector<float> _current_points;
        std::vector<int> _current_indices;
        std::vector<float> _current_distances;
    };

}
