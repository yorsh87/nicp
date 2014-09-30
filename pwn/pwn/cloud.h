#pragma once

#include "stats.h"
#include "informationmatrix.h"
#include "gaussian3.h"

using namespace std;

namespace pwn {

  /** \class Cloud cloud.h "cloud.h"
   *  \brief Class for point cloud representation.
   *  
   *  This class has the objective to easy represent a point cloud. In particular it maintains in a vector
   *  structure the homogenous 3D points composing the cloud. Using this class it is possible also to maintain
   *  additional information like the point normals, point properties, point information matrices, normal
   *  information matrices, a vector of gaussians representing the intrinsic error due to the sensor and also
   *  a traversability information about the points.
   */
  class Cloud {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Empty constructor.
     *  This constructor creates a Cloud with default values for all its attributes.
     */
    Cloud() {}
    
    void resize(size_t s);
    size_t size();

    /**
     *  Method that returns a constant reference to the vector of points.
     *  @return a constant reference to the vector of points.
     */    
    inline const PointVector& points() const { return _points; }

    /**
     *  Method that returns a reference to the vector of points.
     *  @return a reference to the vector of points.
     */
    inline PointVector& points() { return _points; }


    inline Point& point(size_t i) {return _points[i];}
    inline const Point& point(size_t i) const {return _points[i];}

    /**
     *  Method that returns a constant reference to the vector of normals.
     *  @return a constant reference to the vector of normals.
     */        
    inline const NormalVector& normals() const { return _normals; }

    /**
     *  Method that returns a reference to the vector of normals.
     *  @return a reference to the vector of normals.
     */        
    inline NormalVector& normals() { return _normals; }

    inline Normal& normal(size_t i) {return _normals[i];}
    inline const Normal& normal(size_t i) const {return _normals[i];}
    
    /**
     *  Method that returns a constant reference to the vector of point properties.
     *  @return a constant reference to the vector of point properties.
     */    
    inline const StatsVector& stats() const { return _stats; }

    /**
     *  Method that returns a reference to the vector of point properties.
     *  @return a reference to the vector of point properties.
     */    
    inline StatsVector& stats() { return _stats; }
    inline Stats& stat(size_t i) {return _stats[i];}
    inline const Stats& stat(size_t i) const {return _stats[i];}
 
    
    /**
     *  Method that returns a constant reference to the vector of point information matrices.
     *  @return a constant reference to the vector of point information matrices.
     */        
    inline const InformationMatrixVector& pointInformationMatrix() const { return _pointInformationMatrix; }

    /**
     *  Method that returns a reference to the vector of point information matrices.
     *  @return a reference to the vector of point information matrices.
     */        
    inline InformationMatrixVector& pointInformationMatrix() { return _pointInformationMatrix; }

    /**
     *  Method that returns a constant reference to the vector of normal information matrices.
     *  @return a constant reference to the vector of normal information matrices.
     */        
    inline const InformationMatrixVector& normalInformationMatrix() const { return _normalInformationMatrix; }

    /**
     *  Method that returns a reference to the vector of normal information matrices.
     *  @return a reference to the vector of normal information matrices.
     */        
    inline InformationMatrixVector& normalInformationMatrix() { return _normalInformationMatrix; }

    /**
     *  Method that returns a constant reference to the vector of gaussians of the intrinsic error due to the sensor.
     *  @return a constant reference to the vector of gaussians of the intrinsic error due to the sensor.
     */            
    inline const Gaussian3fVector& gaussians() const { return _gaussians; }

    /**
     *  Method that returns a reference to the vector of gaussians of the intrinsic error due to the sensor.
     *  @return a reference to the vector of gaussians of the intrinsic error due to the sensor.
     */            
    inline Gaussian3fVector& gaussians() { return _gaussians; }

    inline const RGBVector& rgbs() const { return _rgbs; }

    inline RGBVector& rgbs() { return _rgbs; }


    /**
     *  Method that allows to load a cloud from a file.
     *  @param T is an output parameter that will contain the isometry transformation read from the file.
     *  @param filename is the name of the input file where to read the cloud.
     *  @return a bool value that is true if the cloud was loaded correctly, false otherwise.
     *  @see save()
     */
    bool load(Eigen::Isometry3f &T, const char *filename);

    /**
     *  Method that allows to load a cloud from a file.
     *  @param T is an output parameter that will contain the isometry transformation read from the file.
     *  @param is is the input stream where to read the cloud.
     *  @return a bool value that is true if the cloud was loaded correctly, false otherwise.
     *  @see save()
     */
    bool load(Eigen::Isometry3f &T, istream &is);

    /**
     *  Method that allows to save a cloud on file.
     *  @param filename is the name of the output file where to save the cloud.
     *  @param T is an isometry transformation that is saved inside the file. The cloud is not transformed in order
     *  to maintain its local properties.
     *  @param step indicates the step at which points are saved, for example if step is 3 it is saved a point each 3.
     *  @param binary indicates if the cloud has to be saved in binary format or not.
     *  @return a bool value that is true if the cloud was saved correctly, false otherwise.
     *  @see load()
     */
    bool save(const char *filename, Eigen::Isometry3f T = Eigen::Isometry3f::Identity(), int step = 1, bool binary = true);

    /**
     *  Method that allows to save a cloud on file.
     *  @param os is the output stream where to save the cloud
     *  @param T is an isometry transformation that is saved inside the file. The cloud is not transformed in order
     *  to maintain its local properties.
     *  @param step indicates the step at which points are saved, for example if step is 3 it is saved a point each 3.
     *  @param binary indicates if the cloud has to be saved in binary format or not.
     *  @return a bool value that is true if the cloud was saved correctly, false otherwise.
     *  @see load()
     */
    bool save(ostream &os, Eigen::Isometry3f T = Eigen::Isometry3f::Identity(), int step = 1, bool binary = true);

    /**
     *  Method that clears all the data vectors of the cloud erasing all the elements.
     */    
    void clear();

    /**
     *  Method that allows to add to the current cloud object the data of an other cloud.
     *  @param cloud is the cloud where to take the data to add.
     *  @param T is an isometry transfromation to apply to input cloud before add it to the current one.
     */    
    void add(Cloud cloud, const Eigen::Isometry3f &T = Eigen::Isometry3f::Identity());

    /**
     *  Method that allows to transform the current cloud with the given input transformation.
     *  @param T is an isometry transfromation to apply to the current cloud.
     */        
    void transformInPlace(const Eigen::Isometry3f& T);

    void projectRGB(RGBImage& dest, const IndexImage& indices) const;

    void unprojectRGB(const RGBImage& rgb, const IndexImage& indices);

  protected:
    PointVector _points; /**< Vector of homegeneous 3D points. */
    NormalVector _normals; /**< Vector of homogeneous 3D normals. */
    RGBVector _rgbs;
    StatsVector _stats; /**< Vector of point properties. */
    InformationMatrixVector _pointInformationMatrix; /**< Vector of point information matrices. */
    InformationMatrixVector _normalInformationMatrix; /**< Vector of normal information matrices. */
    //std::vector<int> _traversabilityVector; /**< Vector of point traversability information. */
    Gaussian3fVector _gaussians; /**< Vector of gaussians representing the intrinsic error due to the sensor. */
  };

}
