#include <iostream>
#include <fstream>
#include <sys/time.h>

#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include "pwn/bm_se3.h"
#include "pwn/imageutils.h"

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace pwn;

void setInputParameters(Eigen::Matrix3f &K, pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt, map<string, float> &inputParameters);
bool fillInputParametersMap(map<string, float> &inputParameters, const string &configurationFilename);
double get_time(); 
void depth2pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const DepthImage &depth, const Eigen::Matrix3f &K);

int main(int argc, char ** argv) {
  /*********************************************************************************
   *                               INPUT HANDLING                                  *
   *********************************************************************************/
  // Usage
  if(argc < 4) {
    std::cout << "Usage: pwn_icra_evaluator <configuration.txt> <associations.txt> <odometry.txt> <keepGoing>" << std::endl
	      << "\tconfiguration.txt\t-->\tinput text configuration filename" << std::endl
	      << "\tassociations.txt\t-->\tfiel containing a set of depth images associations for alignment in the format: " << std::endl
	      << "\t\t\t\t\ttimestampReference referenceDepthImageFilename timestampCurrent currentDepthImageFilename" << std::endl
	      << "\todometry.txt\t\t-->\toutput text filename containing the computed visual odometry" << std::endl;
    return 0;
  }

  // Fill input parameter map
  map<string, float> inputParameters;
  bool fillInputParameters =  fillInputParametersMap(inputParameters, argv[1]);
  if(!fillInputParameters) {
    std::cerr << "Error while reading input parameters... quitting" << std::endl;
    return -1;
  }

  // Get general parameters
  map<string, float>::iterator it;
  float depthScale = 0.001f;
  if((it = inputParameters.find("depthScale")) != inputParameters.end()) depthScale = (*it).second;
  int imageScale = 1;
  if((it = inputParameters.find("imageScale")) != inputParameters.end()) imageScale = (*it).second;
  Vector3f initialTranslation = Vector3f(0.0f, 0.0f, 0.0f);
  if((it = inputParameters.find("tx")) != inputParameters.end()) initialTranslation.x() = (*it).second;
  if((it = inputParameters.find("ty")) != inputParameters.end()) initialTranslation.y() = (*it).second;
  if((it = inputParameters.find("tz")) != inputParameters.end()) initialTranslation.z() = (*it).second;
  Quaternionf initialRotation = Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
  if((it = inputParameters.find("qx")) != inputParameters.end()) initialRotation.x() = (*it).second;
  if((it = inputParameters.find("qy")) != inputParameters.end()) initialRotation.y() = (*it).second;
  if((it = inputParameters.find("qz")) != inputParameters.end()) initialRotation.z() = (*it).second;
  if((it = inputParameters.find("qw")) != inputParameters.end()) initialRotation.w() = (*it).second;
  initialRotation.normalize();
  Isometry3f initialT;
  initialT.translation() = initialTranslation;
  initialT.linear() = initialRotation.toRotationMatrix();
  initialT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  std::cout << "Depth scale: " << depthScale << std::endl;

  /*********************************************************************************
   *                         ALIGNMENT OBJECT CREATION                             *
   *********************************************************************************/
  Eigen::Matrix3f K;
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelFilter;
  float voxelLeaf = 0.01f;
  voxelFilter.setLeafSize(voxelLeaf, voxelLeaf, voxelLeaf);
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  setInputParameters(K, ndt, inputParameters);
  
  
  /*********************************************************************************
   *                             ODOMETRY COMPUTATION                              *
   *********************************************************************************/
  // Open file containing depth images associations
  ifstream is(argv[2]);
  if(!is) {
    std::cerr << "Impossible to open depth images associations file: " << argv[2] << std::endl;
    return -1;
  }
  // Open file for output odometry
  ofstream os(argv[3]);
  if(!os) {
    std::cerr << "Impossible to open odometry file: " << argv[3] << std::endl;
    return -1;
  }

  // Read associations one by one and do the matching
  bool firstAssociation = true;
  RawDepthImage rawDepth;
  DepthImage depth, scaledDepth;
  Isometry3f globalT = initialT;
  pcl::PointCloud<pcl::PointXYZ>::Ptr referenceCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr currentCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr currentCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
  globalT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  Isometry3f deltaT = initialT;
  deltaT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  std::string previousDepthFilename;
  int counter = 0;
  double totTime = 0;
  while(is.good()) {
    char buf[4096];
    is.getline(buf, 4096);
    istringstream iss(buf);
    string timestamp, depthFilename;
    if(!(iss >> timestamp >> depthFilename)) { continue; }
    if(timestamp[0] == '#') { continue; }
    rawDepth = imread(depthFilename, -1);
    DepthImage_convert_16UC1_to_32FC1(depth, rawDepth, depthScale);
    DepthImage_scale(scaledDepth, depth, imageScale);      
    depth2pcl(currentCloud, scaledDepth, K);
    voxelFilter.setInputCloud(currentCloud);
    voxelFilter.filter(*currentCloudFiltered);    
    if(firstAssociation) { firstAssociation = false; }
    else { 
      ndt.setInputTarget(referenceCloud);
      ndt.setInputSource(currentCloudFiltered);
      std::cout << std::endl << "********** " << previousDepthFilename << " <-- " << depthFilename << " ********** " << std::endl;     
      double tBegin = get_time();
      ndt.align(*outputCloud);
      double tEnd = get_time();
      totTime += tEnd - tBegin;
      deltaT.matrix() = ndt.getFinalTransformation(); 
      deltaT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f; 

      // Transforming unfiltered, input cloud using found transform.
      pcl::transformPointCloud(*currentCloud, *outputCloud, globalT.matrix() * ndt.getFinalTransformation());
      char buff[1024];
      sprintf(buff, "pcl_%05d.pcd", counter);
      pcl::io::savePCDFileASCII(buff, *currentCloud);

      globalT = globalT * deltaT;
      globalT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f; 
    
      // Write result
      std::cout << "delta  T: " << t2v(deltaT).transpose() << std::endl; 
      std::cout << "global T: " << t2v(globalT).transpose() << std::endl; 
      Quaternionf globalRotation = Quaternionf(globalT.linear());
      globalRotation.normalize();
      os << timestamp << " " 
	 << globalT.translation().x() << " "  << globalT.translation().y() << " " << globalT.translation().z() << " " 
	 << globalRotation.x() << " " << globalRotation.y() << " " << globalRotation.z() << " " << globalRotation.w() 
	 << std::endl;   
    } 
    previousDepthFilename = depthFilename;
    referenceCloud = currentCloud;
    counter++;
  }
  double mean_time = totTime / (double)counter;
  std::cout << "Mean time frame: " << mean_time << std::endl;

  return 0;
}

void setInputParameters(Eigen::Matrix3f &K, 
			pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt,
			map<string, float> &inputParameters) {
  map<string, float>::iterator it;

  K << 
    525.0f,   0.0f, 319.5f,
      0.0f, 525.0f, 239.5f,
      0.0f,   0.0f,   1.0f;
  int rows = 480;
  int cols = 640;
  int imageScale = 1;
  if((it = inputParameters.find("fx")) != inputParameters.end()) K(0, 0) = (*it).second;
  if((it = inputParameters.find("fy")) != inputParameters.end()) K(1, 1) = (*it).second;
  if((it = inputParameters.find("cx")) != inputParameters.end()) K(0, 2) = (*it).second;
  if((it = inputParameters.find("cy")) != inputParameters.end()) K(1, 2) = (*it).second;
  if((it = inputParameters.find("rows")) != inputParameters.end()) rows = (*it).second;
  if((it = inputParameters.find("cols")) != inputParameters.end()) cols = (*it).second;
  if((it = inputParameters.find("imageScale")) != inputParameters.end()) imageScale = (*it).second;
  std::cerr << "Image scale: " << imageScale << std::endl;

  K = K / imageScale;
  K(2, 2) = 1.0f;
  rows = rows / imageScale;
  cols = cols / imageScale;
  std::cerr << "K: " << std::endl << K << std::endl;
  std::cerr << "Image size: " << rows << " --- " << cols << std::endl;

  // NDT
  ndt.setTransformationEpsilon(0.01f);
  ndt.setStepSize(0.1f);
  ndt.setResolution(1.0f);
  ndt.setMaximumIterations(35);
  if((it = inputParameters.find("transformationEpsilon")) != inputParameters.end()) ndt.setTransformationEpsilon((*it).second);
  if((it = inputParameters.find("stepSize")) != inputParameters.end()) ndt.setStepSize((*it).second);
  if((it = inputParameters.find("resolution")) != inputParameters.end()) ndt.setResolution((*it).second);
  if((it = inputParameters.find("maximumIterations")) != inputParameters.end()) ndt.setMaximumIterations((*it).second);
}

bool fillInputParametersMap(map<string, float> &inputParameters, const string &configurationFilename) {
  ifstream is(configurationFilename.c_str());
  if(!is) {
    std::cerr << "Impossible to open configuration file: " << configurationFilename << std::endl;
    return false;
  }

  while(is.good()) {
    // Get a line from the configuration file
    char buf[1024];
    is.getline(buf, 1024);
    istringstream iss(buf);
    
    // Add the parameter to the map
    string parameter; 
    float value;
    if(!(iss >> parameter >> value)) continue;
    if(parameter[0] == '#' || parameter[0] == '/') continue;
    inputParameters.insert(pair<string, float>(parameter, value));
  }
  
  return true;
}

inline double get_time() {
  struct timeval ts;
  gettimeofday(&ts, 0);
  return ts.tv_sec + ts.tv_usec * 1e-6;
}

void depth2pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const DepthImage &depth, const Eigen::Matrix3f &K) {
  cloud->clear();
  pcl::PointXYZ p;
  for(int r = 0; r < depth.rows; ++r) {
    for(int c = 0; c < depth.rows; ++c) {
      if(depth(r, c) > 0) {
	p.z = depth(r, c);
	p.y = (float(r) - K(1, 2)) * p.z / K(1, 1);
	p.x = (float(c) - K(0, 2)) * p.z / K(0, 0);
	cloud->points.push_back(p);
      }
    }
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
  Eigen::Vector4f t = Eigen::Vector4f::Zero();
  t[3] = 1.0f;
  Eigen::Quaternionf q(1.0f, 0.0f, 0.0f, 0.0f);
  cloud->sensor_origin_ = t;
  cloud->sensor_orientation_ = q;
}
