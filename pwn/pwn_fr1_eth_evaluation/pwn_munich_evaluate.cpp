#include <iostream>
#include <fstream>
#include <sys/time.h>

#include <opencv2/highgui/highgui.hpp>

#include "pwn/bm_se3.h"
#include "pwn/imageutils.h"
#include "pwn/pinholepointprojector.h"
#include "pwn/depthimageconverterintegralimage.h"
#include "pwn/statscalculatorintegralimage.h"
#include "pwn/aligner.h"
#include "pwn/merger.h"

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace pwn;

void setInputParameters(PinholePointProjector &pointProjector, 
			StatsCalculatorIntegralImage &statsCalculator,
			PointInformationMatrixCalculator &pointInformationMatrixCalculator,
			NormalInformationMatrixCalculator &normalInformationMatrixCalculator,
			CorrespondenceFinder &correspondenceFinder,
			Linearizer &linearizer,
			Aligner &aligner,
			Merger &merger, 
			map<string, float> &inputParameters);
bool fillInputParametersMap(map<string, float> &inputParameters, const string &configurationFilename);
double get_time(); 

int main(int argc, char ** argv) {
  /*********************************************************************************
   *                               INPUT HANDLING                                  *
   *********************************************************************************/
  // Usage
  if(argc < 4) {
    std::cout << "Usage: pwn_icra_evaluator <configuration.txt> <associations.txt> <odometry.txt>" << std::endl
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
  float chunkAngle = M_PI / 6.0f;
  if((it = inputParameters.find("chunkAngle")) != inputParameters.end()) chunkAngle = (*it).second;
  float chunkDistance = 1.0f;
  if((it = inputParameters.find("chunkDistance")) != inputParameters.end()) chunkDistance = (*it).second;
  float chunkInliers = 1000;
  if((it = inputParameters.find("chunkInliers")) != inputParameters.end()) chunkInliers = (*it).second;
  bool incremental = false;
  if((it = inputParameters.find("incremental")) != inputParameters.end()) incremental = (*it).second;

  /*********************************************************************************
   *                         ALIGNMENT OBJECT CREATION                             *
   *********************************************************************************/
  PinholePointProjector pointProjector;
  
  StatsCalculatorIntegralImage statsCalculator;  
  PointInformationMatrixCalculator pointInformationMatrixCalculator;
  NormalInformationMatrixCalculator normalInformationMatrixCalculator;
  
  CorrespondenceFinder correspondenceFinder;

  Linearizer linearizer;
  Aligner aligner;

  Merger merger;

  // Set alignment objects properties
  setInputParameters(pointProjector, statsCalculator,
		     pointInformationMatrixCalculator, normalInformationMatrixCalculator,
		     correspondenceFinder, linearizer, aligner,
		     merger,
		     inputParameters);
  
  DepthImageConverterIntegralImage converter(&pointProjector, 
					     &statsCalculator,
					     &pointInformationMatrixCalculator,
					     &normalInformationMatrixCalculator);
  merger.setDepthImageConverter(&converter);

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
  IndexImage scaledIndeces;
  Cloud *refCloud = 0, *curCloud = 0;
  Cloud *scene = new Cloud();  
  Isometry3f globalT = initialT;
  globalT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  Isometry3f deltaT = initialT;
  deltaT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  std::string previousDepthFilename;
  int counter = 0;
  double totTime = 0;
  while(is.good()) {
    // Read a pair of depth images and convert them to point cloud
    char buf[4096];
    is.getline(buf, 4096);
    istringstream iss(buf);
    string timestamp, depthFilename;
    if(!(iss >> timestamp >> depthFilename)) { continue; }
    if(timestamp[0] == '#') { continue; }
    if(firstAssociation) {
      rawDepth = imread(depthFilename, -1);
      DepthImage_convert_16UC1_to_32FC1(depth, rawDepth, depthScale);
      DepthImage_scale(scaledDepth, depth, imageScale);
      refCloud = new Cloud();    
      converter.compute(*refCloud, scaledDepth);
      scaledIndeces.create(scaledDepth.rows, scaledDepth.cols);
      firstAssociation = false;
    }
    else {   
      if(incremental) {
	// Scene reset
	Eigen::AngleAxisf aa(deltaT.linear());
	if(curCloud != 0 && 
	   ((float)aligner.inliers() / curCloud->points().size() < chunkInliers ||
	    deltaT.translation().norm() > chunkDistance ||
	    fabs(aa.angle()) > chunkAngle)) {
	  // char tmp[1024];
	  // sprintf(tmp, "0chunk%05d.pwn", counter);
	  // scene->save(tmp, deltaT.inverse() * globalT, 1, true);
	  std::cerr << "#################################################" 
		    << " SCENE RESET " 
		    << "#################################################" << std::endl;
	  scene->clear();
	  deltaT = Eigen::Isometry3f::Identity();
	}
	
	// Increment and merge scene
	scene->add(*refCloud, deltaT);
	merger.merge(scene, deltaT);
	converter.projector()->setTransform(Eigen::Isometry3f::Identity());

	converter.projector()->setTransform(deltaT);
	converter.projector()->project(scaledIndeces, scaledDepth, scene->points());    
	converter.compute(*refCloud, scaledDepth);
	converter.projector()->setTransform(Isometry3f::Identity());
	aligner.setReferenceCloud(refCloud);
      }
      else {
	aligner.setReferenceCloud(refCloud);
      }

      rawDepth = imread(depthFilename, -1);
      DepthImage_convert_16UC1_to_32FC1(depth, rawDepth, depthScale);
      DepthImage_scale(scaledDepth, depth, imageScale);
      curCloud = new Cloud();    
      converter.compute(*curCloud, scaledDepth);

      std::cout << std::endl << "********** " << previousDepthFilename << " <-- " << depthFilename << " ********** " << std::endl;     

      Eigen::Isometry3f initialGuess = Eigen::Isometry3f::Identity();
      initialGuess.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      aligner.setInitialGuess(initialGuess);
      aligner.setCurrentCloud(curCloud);
      double tBegin = get_time();
      aligner.align();  
      double tEnd = get_time();
      totTime += tEnd - tBegin;
      deltaT = deltaT * aligner.T();
      globalT = globalT * aligner.T();
      globalT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f; 
      
      // Write result
      std::cout << "delta T: " << t2v(aligner.T()).transpose() << std::endl;
      std::cout << "T: " << t2v(globalT).transpose() << std::endl; 
      Quaternionf globalRotation = Quaternionf(globalT.linear());
      globalRotation.normalize();
      os << timestamp << " " 
	 << globalT.translation().x() << " "  << globalT.translation().y() << " " << globalT.translation().z() << " " 
	 << globalRotation.x() << " " << globalRotation.y() << " " << globalRotation.z() << " " << globalRotation.w() 
	 << std::endl;

      if(refCloud) {
	delete refCloud;
	refCloud = 0;
      }
      refCloud = curCloud;
    }
    previousDepthFilename = depthFilename;
    counter++;
  }
  double mean_time = totTime / (double)counter;
  std::cout << "Mean time frame: " << mean_time << std::endl;

  return 0;
}

void setInputParameters(PinholePointProjector &pointProjector, 
			StatsCalculatorIntegralImage &statsCalculator,
			PointInformationMatrixCalculator &pointInformationMatrixCalculator,
			NormalInformationMatrixCalculator &normalInformationMatrixCalculator,
			CorrespondenceFinder &correspondenceFinder,
			Linearizer &linearizer,
			Aligner &aligner,
			Merger &merger,
			map<string, float> &inputParameters) {
  map<string, float>::iterator it;

  // Point projector
  Matrix3f cameraMatrix;
  cameraMatrix << 
    525.0f,   0.0f, 319.5f,
      0.0f, 525.0f, 239.5f,
      0.0f,   0.0f,   1.0f;
  int rows = 480;
  int cols = 640;
  int imageScale = 1;
  if((it = inputParameters.find("fx")) != inputParameters.end()) cameraMatrix(0, 0) = (*it).second;
  if((it = inputParameters.find("fy")) != inputParameters.end()) cameraMatrix(1, 1) = (*it).second;
  if((it = inputParameters.find("cx")) != inputParameters.end()) cameraMatrix(0, 2) = (*it).second;
  if((it = inputParameters.find("cy")) != inputParameters.end()) cameraMatrix(1, 2) = (*it).second;
  if((it = inputParameters.find("minDistance")) != inputParameters.end()) pointProjector.setMinDistance((*it).second);
  if((it = inputParameters.find("maxDistance")) != inputParameters.end()) pointProjector.setMaxDistance((*it).second);
  if((it = inputParameters.find("rows")) != inputParameters.end()) rows = (*it).second;
  if((it = inputParameters.find("cols")) != inputParameters.end()) cols = (*it).second;
  if((it = inputParameters.find("imageScale")) != inputParameters.end()) imageScale = (*it).second;
  pointProjector.setCameraMatrix(cameraMatrix);
  pointProjector.setImageSize(rows, cols);
  pointProjector.scale(1.0f / imageScale);

  // Stats calculator and information matrix calculators
  if((it = inputParameters.find("minImageRadius")) != inputParameters.end()) statsCalculator.setMinImageRadius((*it).second);
  if((it = inputParameters.find("maxImageRadius")) != inputParameters.end()) statsCalculator.setMaxImageRadius((*it).second);
  if((it = inputParameters.find("minPoints")) != inputParameters.end()) statsCalculator.setMinPoints((*it).second);
  if((it = inputParameters.find("curvatureThreshold")) != inputParameters.end()) statsCalculator.setCurvatureThreshold((*it).second);
  if((it = inputParameters.find("worldRadius")) != inputParameters.end()) statsCalculator.setWorldRadius((*it).second);    
  if((it = inputParameters.find("informationMatrixCurvatureThreshold")) != inputParameters.end()) {
    pointInformationMatrixCalculator.setCurvatureThreshold((*it).second);
    normalInformationMatrixCalculator.setCurvatureThreshold((*it).second);
  }

  // Correspondence finder
  if((it = inputParameters.find("inlierDistanceThreshold")) != inputParameters.end()) correspondenceFinder.setInlierDistanceThreshold((*it).second);
  if((it = inputParameters.find("inlierNormalAngularThreshold")) != inputParameters.end()) correspondenceFinder.setInlierNormalAngularThreshold((*it).second);
  if((it = inputParameters.find("inlierCurvatureRatioThreshold")) != inputParameters.end()) correspondenceFinder.setInlierCurvatureRatioThreshold((*it).second);
  if((it = inputParameters.find("flatCurvatureThreshold")) != inputParameters.end()) correspondenceFinder.setFlatCurvatureThreshold((*it).second);
  correspondenceFinder.setImageSize(pointProjector.imageRows(), pointProjector.imageCols());
  
  // Linearizer
  if((it = inputParameters.find("inlierMaxChi2")) != inputParameters.end()) linearizer.setInlierMaxChi2((*it).second);
  if((it = inputParameters.find("robustKernel")) != inputParameters.end()) linearizer.setRobustKernel((*it).second);    
  linearizer.setAligner(&aligner);

  // Aligner
  if((it = inputParameters.find("outerIterations")) != inputParameters.end()) aligner.setOuterIterations((*it).second);    
  if((it = inputParameters.find("innerIterations")) != inputParameters.end()) aligner.setInnerIterations((*it).second);
  if((it = inputParameters.find("minInliers")) != inputParameters.end()) aligner.setMinInliers((*it).second);
  if((it = inputParameters.find("translationalMinEigenRatio")) != inputParameters.end()) aligner.setTranslationalMinEigenRatio((*it).second);
  if((it = inputParameters.find("rotationalMinEigenRatio")) != inputParameters.end()) aligner.setRotationalMinEigenRatio((*it).second);
  aligner.setProjector(&pointProjector);
  aligner.setCorrespondenceFinder(&correspondenceFinder);
  aligner.setLinearizer(&linearizer);

  // Merger
  if((it = inputParameters.find("depthThreshold")) != inputParameters.end()) merger.setMaxPointDepth((*it).second);
  if((it = inputParameters.find("normalThreshold")) != inputParameters.end()) merger.setNormalThreshold((*it).second);
  if((it = inputParameters.find("distanceThreshold")) != inputParameters.end()) merger.setDistanceThreshold((*it).second);
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
