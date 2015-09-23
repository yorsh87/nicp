#include <iostream>
#include <map>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>

#include "imageutils.h"
#include "pinholepointprojector.h"
#include "depthimageconverterintegralimage.h"
#include "statscalculatorintegralimage.h"
#include "alignerprojective.h"

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace nicp;

bool fillInputParametersMap(map<string, float> &inputParameters, const string &configurationFilename);
void setInputParameters(PinholePointProjector &pointProjector,
			StatsCalculatorIntegralImage &statsCalculator,
			PointInformationMatrixCalculator &pointInformationMatrixCalculator,
			NormalInformationMatrixCalculator &normalInformationMatrixCalculator,
			CorrespondenceFinderProjective &correspondenceFinder,
			Linearizer &linearizer,
			AlignerProjective &aligner,
			map<string, float> &inputParameters);

int main(int argc, char **argv) {
  /*********************************************************************************
   *                               INPUT HANDLING                                  *
   *********************************************************************************/
  // Print usage
  if(argc < 4) {
    std::cout << "USAGE: ";
    std::cout << "nicp_odometry configurationFilename.txt depthImageListFilename.txt visualOdometryFilename.txt" << std::endl;
    std::cout << "   configurationFilename.txt \t-->\t input text configuration filename" << std::endl;
    std::cout << "   depthImageListFilename.txt \t-->\t input text filename containing the name of the depth images" << std::endl;
    std::cout << "   visualOdometryFilename.txt \t-->\t output text filename containing computed visual odometry" << std::endl;
    return 0;
  }

  // Fill input parameter map
  map<string, float> inputParameters;
  bool fillInputParameters =  fillInputParametersMap(inputParameters,
						     argv[1]);
  if(!fillInputParameters) {
    std::cerr << "Error while reading input parameters" << std::endl;
    return 0;
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
  Isometry3f initialT;
  initialT.translation() = initialTranslation;
  initialT.linear() = initialRotation.toRotationMatrix();
  initialT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

  /*********************************************************************************
   *                         ALIGNMENT OBJECT CREATION                             *
   *********************************************************************************/
  // Create the PinholePointProjector
  PinholePointProjector pointProjector;

  // Create StatsCalculator and InformationMatrixCalculator
  StatsCalculatorIntegralImage statsCalculator;
  PointInformationMatrixCalculator pointInformationMatrixCalculator;
  NormalInformationMatrixCalculator normalInformationMatrixCalculator;

  // Create CorrespondenceFinder
  CorrespondenceFinderProjective correspondenceFinder;

  // Create Linearizer and Aligner
  Linearizer linearizer;
  AlignerProjective aligner;

  // Set alignment objects properties
  setInputParameters(pointProjector,
		     statsCalculator,
		     pointInformationMatrixCalculator,
		     normalInformationMatrixCalculator,
		     correspondenceFinder,
		     linearizer,
		     aligner,
		     inputParameters);

  // Create DepthImageConverter
  DepthImageConverterIntegralImage converter(&pointProjector,
					     &statsCalculator,
					     &pointInformationMatrixCalculator,
					     &normalInformationMatrixCalculator);

  /*********************************************************************************
   *                             ODOMETRY COMPUTATION                              *
   *********************************************************************************/
  // Open file containing the list of depth images
  ifstream is(argv[2]);
  if(!is) {
    std::cerr << "Impossible to open depth image list file: " << argv[2] << std::endl;
    return 0;
  }
  // Open file that will contain the computed visual odometry
  ofstream os(argv[3]);
  if(!os) {
    std::cerr << "Impossible to open visual odometry file: " << argv[3] << std::endl;
    return 0;
  }
  // Sequentially read the images and match each one with the previous one
  RawDepthImage rawDepth;
  DepthImage depth, scaledDepth;
  Cloud *cloud = 0, *previousCloud = 0;
  bool firstDepth = true;
  Isometry3f sensorOffset = Isometry3f::Identity();
  sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  Isometry3f globalT = initialT;
  globalT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  while(is.good()) {
    // Read an image
    char buf[1024];
    is.getline(buf, 1024);
    istringstream iss(buf);
    string timestamp, depthFilename;
    if(!(iss >> timestamp >> depthFilename)) continue;
    if(timestamp[0] == '#') continue;
    rawDepth = imread(depthFilename, -1);
    DepthImage_convert_16UC1_to_32FC1(depth, rawDepth, depthScale);
    DepthImage_scale(scaledDepth, depth, imageScale);

    // Set remaining parameters
    if(firstDepth) {
      // Scale the image size and the camera matrix to the imageScale desired
      pointProjector.setImageSize(rawDepth.rows, rawDepth.cols);
      pointProjector.scale(1.0f / imageScale);
      correspondenceFinder.setImageSize(pointProjector.imageRows(), pointProjector.imageCols());
      std::cerr << "K: " << std::endl << pointProjector.cameraMatrix() << std::endl;
      std::cerr << "Image size: " << pointProjector.imageRows() << " " << pointProjector.imageCols() << std::endl;
    }

    // Convert the depth image to a cloud
    cloud = new Cloud();

    converter.compute(*cloud, scaledDepth, sensorOffset);
    std::cout << "Current image " << depthFilename << std::endl;

    // If it is not the first depth align it with the previous one
    if(!firstDepth) {
      Eigen::Isometry3f initialGuess = Eigen::Isometry3f::Identity();
      initialGuess.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      aligner.setReferenceCloud(previousCloud);
      aligner.setCurrentCloud(cloud);
      aligner.setInitialGuess(initialGuess);
      aligner.setSensorOffset(sensorOffset);
      aligner.align();
      globalT = globalT * aligner.T();
      globalT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      std::cout << "Relative transformation: " << std::endl << aligner.T().matrix() << std::endl;
      delete(previousCloud);
    }

    // Write out global transformation
    std::cout << "Global transformation: " << std::endl << globalT.matrix() << std::endl;
    std::cout << "********************************************************" << std::endl;
    cloud->save((depthFilename + ".nicp").c_str(), globalT, 1, true);
    Quaternionf globalRotation = Quaternionf(globalT.linear());
    globalRotation.normalize();
    os << timestamp << " "
       << globalT.translation().x() << " "  << globalT.translation().y() << " " << globalT.translation().z() << " "
       << globalRotation.x() << " " << globalRotation.y() << " " << globalRotation.z() << " " << globalRotation.w()
       << std::endl;
    previousCloud = cloud;
    firstDepth = false;
  }

  return 0;
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
    if(parameter[0] == '#') continue;
    inputParameters.insert(pair<string, float>(parameter, value));
  }

  return true;
}

void setInputParameters(PinholePointProjector &pointProjector,
			StatsCalculatorIntegralImage &statsCalculator,
			PointInformationMatrixCalculator &pointInformationMatrixCalculator,
			NormalInformationMatrixCalculator &normalInformationMatrixCalculator,
			CorrespondenceFinderProjective &correspondenceFinder,
			Linearizer &linearizer,
			AlignerProjective &aligner,
			map<string, float> &inputParameters) {
  map<string, float>::iterator it;

  // Point projector
  Matrix3f cameraMatrix;
  cameraMatrix <<
    525.0f,   0.0f, 319.5f,
      0.0f, 525.0f, 239.5f,
      0.0f,   0.0f,   1.0f;
  if((it = inputParameters.find("fx")) != inputParameters.end()) cameraMatrix(0, 0) = (*it).second;
  if((it = inputParameters.find("fy")) != inputParameters.end()) cameraMatrix(1, 1) = (*it).second;
  if((it = inputParameters.find("cx")) != inputParameters.end()) cameraMatrix(0, 2) = (*it).second;
  if((it = inputParameters.find("cy")) != inputParameters.end()) cameraMatrix(1, 2) = (*it).second;
  if((it = inputParameters.find("minDistance")) != inputParameters.end()) pointProjector.setMinDistance((*it).second);
  if((it = inputParameters.find("maxDistance")) != inputParameters.end()) pointProjector.setMaxDistance((*it).second);
  pointProjector.setCameraMatrix(cameraMatrix);

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
}
