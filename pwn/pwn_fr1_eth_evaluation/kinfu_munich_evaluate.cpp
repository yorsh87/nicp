#include <iostream>
#include <fstream>
#include <sys/time.h>

#include <opencv2/highgui/highgui.hpp>

#include <pcl/gpu/kinfu_large_scale/kinfu.h>

#include "pwn/bm_se3.h"
#include "pwn/imageutils.h"
#include "pwn_kinfu_tracker.h"
#include "image_view.h"

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace pwn;
using namespace pwn_fr1_eth_evaluation;

void setInputParameters(PWNKinfuTracker &pwnKinfuTracker, map<string, float> &inputParameters);
bool fillInputParametersMap(map<string, float> &inputParameters, const string &configurationFilename);
double get_time(); 
void rawDepth2PtrStepSz(pcl::gpu::PtrStepSz<const unsigned short>& destBuffer, 
			std::vector<unsigned short>& destData,
			const RawDepthImage& srcBuffer);

int main(int argc, char ** argv) {
  /*********************************************************************************
   *                               INPUT HANDLING                                  *
   *********************************************************************************/
  // Usage
  if(argc < 5) {
    std::cout << "Usage: pwn_icra_evaluator <configuration.txt> <associations.txt> <odometry.txt> <keepGoing>" << std::endl
	      << "\tconfiguration.txt\t-->\tinput text configuration filename" << std::endl
	      << "\tassociations.txt\t-->\tfiel containing a set of depth images associations for alignment in the format: " << std::endl
	      << "\t\t\t\t\ttimestampReference referenceDepthImageFilename timestampCurrent currentDepthImageFilename" << std::endl
	      << "\todometry.txt\t\t-->\toutput text filename containing the computed visual odometry" << std::endl
	      << "\tkeepGoing\t\t-->\t0 if you want to stop the evaluation when kinfu is lost, 1 otherwise" << std::endl;
    return 0;
  }
  bool keepGoing = atoi(argv[4]);

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
  PWNKinfuTracker pwnKinfuTracker(Vector3f::Constant(0.0f), 0.0f, 0, 0);  
  setInputParameters(pwnKinfuTracker, inputParameters);
  // ImageView imageView;
  
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
  std::vector<unsigned short> kinfuDepthData;
  pcl::gpu::PtrStepSz<const unsigned short> kinfuDepth;
  pcl::gpu::kinfuLS::KinfuTracker::DepthMap kinfuDepthDevice;
  bool firstAssociation = true;
  RawDepthImage rawDepth, scaledRawDepth;
  DepthImage depth, scaledDepth;
  IndexImage scaledIndeces;
  Isometry3f globalT = initialT;
  globalT.translation() = pwnKinfuTracker.getLastEstimatedPose().translation();    
  globalT.linear() = pwnKinfuTracker.getLastEstimatedPose().rotation();
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
    DepthImage_convert_32FC1_to_16UC1(scaledRawDepth, scaledDepth);
    if(firstAssociation) {
      scaledIndeces.create(scaledDepth.rows, scaledDepth.cols);
      firstAssociation = false;
    }
    rawDepth2PtrStepSz(kinfuDepth, kinfuDepthData, scaledRawDepth);
    kinfuDepthDevice.upload(kinfuDepth.data, kinfuDepth.step, kinfuDepth.rows, kinfuDepth.cols);
    // imageView.showDepth(kinfuDepth);
    std::cout << std::endl << "********** " << previousDepthFilename << " <-- " << depthFilename << " ********** " << std::endl;     
    double tBegin = get_time();
    pwnKinfuTracker.processFrame(kinfuDepthDevice);
    double tEnd = get_time();
    totTime += tEnd - tBegin;
    if(!keepGoing && pwnKinfuTracker.icpIsLost()) { break; }
    globalT.translation() = pwnKinfuTracker.getLastEstimatedPose().translation();
    globalT.linear() = pwnKinfuTracker.getLastEstimatedPose().rotation();
    globalT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f; 
    
    // Write result
    std::cout << "global T: " << t2v(globalT).transpose() << std::endl; 
    Quaternionf globalRotation = Quaternionf(globalT.linear());
    globalRotation.normalize();
    os << timestamp << " " 
       << globalT.translation().x() << " "  << globalT.translation().y() << " " << globalT.translation().z() << " " 
       << globalRotation.x() << " " << globalRotation.y() << " " << globalRotation.z() << " " << globalRotation.w() 
       << std::endl;    
    previousDepthFilename = depthFilename;
    counter++;
  }
  double mean_time = totTime / (double)counter;
  std::cout << "Mean time frame: " << mean_time << std::endl;

  return 0;
}

void setInputParameters(PWNKinfuTracker &pwnKinfuTracker, map<string, float> &inputParameters) {
  map<string, float>::iterator it;

  Matrix3f K;
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
  std::cerr << "Image size: " << rows << " --- " << cols << std::endl;
  std::cerr << "K: " << std::endl << K << std::endl;  

  // Kinfu
  float volumeSize = 3.0f;
  float shiftingDistance = 1.5f;
  if((it = inputParameters.find("volumeSize")) != inputParameters.end()) volumeSize = (*it).second;
  if((it = inputParameters.find("shiftingDistance")) != inputParameters.end()) shiftingDistance = (*it).second;
  pwnKinfuTracker = PWNKinfuTracker(Vector3f::Constant(volumeSize), shiftingDistance, rows, cols);
  pwnKinfuTracker.setDepthIntrinsics(K(0, 0), K(1, 1), K(0, 2), K(1, 2));
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

void rawDepth2PtrStepSz(pcl::gpu::PtrStepSz<const unsigned short>& destBuffer, 
			std::vector<unsigned short>& destData,
			const RawDepthImage& srcBuffer) {
  int rows = srcBuffer.rows;
  int cols = srcBuffer.cols;   

  destBuffer.cols = cols;
  destBuffer.rows = rows;
  destBuffer.step = destBuffer.cols * destBuffer.elemSize();
  destData.resize(destBuffer.cols * destBuffer.rows);   
  std::fill(destData.begin(), destData.end(), 0);
  
  for(int r = 0; r < rows; r++) {
    const unsigned short *src_z_ptr = srcBuffer.ptr<unsigned short>(r);
    unsigned short *dest_z_ptr = (unsigned short*)&destData[r * cols];  
    for(int c = 0; c < cols; c++) {
      const unsigned short& src_z = *src_z_ptr;
      unsigned short& dest_z = *dest_z_ptr;
      src_z_ptr++;
      dest_z_ptr++;
      if(src_z == 0) { continue; }
      dest_z = src_z;
      dest_z = src_z; 
    }
  }
  destBuffer.data = &destData[0];      
}
