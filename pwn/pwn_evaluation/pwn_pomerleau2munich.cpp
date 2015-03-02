#include <iostream>
#include <fstream>
#include <cstdio>

#include <opencv2/highgui/highgui.hpp>

#include "pwn/imageutils.h"
#include "pwn/sphericalpointprojector.h"
#include "pwn/cloud.h"

int main(int argc, char** argv) {
  // Print usage if needed
  if(argc < 2) {
    std::cout << "Usage: pwn_pomerleau2munich <groundtruth.csv>" << std::endl
	      << "\tgroundtruth.csv\t->\tfile containing the groundtruth of ETH Pomerleau dataset" << std::endl;
    return 0;
  }

  // Open file containing the the input groundtruth
  ifstream is(argv[1]);
  if(!is) {
    std::cerr << "[ERROR] Impossible to open input groundtruth file: " << argv[1] << "... quitting" << std::endl;
    return 0;
  }

  // Open file containing the output groundtruth file
  ofstream g_os("groundtruth.txt");
  if(!g_os) {
    std::cerr << "[ERROR] Impossible to open output groundtruth file groundtruth.txt... quitting" << std::endl;
    return 0;
  }
  
  // Open file containing the depth images list file
  ofstream d_os("depth.txt");
  if(!d_os) {
    std::cerr << "[ERROR] Impossible to open output depth images list file depth.txt... quitting" << std::endl;
    return 0;
  }

  pwn::IndexImage indeces;
  pwn::DepthImage depth;
  pwn::RawDepthImage rawDepth;
  Eigen::Isometry3f sensorOffset = Eigen::Isometry3f::Identity();
  sensorOffset.linear() = Eigen::Quaternionf(-0.5f, 0.5f, -0.5f, 0.5f).toRotationMatrix();
  
  while(is.good()) {
    // Read next entry
    char buff[4096];
    is.getline(buff, 4096);
    istringstream iss(buff);
    int id;
    Eigen::Matrix4f T;
    std::string timestamp;
    bool readLine = iss >> id >> timestamp
			>> T(0, 0) >> T(0, 1) >> T(0, 2) >> T(0, 3)
			>> T(1, 0) >> T(1, 1) >> T(1, 2) >> T(1, 3)
			>> T(2, 0) >> T(2, 1) >> T(2, 2) >> T(2, 3)
			>> T(3, 0) >> T(3, 1) >> T(3, 2) >> T(3, 3);
    if(!readLine) { 
      std::cerr << std::endl << "[WARNING] Not valid entry encountered during groundtruth parsing... skipping" << std::endl;
      continue; 
    }    
    
    // Read the point cloud
    char cloudFilename[1024];
    sprintf(cloudFilename, "Hokuyo_%d.csv", id);
    ifstream cloud_is(cloudFilename);
    if(!cloud_is) {
      std::cerr << std::endl << "[WARNING] Impossible to open file: " << cloudFilename << "... skipping" << std::endl;
      continue;
    }
    else { 
      std::cout << ".";
      cout.flush();
    }
    pwn::Cloud* cloud = new pwn::Cloud();
    while(cloud_is.good()) {      
      char row[4096];
      cloud_is.getline(row, 4096);
      istringstream cloud_iss(row);
      std::string pTimestamp;
      pwn::Point p;
      if(cloud_iss >> pTimestamp >> p.x() >> p.y() >> p.z()) { cloud->points().push_back(p); }
    }
    
    // Create and save depth image
    if(cloud->points().size() > 0) {
      cloud->transformInPlace(sensorOffset.inverse());
      std::string filename(cloudFilename);
      Eigen::Vector3f t = T.block<3, 1>(0, 3);
      Eigen::Quaternionf q(T.block<3, 3>(0, 0));
      q.normalize();
      pwn::SphericalPointProjector projector;
      projector.setHorizontalFov(M_PI);
      projector.setVerticalFov(M_PI / 2.0f);
      projector.setTransform(Eigen::Isometry3f::Identity());
      projector.setImageSize(1000, 1500);
      projector.setMinDistance(0.01f);
      projector.setMaxDistance(30.0f);
      projector.scale(1.0f);
      projector.project(indeces, depth, cloud->points());
      pwn::DepthImage_convert_32FC1_to_16UC1(rawDepth, depth);
      cv::imwrite("depth/" + filename.substr(0, filename.length() - 3) + "pgm", rawDepth);
      // cloud->save(std::string(filename.substr(0, filename.length() - 3) + "pwn").c_str());
      g_os << timestamp << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      d_os << timestamp << " " << "depth/" + filename.substr(0, filename.length() - 3) + "pgm" << std::endl;

      // cloud->clear();
      // projector.unProject(cloud->points(), indeces, depth);
      // cloud->save("test.pwn");
    }    
    
    delete cloud;
  }
  

  return 0;
}
