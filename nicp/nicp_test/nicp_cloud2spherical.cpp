#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <cstdio>
#include <string.h>
#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <set>

#include <opencv2/highgui/highgui.hpp>

#include "nicp/cloud.h"
#include "nicp/imageutils.h"
#include "nicp/sphericalpointprojector.h"

const char* banner[] = {
  "nicp_cloud2spherical: load all the cloud file .nicp inside the input folder and generates the associated spherical depth image",
  "usage:",
  " nicp_cloud2spherical [options] <cloud_path>",  
  " where: ",
  " -h                     [], prints this help",
  " -offset            [bool], if true (1) it applies a transform to bring the z axis from up to forward, (default: 1)",
  " -rows               [int], number of rows for the output spherical depth image (defualt: 300)",
  " -cols               [int], number of columns for the output spherical depth image (defualt: 2000)",
  " -depth_pixel_scale  [float], scale to apply to each pixel of the depth images generated (defualt: 1000)",
  " -vfov               [float], half of the vertical field of view of the projector (defualt: PI / 6)",
  " -hfov               [float], half of the horizontal field of view of the projector (defualt: PI)",
  " -min_distance       [float], minimum point distance allowed (defualt: 0.5)",
  " -max_distance       [float], maximum point distance allowed (defualt: 30.0)",
  " -vcenter            [float], vertical center of the projector (defualt: 150)",
  " -hcenter            [float], horizontal center of the projector (defualt: 1000)",
  " <cloud_path>       [string], path to the folder containing the cloud files .nicp (defualt: .)",
  0
};

void printBanner(const char** banner) {
  const char** b = banner;
  while(*b) {
    std::cerr << *b << std::endl;
    b++;
  }
}

std::set<std::string> readDirectory(std::string dir) {
  DIR* dp;
  struct dirent* dirp;
  struct stat filestat;
  std::set<std::string> filenames;
  dp = opendir(dir.c_str());
  if(dp == NULL) { return filenames; }
  
  while((dirp = readdir(dp))) {
    std::string filepath = dir + "/" + dirp->d_name;
    if(stat(filepath.c_str(), &filestat)) { continue; }
    if(S_ISDIR(filestat.st_mode)) { continue; }
    filenames.insert(filepath);
  }
  closedir(dp);
  
  return filenames;
}

int main(int argc, char** argv) {
  bool offset = true;
  int rows = 300, cols = 2000;
  float depth_pixel_scale = 1000.0f, min_distance = 0.5f, max_distance = 30.0f;
  float h_fov = M_PI, h_center = cols / 2.0f, v_fov = M_PI / 6.0f, v_center = rows / 2.0f;
  std::string cloud_path = ".";		
  
  int c = 1;
  while(c < argc) {
    if(!strcmp(argv[c], "-h")) {
      printBanner(banner);
      return 0;
    }
    else if(!strcmp(argv[c], "-offset")) {
      c++;
      offset = atoi(argv[c]);
    }
    else if(!strcmp(argv[c], "-rows")) {
      c++;
      rows = atoi(argv[c]);
    }
    else if(!strcmp(argv[c], "-cols")) {
      c++;
      cols = atoi(argv[c]);
    }
    else if(!strcmp(argv[c], "-depth_pixel_scale")) {
      c++;
      depth_pixel_scale = atof(argv[c]);
    } 
    else if(!strcmp(argv[c], "-vfov")) {
      c++;
      v_fov = atof(argv[c]);
    }
    else if(!strcmp(argv[c], "-hfov")) {
      c++;
      h_fov = atof(argv[c]);
    }
    else if(!strcmp(argv[c], "-min_distance")) {
      c++;
      min_distance = atof(argv[c]);
    }
    else if(!strcmp(argv[c], "-max_distance")) {
      c++;
      max_distance = atof(argv[c]);
    }
    else if(!strcmp(argv[c], "-vcenter")) {
      c++;
      v_center = atof(argv[c]);
    }
    else if(!strcmp(argv[c], "-hcenter")) {
      c++;
      h_center = atof(argv[c]);
    }
    else { cloud_path = std::string(argv[c]); }
    c++;
  }

  Eigen::Isometry3f transform_offset = Eigen::Isometry3f::Identity();
  transform_offset.linear() = Eigen::Quaternionf(-0.5f, 0.5f, -0.5f, 0.5f).toRotationMatrix();  
  nicp::SphericalPointProjector projector;
  if(offset) { projector.setTransform(transform_offset); }
  projector.setImageSize(rows, cols);
  projector.setVerticalFov(v_fov);
  projector.setHorizontalFov(h_fov);
  projector.setMinDistance(min_distance);
  projector.setMaxDistance(max_distance);
  projector.scale(1.0f);
  projector.setVerticalCenter(v_center);
  projector.setHorizontalCenter(h_center);
  std::cout << "[INFO]: projector transform " << std::endl << projector.transform().matrix() << std::endl;
  std::cout << "[INFO]: projector size " << projector.imageCols() << " x " << projector.imageRows() << std::endl;
  std::cout << "[INFO]: projector vertical field of view " << projector.verticalFov() << std::endl;
  std::cout << "[INFO]: projector horizontal field of view " << projector.horizontalFov() << std::endl;
  std::cout << "[INFO]: projector max distance " << projector.maxDistance() << std::endl;
  std::cout << "[INFO]: projector min distance " << projector.minDistance() << std::endl;
  
  char buffer[1024];
  Eigen::Isometry3f cloud_global_pose;
  std::set<std::string> filename_set = readDirectory(cloud_path);
  nicp::RawDepthImage raw_depth;
  nicp::DepthImage depth;
  nicp::IndexImage indices;
  nicp::Cloud cloud;
  for(std::set<std::string>::const_iterator it = filename_set.begin(); it != filename_set.end(); ++it) {
    if(!((*it).rfind(".nicp") == ((*it).size() - 5))) { continue; }    
    std::cout << "[INFO]: processing " << (*it) << std::endl;
    cloud.load(cloud_global_pose, (*it).c_str());
    projector.project(indices, depth, cloud.points());
    sprintf(buffer, "%s.pgm", (*it).c_str());
    nicp::DepthImage_convert_32FC1_to_16UC1(raw_depth, depth, depth_pixel_scale);
    cv::imwrite(buffer, raw_depth);
  }

  return 0;
}
