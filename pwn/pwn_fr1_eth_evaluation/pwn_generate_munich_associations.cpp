#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdlib.h>

int main(int argc, char** argv) {
  /*********************************************************************************
   *                               INPUT HANDLING                                  *
   *********************************************************************************/
  // Print usage
  if(argc < 4) {
    std::cout << "USAGE: ";
    std::cout << "pwn_generate_kinfu_associations skip_rate fullAssociationsFilename.txt associations.txt" << std::endl;
    std::cout << "   skip_rate.txt \t-->\t take a depth image each skip_rate depth images read" << std::endl;
    std::cout << "   fullAssociationsFilename.txt.txt \t-->\t input text filename containing the full associations list of depth and rgb images" << std::endl;
    std::cout << "   associationsFilename.txt \t-->\t output text filename containing the computed associations" << std::endl;
    return 0;
  }
  int skip_rate = atoi(argv[1]);

  // Open file containing the full associations list
  std::ifstream is(argv[2]);
  if(!is) {
    std::cerr << "Impossible to open full associations list file: " << argv[2] << std::endl;
    return 0;
  }
  // Open file that will contain the associations
  std::ofstream os(argv[3]);
  if(!os) {
    std::cerr << "Impossible to open associations file: " << argv[3] << std::endl;
    return 0;
  }  
  
  size_t counter = 0;
  while(is.good()) {
    char buf[4096];
    is.getline(buf, 4096);
    std::istringstream iss(buf);
    std::string rgbTimestamp, rgbFilename, depthTimestamp, depthFilename;
    if(!(iss >> rgbTimestamp >> rgbFilename >> depthTimestamp >> depthFilename)) { continue; }
    if(rgbTimestamp[0] == '#') { continue; }
    if(counter++ % skip_rate != 0) { continue; }  
    os << rgbTimestamp << " " << rgbFilename << " " << depthTimestamp << " " << depthFilename << std::endl; 
  }

  return 0;
}
