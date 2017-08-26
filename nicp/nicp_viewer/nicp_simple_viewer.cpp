#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include <fstream>
#include <string>
#include <set>
#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QListWidget>

#include "drawable_normals.h"
#include "drawable_points.h"
#include "gl_parameter_normals.h"
#include "gl_parameter_points.h"
#include "nicp_qglviewer.h"
#include "nicp/cloud.h"

using namespace Eigen;
using namespace std;
using namespace nicp;
using namespace nicp_viewer;

set<string> readDirectory(string dir);

int main(int argc, char** argv) {
  // Print usage
  std::string workingDirectory = ".";
  bool applyGlobalTransform = true;
  int step = 1;
  float pointSize = 1.0f;
  float normalLength = 0.0f;

  if(argc == 2) {
    if(std::string(argv[1]) == "-help" || std::string(argv[1]) == "--help" || 
       std::string(argv[1]) == "-h" || std::string(argv[1]) == "--h") {
      std::cout << "USAGE: ";
      std::cout << "nicp_simple_viewer <working_directory> <apply_global_transform> <step> <point_size> <normal_length>" << std::endl;
      std::cout << "   <working_directory> \t-->\t directory where to find the .nicp files (default: .)" << std::endl;
      std::cout << "   <apply_global_transform> \t-->\t if not setted all the clouds are placed in the origin (default: 1)" << std::endl;
      std::cout << "   <step> \t-->\t select how many graphics elements to skip for each cloud (default: 1)" << std::endl;
      std::cout << "   <point_size> \t-->\t size of the points (default: 1.0)" << std::endl;
      std::cout << "   <normal_length> \t-->\t lenght of the normal (default: 0.0)" << std::endl;
      return 0;
    }
  }
  if(argc > 1) { workingDirectory = std::string(argv[1]); }
  if(argc > 2) { applyGlobalTransform = atoi(argv[2]); }
  if(argc > 3) { step = atoi(argv[3]); }
  if(argc > 4) { pointSize = atof(argv[4]); }
  if(argc > 5) { normalLength = atof(argv[5]); }
  std::vector<std::string> filenames;  

  // Create GUI
  QApplication application(argc,argv);
  QWidget* mainWindow = new QWidget();
  mainWindow->setWindowTitle("nicp_simple_viewer");
  QHBoxLayout* hlayout = new QHBoxLayout();
  mainWindow->setLayout(hlayout);
  QVBoxLayout* vlayout = new QVBoxLayout();
  hlayout->addItem(vlayout);
  QVBoxLayout* vlayout2 = new QVBoxLayout();
  hlayout->addItem(vlayout2);
  hlayout->setStretch(1, 1);
  QListWidget* listWidget = new QListWidget(mainWindow);
  listWidget->setSelectionMode(QAbstractItemView::MultiSelection);
  vlayout->addWidget(listWidget);
  NICPQGLViewer* viewer = new NICPQGLViewer(mainWindow);
  vlayout2->addWidget(viewer);
  mainWindow->show();
  viewer->init();
  viewer->setAxisIsDrawn(true);
  viewer->setBackgroundColor(QColor(100, 100, 100));
  viewer->show();
  listWidget->show();
  mainWindow->showMaximized();

  set<string> filenamesSet = readDirectory(workingDirectory);
  for(set<string>::const_iterator it = filenamesSet.begin(); it != filenamesSet.end(); ++it) {
    filenames.push_back(*it);       
    QString listItem(&(*it)[0]);
    if(listItem.endsWith(".nicp", Qt::CaseInsensitive)) { 
      nicp::Cloud *cloud = new nicp::Cloud();
      Isometry3f transform;
      if(!cloud->load(transform, filenames.back().c_str())) {
	std::cerr << "Unable to load points from file [" << filenames.back() << "]" << std::endl;
	return -1;
      } 
      else { listWidget->addItem(listItem); }
      if(!applyGlobalTransform) { transform = Eigen::Matrix3f::Identity(); }
      transform.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      
      GLParameterPoints* pointsParams = new GLParameterPoints(pointSize, Eigen::Vector4f(1.0f, 0.5f, 0.0f, 1.0f));
      pointsParams->setStep(step);
      DrawablePoints* drawablePoints = new DrawablePoints(transform, pointsParams, &cloud->points(), &cloud->normals());
      viewer->addDrawable(drawablePoints);
      
      GLParameterNormals* normalParams = new GLParameterNormals(pointSize, Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f), normalLength);
      DrawableNormals* drawableNormals = new DrawableNormals(transform, normalParams, &cloud->points(), &cloud->normals());
      normalParams->setStep(step);
      normalParams->setNormalLength(normalLength);
      viewer->addDrawable(drawableNormals);
    }
  }

  // Manage viewer
  while(mainWindow->isVisible()) {        
    bool selectionChanged= false;
    for(int i = 0; i < listWidget->count(); ++i) {
      QListWidgetItem* item = listWidget->item(i);
      int dpIndex = i * 2;
      int dnIndex = dpIndex + 1;
      Drawable* drawablePoints = viewer->drawableList().at(dpIndex);
      Drawable* drawableNormals = viewer->drawableList().at(dnIndex);
      if(item && item->isSelected()) {
	if(!drawablePoints->parameter()->show()) { selectionChanged = true; }
	drawablePoints->parameter()->setShow(true);
	drawableNormals->parameter()->setShow(true);
      } 
      else {
	if(drawablePoints->parameter()->show()) { selectionChanged = true; }
	drawablePoints->parameter()->setShow(false);
	drawableNormals->parameter()->setShow(false);
      }
    }
    if(selectionChanged) { viewer->updateGL(); }

    application.processEvents();
  }
}

set<string> readDirectory(string dir) {
  DIR* dp;
  struct dirent* dirp;
  struct stat filestat;
  std::set<std::string> filenames;
  dp = opendir(dir.c_str());
  if(dp == NULL) { return filenames; }
  
  while((dirp = readdir(dp))) {
    string filepath = dir + "/" + dirp->d_name;
    if(stat(filepath.c_str(), &filestat)) { continue; }
    if(S_ISDIR(filestat.st_mode)) { continue; }
    filenames.insert(filepath);
  }
  closedir(dp);
  
  return filenames;
}
