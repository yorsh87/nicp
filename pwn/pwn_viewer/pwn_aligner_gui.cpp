#include <iostream>

#include <qapplication.h>

#include "pwn_aligner_gui_main_window.h"

int main(int argc, char **argv) {
  if(argc < 2 || 
     std::string(argv[1]) == "-h" || std::string(argv[1]) == "-help" || 
     std::string(argv[1]) == "--h" || std::string(argv[1]) == "--help") {
    std::cout << "USAGE: pwn_aligner_gui working_directory" << std::endl;
    std::cout << "working_directory \n "
	      << "\t directory where to search for .pgm depth images" 
	      << std::endl;
    return 0;
  }
  std::string working_directory = std::string(argv[1]);

  QApplication qApplication(argc, argv);
  pwn_viewer::PwnAlignerGuiMainWindow mainWindow(working_directory);
  mainWindow.viewer->setKinectFrameCameraPosition();
  mainWindow.showMaximized();
  mainWindow.show();

  mainWindow.spherical_projector_radioButton->setChecked(true);
  mainWindow.rows_spinBox->setValue(800);
  mainWindow.cols_spinBox->setValue(1500);
  mainWindow.min_distance_doubleSpinBox->setValue(1.25f);
  mainWindow.max_distance_doubleSpinBox->setValue(10.0f);
  mainWindow.min_image_radius_spinBox->setValue(50);
  mainWindow.max_image_radius_spinBox->setValue(100);
  mainWindow.min_points_spinBox->setValue(100);
  mainWindow.world_radius_doubleSpinBox->setValue(0.25f);
  mainWindow.curv_threshold_doubleSpinBox->setValue(0.2f);
  // mainWindow.normal_angle_doubleSpinBox->setValue(0.5f);
  // mainWindow.point_distance_doubleSpinBox->setValue(1.5f);
  mainWindow.statsUpdate();
  mainWindow.correspondencesUpdate();
  mainWindow.alignerUpdate();
  mainWindow.projectorsUpdate();

  while(mainWindow.isVisible()) { qApplication.processEvents(); }

  return 0;
}
