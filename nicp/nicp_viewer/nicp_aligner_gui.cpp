#include <iostream>

#include <qapplication.h>

#include "nicp_aligner_gui_main_window.h"

int main(int argc, char **argv) {
  if(argc < 2 || 
     std::string(argv[1]) == "-h" || std::string(argv[1]) == "-help" || 
     std::string(argv[1]) == "--h" || std::string(argv[1]) == "--help") {
    std::cout << "USAGE: nicp_aligner_gui working_directory" << std::endl;
    std::cout << "working_directory \n "
	      << "\t directory where to search for .pgm depth images" 
	      << std::endl;
    return 0;
  }
  std::string working_directory = std::string(argv[1]);

  QApplication qApplication(argc, argv);
  nicp_viewer::NICPAlignerGuiMainWindow mainWindow(working_directory);
  mainWindow.viewer->setKinectFrameCameraPosition();
  mainWindow.showMaximized();
  mainWindow.show();

  // ETH Laser
  mainWindow.spherical_projector_radioButton->setChecked(true);
  mainWindow.rows_spinBox->setValue(1000);
  mainWindow.cols_spinBox->setValue(1500);
  mainWindow.min_distance_doubleSpinBox->setValue(0.5f);
  mainWindow.max_distance_doubleSpinBox->setValue(30.0f);
  mainWindow.min_image_radius_spinBox->setValue(10);
  mainWindow.max_image_radius_spinBox->setValue(100);
  mainWindow.min_points_spinBox->setValue(30);
  mainWindow.world_radius_doubleSpinBox->setValue(0.25f);
  mainWindow.curv_threshold_doubleSpinBox->setValue(0.1f);
  mainWindow.normal_angle_doubleSpinBox->setValue(0.78f);
  mainWindow.point_distance_doubleSpinBox->setValue(1.5f);
  mainWindow.statsUpdate();
  mainWindow.correspondencesUpdate();
  mainWindow.alignerUpdate();
  mainWindow.projectorsUpdate();

  // ETH Kinect
  mainWindow.pinhole_projector_radioButton->setChecked(true);
  mainWindow.fx_doubleSpinBox->setValue(131.25); 
  mainWindow.fy_doubleSpinBox->setValue(131.25);
  mainWindow.cx_doubleSpinBox->setValue(79.875);
  mainWindow.cy_doubleSpinBox->setValue(59.875);
  mainWindow.rows_spinBox->setValue(120);
  mainWindow.cols_spinBox->setValue(160);
  mainWindow.min_distance_doubleSpinBox->setValue(0.01f);
  mainWindow.max_distance_doubleSpinBox->setValue(5.0f);
  mainWindow.min_image_radius_spinBox->setValue(5);
  mainWindow.max_image_radius_spinBox->setValue(10);
  mainWindow.min_points_spinBox->setValue(20);
  mainWindow.world_radius_doubleSpinBox->setValue(0.2f);
  mainWindow.curv_threshold_doubleSpinBox->setValue(0.3f);
  mainWindow.normal_angle_doubleSpinBox->setValue(0.8f);
  mainWindow.point_distance_doubleSpinBox->setValue(0.25f);
  mainWindow.max_chi2_doubleSpinBox->setValue(1.0f);
  mainWindow.curv_flatness_doubleSpinBox->setValue(0.3f);
  mainWindow.statsUpdate();
  mainWindow.correspondencesUpdate();
  mainWindow.alignerUpdate();
  mainWindow.projectorsUpdate();

  while(mainWindow.isVisible()) { qApplication.processEvents(); }

  return 0;
}
