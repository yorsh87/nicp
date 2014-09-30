#pragma once

#include <set>

#include "pwn_aligner_gui_ui_main_window.h"

#include "pwn/pinholepointprojector.h"
#include "pwn/cylindricalpointprojector.h"
#include "pwn/sphericalpointprojector.h"
#include "pwn/statscalculatorintegralimage.h"
#include "pwn/informationmatrixcalculator.h"
#include "pwn/depthimageconverterintegralimage.h"
#include "pwn/correspondencefinder.h"
#include "pwn/linearizer.h"
#include "pwn/aligner.h"

namespace pwn_viewer {

  class PwnAlignerGuiMainWindow : public QMainWindow, public Ui::MainWindow {
    Q_OBJECT 
  public:
    PwnAlignerGuiMainWindow(std::string directory, QWidget *parent = 0, Qt::WindowFlags flags = 0);
    ~PwnAlignerGuiMainWindow();

    void setSensorOffset(const Eigen::Isometry3f &sensorOffset_) { _sensorOffset = sensorOffset_; }
    const Eigen::Isometry3f& sensorOffset() { return _sensorOffset; }

    public slots:
      virtual void visualizationUpdate();
    virtual void statsUpdate();
    virtual void correspondencesUpdate();
    virtual void alignerUpdate();
    virtual void projectorsUpdate();
    virtual void addCloud();
    virtual void pwnSnapshot();
    virtual void jpgSnapshot();
    virtual void clearLast();
    virtual void clearAll();
    virtual void correspondences();
    virtual void initialGuess();
    virtual void optimize();
    virtual void merge();
  
  protected:
    std::set<string> _readDirectory(std::string directory);
    void _selectProjector();

    unsigned int _pngCounter, _pwnCounter;

    float _depthScale;
    Eigen::Isometry3f _sensorOffset;

    pwn::PointProjector *_projector;
    pwn::PinholePointProjector *_pinholeProjector;
    pwn::CylindricalPointProjector *_cylindricalProjector;
    pwn::SphericalPointProjector *_sphericalProjector;  
    pwn::StatsCalculatorIntegralImage *_statsCalculatorIntegralImage; 
    pwn::PointInformationMatrixCalculator *_pointInformationMatrixCalculator; 
    pwn::NormalInformationMatrixCalculator *_normalInformationMatrixCalculator; 
    pwn::DepthImageConverterIntegralImage *_depthImageConverterIntegralImage; 
    pwn::CorrespondenceFinder *_correspondenceFinder;
    pwn::Linearizer *_linearizer;
    pwn::Aligner *_aligner;

    pwn::RawDepthImage _rawDepth;
    pwn::DepthImage _depth, _scaledDepth, _referenceDepth, _currentDepth;

    std::vector<pwn::Cloud*> _clouds;
    std::vector<Eigen::Isometry3f> _poses;
    std::vector<QImage*> _depths;

    QGraphicsScene *_referenceScene, *_currentScene;
  };

}
