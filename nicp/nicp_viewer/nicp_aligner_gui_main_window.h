#pragma once

#include <set>

#include "nicp_aligner_gui_ui_main_window.h"

#include "nicp/pinholepointprojector.h"
#include "nicp/cylindricalpointprojector.h"
#include "nicp/sphericalpointprojector.h"
#include "nicp/statscalculatorintegralimage.h"
#include "nicp/informationmatrixcalculator.h"
#include "nicp/depthimageconverterintegralimage.h"
#include "nicp/linearizer.h"
#include "nicp/alignerprojective.h"
#include "nicp/alignernn.h"

namespace nicp_viewer {

  class NICPAlignerGuiMainWindow : public QMainWindow, public Ui::MainWindow {
    Q_OBJECT
  public:
    NICPAlignerGuiMainWindow(std::string directory, QWidget *parent = 0, Qt::WindowFlags flags = 0);
    ~NICPAlignerGuiMainWindow();

    void setSensorOffset(const Eigen::Isometry3f &sensorOffset_) { _sensorOffset = sensorOffset_; }
    const Eigen::Isometry3f& sensorOffset() { return _sensorOffset; }

    public slots:
    virtual void visualizationUpdate();
    virtual void statsUpdate();
    virtual void correspondencesUpdate();
    virtual void alignerUpdate();
    virtual void projectorsUpdate();
    virtual void addCloud();
    virtual void nicpSnapshot();
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

    unsigned int _pngCounter, _nicpCounter;

    float _depthScale;
    Eigen::Isometry3f _sensorOffset;

    nicp::PointProjector *_projector;
    nicp::PinholePointProjector *_pinholeProjector;
    nicp::CylindricalPointProjector *_cylindricalProjector;
    nicp::SphericalPointProjector *_sphericalProjector;
    nicp::StatsCalculatorIntegralImage *_statsCalculatorIntegralImage;
    nicp::PointInformationMatrixCalculator *_pointInformationMatrixCalculator;
    nicp::NormalInformationMatrixCalculator *_normalInformationMatrixCalculator;
    nicp::DepthImageConverterIntegralImage *_depthImageConverterIntegralImage;
    nicp::CorrespondenceFinderProjective *_correspondenceFinderProjective;
    nicp::CorrespondenceFinderNN *_correspondenceFinderNN;
    nicp::CorrespondenceFinder *_correspondenceFinder;
    nicp::Linearizer *_linearizer;
    nicp::AlignerProjective *_alignerProjective;
    nicp::AlignerNN *_alignerNN;
    nicp::Aligner *_aligner;

    nicp::RawDepthImage _rawDepth;
    nicp::DepthImage _depth, _scaledDepth, _referenceDepth, _currentDepth;

    std::vector<nicp::Cloud*> _clouds;
    std::vector<Eigen::Isometry3f> _poses;
    std::vector<QImage*> _depths;

    QGraphicsScene *_referenceScene, *_currentScene;
  };

}
