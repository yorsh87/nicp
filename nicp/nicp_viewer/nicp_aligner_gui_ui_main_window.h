/********************************************************************************
** Form generated from reading UI file 'nicp_aligner_gui_ui_main_window.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_NICP_ALIGNER_GUI_UI_MAIN_WINDOW_H
#define UI_NICP_ALIGNER_GUI_UI_MAIN_WINDOW_H

#include <QtCore/QVariant>
#include <QAction>
#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QGraphicsView>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QListWidget>
#include <QMainWindow>
#include <QMenuBar>
#include <QPushButton>
#include <QRadioButton>
#include <QSpacerItem>
#include <QSpinBox>
#include <QStatusBar>
#include <QVBoxLayout>
#include <QWidget>
#include "nicp_qglviewer.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout_3;
    QHBoxLayout *main_horizontalLayout;
    QVBoxLayout *settings_verticalLayout;
    QGridLayout *visualization_gridLayout_;
    QLabel *normals_length_label;
    QDoubleSpinBox *normals_length_doubleSpinBox;
    QSpinBox *step_spinBox;
    QLabel *step_label;
    QLabel *points_label;
    QDoubleSpinBox *points_doubleSpinBox;
    QLabel *vosualization_options_label;
    QDoubleSpinBox *correspondences_doubleSpinBox;
    QLabel *correspondences_label;
    QLabel *covariances_label;
    QDoubleSpinBox *covariances_doubleSpinBox;
    QLabel *normals_width_label;
    QDoubleSpinBox *normals_width_doubleSpinBox;
    QFrame *line;
    QGridLayout *stats_gridLayout;
    QDoubleSpinBox *curv_threshold_doubleSpinBox;
    QLabel *max_image_radius_label;
    QLabel *min_image_radius_label;
    QSpinBox *min_image_radius_spinBox;
    QSpinBox *max_image_radius_spinBox;
    QLabel *world_radius_label;
    QDoubleSpinBox *world_radius_doubleSpinBox;
    QLabel *min_points_label;
    QLabel *stats_options_label;
    QLabel *curv_threshold_label;
    QSpinBox *min_points_spinBox;
    QFrame *line_2;
    QGridLayout *correspondences_gridLayout;
    QLabel *curva_flatness_label;
    QLabel *normal_angle_label;
    QLabel *curv_ratio_label;
    QDoubleSpinBox *normal_angle_doubleSpinBox;
    QDoubleSpinBox *curv_ratio_doubleSpinBox;
    QDoubleSpinBox *curv_flatness_doubleSpinBox;
    QLabel *point_distance_label;
    QDoubleSpinBox *point_distance_doubleSpinBox;
    QLabel *correspondences_threshold_label;
    QFrame *line_3;
    QGridLayout *aligner_gridLayout;
    QLabel *min_inliers_label;
    QSpinBox *inner_iter_spinBox;
    QLabel *inner_iter_label;
    QSpinBox *min_inliers_spinBox;
    QLabel *outer_iter_label;
    QSpinBox *outer_iter_spinBox;
    QLabel *aligner_options_label;
    QDoubleSpinBox *max_chi2_doubleSpinBox;
    QLabel *max_chi2_label;
    QLabel *lambda_label;
    QDoubleSpinBox *lambda_doubleSpinBox;
    QFrame *line_4;
    QGridLayout *alignment_gridLayout;
    QLabel *alignment_options_label;
    QPushButton *clear_last_pushButton;
    QPushButton *initial_guess_pushButton;
    QPushButton *claer_all_pushButton;
    QPushButton *optimize_pushButton;
    QPushButton *correspondences_pushButton;
    QPushButton *merge_pushButton;
    QCheckBox *step_by_step_checkBox;
    QFrame *line_5;
    QGridLayout *saving_gridLayout;
    QPushButton *png_snapshot_pushButton;
    QPushButton *nicp_snapshot_pushButton;
    QLabel *saving_options_label;
    QSpacerItem *verticalSpacer_2;
    QVBoxLayout *visualization_verticalLayout;
    QHBoxLayout *viewer_horizontalLayout;
    QVBoxLayout *viewer_verticalLayout;
    QHBoxLayout *qglviewer_horizontalLayout;
    QVBoxLayout *verticalLayout_3;
    QLabel *viewer_label;
    nicp_viewer::NICPQGLViewer *viewer;
    QGridLayout *gridLayout;
    QGraphicsView *reference_graphicsView;
    QGraphicsView *current_graphicsView;
    QLabel *reference_depth_image_label;
    QLabel *reference_depth_image_label_2;
    QHBoxLayout *depth_images_horizontalLayout;
    QVBoxLayout *reference_depth_image_verticalLayout;
    QVBoxLayout *current_depth_image_verticalLayout;
    QHBoxLayout *projectors_horizontalLayout;
    QGridLayout *point_projector_gridLayout;
    QLabel *cols_label;
    QLabel *max_distance_label;
    QLabel *min_distance_label;
    QLabel *rows_label;
    QDoubleSpinBox *max_distance_doubleSpinBox;
    QSpacerItem *verticalSpacer_8;
    QSpinBox *cols_spinBox;
    QDoubleSpinBox *min_distance_doubleSpinBox;
    QLabel *point_projector_options_label;
    QSpacerItem *verticalSpacer_6;
    QSpinBox *rows_spinBox;
    QLabel *scale_label;
    QDoubleSpinBox *scale_doubleSpinBox;
    QLabel *image_scale_label;
    QDoubleSpinBox *image_scale_doubleSpinBox;
    QGridLayout *pinhole_gridLayout;
    QDoubleSpinBox *cx_doubleSpinBox;
    QDoubleSpinBox *fy_doubleSpinBox;
    QDoubleSpinBox *fx_doubleSpinBox;
    QLabel *cy_label;
    QLabel *baseline_label;
    QLabel *fy_label;
    QDoubleSpinBox *cy_doubleSpinBox;
    QDoubleSpinBox *baseline_doubleSpinBox;
    QLabel *fx_label;
    QRadioButton *pinhole_projector_radioButton;
    QLabel *cx_label;
    QDoubleSpinBox *alpha_doubleSpinBox;
    QLabel *alpha_label;
    QSpacerItem *verticalSpacer_3;
    QSpacerItem *verticalSpacer;
    QGridLayout *spherical_gridLayout;
    QDoubleSpinBox *sph_horizontal_fov_doubleSpinBox;
    QLabel *sph_horizontal_fov_label;
    QLabel *sph_vertical_fov_label;
    QDoubleSpinBox *sph_vertical_fov_doubleSpinBox;
    QRadioButton *spherical_projector_radioButton;
    QSpacerItem *verticalSpacer_10;
    QSpacerItem *verticalSpacer_7;
    QLabel *sph_vertical_center_label;
    QLabel *sph_horizontal_center_label;
    QDoubleSpinBox *sph_vertical_center_doubleSpinBox;
    QDoubleSpinBox *sph_horizontal_center_doubleSpinBox;
    QVBoxLayout *cloud_selection_verticalLayout;
    QLabel *cloud_selection_label;
    QListWidget *cloud_selection_listWidget;
    QPushButton *add_cloud_pushButton;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString("MainWindow"));
        MainWindow->resize(1387, 862);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString("centralwidget"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(centralwidget->sizePolicy().hasHeightForWidth());
        centralwidget->setSizePolicy(sizePolicy);
        centralwidget->setMinimumSize(QSize(1387, 813));
        horizontalLayout_3 = new QHBoxLayout(centralwidget);
        horizontalLayout_3->setObjectName(QString("horizontalLayout_3"));
        main_horizontalLayout = new QHBoxLayout();
        main_horizontalLayout->setObjectName(QString("main_horizontalLayout"));
        main_horizontalLayout->setSizeConstraint(QLayout::SetDefaultConstraint);
        settings_verticalLayout = new QVBoxLayout();
        settings_verticalLayout->setObjectName(QString("settings_verticalLayout"));
        visualization_gridLayout_ = new QGridLayout();
        visualization_gridLayout_->setObjectName(QString("visualization_gridLayout_"));
        normals_length_label = new QLabel(centralwidget);
        normals_length_label->setObjectName(QString("normals_length_label"));

        visualization_gridLayout_->addWidget(normals_length_label, 3, 0, 1, 1);

        normals_length_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        normals_length_doubleSpinBox->setObjectName(QString("normals_length_doubleSpinBox"));
        normals_length_doubleSpinBox->setDecimals(2);
        normals_length_doubleSpinBox->setMaximum(9.99);
        normals_length_doubleSpinBox->setSingleStep(0.01);

        visualization_gridLayout_->addWidget(normals_length_doubleSpinBox, 3, 1, 1, 1);

        step_spinBox = new QSpinBox(centralwidget);
        step_spinBox->setObjectName(QString("step_spinBox"));
        step_spinBox->setValue(1);

        visualization_gridLayout_->addWidget(step_spinBox, 1, 1, 1, 1);

        step_label = new QLabel(centralwidget);
        step_label->setObjectName(QString("step_label"));

        visualization_gridLayout_->addWidget(step_label, 1, 0, 1, 1);

        points_label = new QLabel(centralwidget);
        points_label->setObjectName(QString("points_label"));

        visualization_gridLayout_->addWidget(points_label, 1, 2, 1, 1);

        points_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        points_doubleSpinBox->setObjectName(QString("points_doubleSpinBox"));
        points_doubleSpinBox->setDecimals(2);
        points_doubleSpinBox->setMaximum(9.99);
        points_doubleSpinBox->setSingleStep(0.01);
        points_doubleSpinBox->setValue(1);

        visualization_gridLayout_->addWidget(points_doubleSpinBox, 1, 3, 1, 1);

        vosualization_options_label = new QLabel(centralwidget);
        vosualization_options_label->setObjectName(QString("vosualization_options_label"));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        vosualization_options_label->setFont(font);
        vosualization_options_label->setAlignment(Qt::AlignCenter);

        visualization_gridLayout_->addWidget(vosualization_options_label, 0, 0, 1, 4);

        correspondences_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        correspondences_doubleSpinBox->setObjectName(QString("correspondences_doubleSpinBox"));
        correspondences_doubleSpinBox->setDecimals(2);
        correspondences_doubleSpinBox->setMaximum(9.99);
        correspondences_doubleSpinBox->setSingleStep(0.01);

        visualization_gridLayout_->addWidget(correspondences_doubleSpinBox, 5, 3, 1, 1);

        correspondences_label = new QLabel(centralwidget);
        correspondences_label->setObjectName(QString("correspondences_label"));

        visualization_gridLayout_->addWidget(correspondences_label, 5, 2, 1, 1);

        covariances_label = new QLabel(centralwidget);
        covariances_label->setObjectName(QString("covariances_label"));

        visualization_gridLayout_->addWidget(covariances_label, 5, 0, 1, 1);

        covariances_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        covariances_doubleSpinBox->setObjectName(QString("covariances_doubleSpinBox"));
        covariances_doubleSpinBox->setDecimals(2);
        covariances_doubleSpinBox->setMaximum(9.99);
        covariances_doubleSpinBox->setSingleStep(0.01);

        visualization_gridLayout_->addWidget(covariances_doubleSpinBox, 5, 1, 1, 1);

        normals_width_label = new QLabel(centralwidget);
        normals_width_label->setObjectName(QString("normals_width_label"));

        visualization_gridLayout_->addWidget(normals_width_label, 3, 2, 1, 1);

        normals_width_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        normals_width_doubleSpinBox->setObjectName(QString("normals_width_doubleSpinBox"));
        normals_width_doubleSpinBox->setDecimals(2);
        normals_width_doubleSpinBox->setMaximum(9.99);
        normals_width_doubleSpinBox->setSingleStep(0.01);
        normals_width_doubleSpinBox->setValue(1);

        visualization_gridLayout_->addWidget(normals_width_doubleSpinBox, 3, 3, 1, 1);


        settings_verticalLayout->addLayout(visualization_gridLayout_);

        line = new QFrame(centralwidget);
        line->setObjectName(QString("line"));
        line->setFrameShadow(QFrame::Plain);
        line->setFrameShape(QFrame::HLine);

        settings_verticalLayout->addWidget(line);

        stats_gridLayout = new QGridLayout();
        stats_gridLayout->setObjectName(QString("stats_gridLayout"));
        curv_threshold_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        curv_threshold_doubleSpinBox->setObjectName(QString("curv_threshold_doubleSpinBox"));
        curv_threshold_doubleSpinBox->setMaximum(1);
        curv_threshold_doubleSpinBox->setSingleStep(0.01);
        curv_threshold_doubleSpinBox->setValue(0.2);

        stats_gridLayout->addWidget(curv_threshold_doubleSpinBox, 5, 1, 1, 1);

        max_image_radius_label = new QLabel(centralwidget);
        max_image_radius_label->setObjectName(QString("max_image_radius_label"));

        stats_gridLayout->addWidget(max_image_radius_label, 1, 2, 1, 1);

        min_image_radius_label = new QLabel(centralwidget);
        min_image_radius_label->setObjectName(QString("min_image_radius_label"));

        stats_gridLayout->addWidget(min_image_radius_label, 1, 0, 1, 1);

        min_image_radius_spinBox = new QSpinBox(centralwidget);
        min_image_radius_spinBox->setObjectName(QString("min_image_radius_spinBox"));
        min_image_radius_spinBox->setMinimum(1);
        min_image_radius_spinBox->setMaximum(999);
        min_image_radius_spinBox->setValue(10);

        stats_gridLayout->addWidget(min_image_radius_spinBox, 1, 1, 1, 1);

        max_image_radius_spinBox = new QSpinBox(centralwidget);
        max_image_radius_spinBox->setObjectName(QString("max_image_radius_spinBox"));
        max_image_radius_spinBox->setMinimum(1);
        max_image_radius_spinBox->setMaximum(999);
        max_image_radius_spinBox->setValue(30);

        stats_gridLayout->addWidget(max_image_radius_spinBox, 1, 3, 1, 1);

        world_radius_label = new QLabel(centralwidget);
        world_radius_label->setObjectName(QString("world_radius_label"));

        stats_gridLayout->addWidget(world_radius_label, 3, 2, 1, 1);

        world_radius_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        world_radius_doubleSpinBox->setObjectName(QString("world_radius_doubleSpinBox"));
        world_radius_doubleSpinBox->setDecimals(2);
        world_radius_doubleSpinBox->setMinimum(0.01);
        world_radius_doubleSpinBox->setSingleStep(0.01);
        world_radius_doubleSpinBox->setValue(0.1);

        stats_gridLayout->addWidget(world_radius_doubleSpinBox, 3, 3, 1, 1);

        min_points_label = new QLabel(centralwidget);
        min_points_label->setObjectName(QString("min_points_label"));

        stats_gridLayout->addWidget(min_points_label, 3, 0, 1, 1);

        stats_options_label = new QLabel(centralwidget);
        stats_options_label->setObjectName(QString("stats_options_label"));
        stats_options_label->setFont(font);
        stats_options_label->setAlignment(Qt::AlignCenter);

        stats_gridLayout->addWidget(stats_options_label, 0, 0, 1, 4);

        curv_threshold_label = new QLabel(centralwidget);
        curv_threshold_label->setObjectName(QString("curv_threshold_label"));

        stats_gridLayout->addWidget(curv_threshold_label, 5, 0, 1, 1);

        min_points_spinBox = new QSpinBox(centralwidget);
        min_points_spinBox->setObjectName(QString("min_points_spinBox"));
        min_points_spinBox->setMinimum(1);
        min_points_spinBox->setMaximum(99999);
        min_points_spinBox->setValue(50);

        stats_gridLayout->addWidget(min_points_spinBox, 3, 1, 1, 1);


        settings_verticalLayout->addLayout(stats_gridLayout);

        line_2 = new QFrame(centralwidget);
        line_2->setObjectName(QString("line_2"));
        line_2->setFrameShadow(QFrame::Plain);
        line_2->setFrameShape(QFrame::HLine);

        settings_verticalLayout->addWidget(line_2);

        correspondences_gridLayout = new QGridLayout();
        correspondences_gridLayout->setObjectName(QString("correspondences_gridLayout"));
        curva_flatness_label = new QLabel(centralwidget);
        curva_flatness_label->setObjectName(QString("curva_flatness_label"));

        correspondences_gridLayout->addWidget(curva_flatness_label, 1, 2, 1, 1);

        normal_angle_label = new QLabel(centralwidget);
        normal_angle_label->setObjectName(QString("normal_angle_label"));

        correspondences_gridLayout->addWidget(normal_angle_label, 1, 0, 1, 1);

        curv_ratio_label = new QLabel(centralwidget);
        curv_ratio_label->setObjectName(QString("curv_ratio_label"));

        correspondences_gridLayout->addWidget(curv_ratio_label, 3, 0, 1, 1);

        normal_angle_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        normal_angle_doubleSpinBox->setObjectName(QString("normal_angle_doubleSpinBox"));
        normal_angle_doubleSpinBox->setMaximum(3.14);
        normal_angle_doubleSpinBox->setSingleStep(0.01);
        normal_angle_doubleSpinBox->setValue(0.9);

        correspondences_gridLayout->addWidget(normal_angle_doubleSpinBox, 1, 1, 1, 1);

        curv_ratio_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        curv_ratio_doubleSpinBox->setObjectName(QString("curv_ratio_doubleSpinBox"));
        curv_ratio_doubleSpinBox->setMaximum(9.99);
        curv_ratio_doubleSpinBox->setSingleStep(0.01);
        curv_ratio_doubleSpinBox->setValue(1.3);

        correspondences_gridLayout->addWidget(curv_ratio_doubleSpinBox, 3, 1, 1, 1);

        curv_flatness_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        curv_flatness_doubleSpinBox->setObjectName(QString("curv_flatness_doubleSpinBox"));
        curv_flatness_doubleSpinBox->setMaximum(1);
        curv_flatness_doubleSpinBox->setSingleStep(0.01);
        curv_flatness_doubleSpinBox->setValue(0.02);

        correspondences_gridLayout->addWidget(curv_flatness_doubleSpinBox, 1, 3, 1, 1);

        point_distance_label = new QLabel(centralwidget);
        point_distance_label->setObjectName(QString("point_distance_label"));

        correspondences_gridLayout->addWidget(point_distance_label, 3, 2, 1, 1);

        point_distance_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        point_distance_doubleSpinBox->setObjectName(QString("point_distance_doubleSpinBox"));
        point_distance_doubleSpinBox->setMaximum(9.99);
        point_distance_doubleSpinBox->setSingleStep(0.01);
        point_distance_doubleSpinBox->setValue(1);

        correspondences_gridLayout->addWidget(point_distance_doubleSpinBox, 3, 3, 1, 1);

        correspondences_threshold_label = new QLabel(centralwidget);
        correspondences_threshold_label->setObjectName(QString("correspondences_threshold_label"));
        correspondences_threshold_label->setFont(font);
        correspondences_threshold_label->setAlignment(Qt::AlignCenter);

        correspondences_gridLayout->addWidget(correspondences_threshold_label, 0, 0, 1, 4);


        settings_verticalLayout->addLayout(correspondences_gridLayout);

        line_3 = new QFrame(centralwidget);
        line_3->setObjectName(QString("line_3"));
        line_3->setFrameShadow(QFrame::Plain);
        line_3->setFrameShape(QFrame::HLine);

        settings_verticalLayout->addWidget(line_3);

        aligner_gridLayout = new QGridLayout();
        aligner_gridLayout->setObjectName(QString("aligner_gridLayout"));
        min_inliers_label = new QLabel(centralwidget);
        min_inliers_label->setObjectName(QString("min_inliers_label"));

        aligner_gridLayout->addWidget(min_inliers_label, 3, 0, 1, 1);

        inner_iter_spinBox = new QSpinBox(centralwidget);
        inner_iter_spinBox->setObjectName(QString("inner_iter_spinBox"));
        inner_iter_spinBox->setMinimum(1);
        inner_iter_spinBox->setValue(1);

        aligner_gridLayout->addWidget(inner_iter_spinBox, 1, 1, 1, 1);

        inner_iter_label = new QLabel(centralwidget);
        inner_iter_label->setObjectName(QString("inner_iter_label"));

        aligner_gridLayout->addWidget(inner_iter_label, 1, 0, 1, 1);

        min_inliers_spinBox = new QSpinBox(centralwidget);
        min_inliers_spinBox->setObjectName(QString("min_inliers_spinBox"));
        min_inliers_spinBox->setMinimum(0);
        min_inliers_spinBox->setMaximum(999999);
        min_inliers_spinBox->setValue(10000);

        aligner_gridLayout->addWidget(min_inliers_spinBox, 3, 1, 1, 1);

        outer_iter_label = new QLabel(centralwidget);
        outer_iter_label->setObjectName(QString("outer_iter_label"));

        aligner_gridLayout->addWidget(outer_iter_label, 1, 2, 1, 1);

        outer_iter_spinBox = new QSpinBox(centralwidget);
        outer_iter_spinBox->setObjectName(QString("outer_iter_spinBox"));
        outer_iter_spinBox->setMinimum(1);
        outer_iter_spinBox->setValue(10);

        aligner_gridLayout->addWidget(outer_iter_spinBox, 1, 3, 1, 1);

        aligner_options_label = new QLabel(centralwidget);
        aligner_options_label->setObjectName(QString("aligner_options_label"));
        aligner_options_label->setFont(font);
        aligner_options_label->setAlignment(Qt::AlignCenter);

        aligner_gridLayout->addWidget(aligner_options_label, 0, 0, 1, 4);

        max_chi2_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        max_chi2_doubleSpinBox->setObjectName(QString("max_chi2_doubleSpinBox"));
        max_chi2_doubleSpinBox->setDecimals(2);
        max_chi2_doubleSpinBox->setMaximum(1e+06);
        max_chi2_doubleSpinBox->setSingleStep(0.01);
        max_chi2_doubleSpinBox->setValue(9000);

        aligner_gridLayout->addWidget(max_chi2_doubleSpinBox, 3, 3, 1, 1);

        max_chi2_label = new QLabel(centralwidget);
        max_chi2_label->setObjectName(QString("max_chi2_label"));

        aligner_gridLayout->addWidget(max_chi2_label, 3, 2, 1, 1);

        lambda_label = new QLabel(centralwidget);
        lambda_label->setObjectName(QString("lambda_label"));

        aligner_gridLayout->addWidget(lambda_label, 4, 0, 1, 1);

        lambda_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        lambda_doubleSpinBox->setObjectName(QString("lambda_doubleSpinBox"));
        lambda_doubleSpinBox->setDecimals(5);
        lambda_doubleSpinBox->setMaximum(100000);
        lambda_doubleSpinBox->setValue(1000);

        aligner_gridLayout->addWidget(lambda_doubleSpinBox, 4, 1, 1, 1);


        settings_verticalLayout->addLayout(aligner_gridLayout);

        line_4 = new QFrame(centralwidget);
        line_4->setObjectName(QString("line_4"));
        line_4->setFrameShadow(QFrame::Plain);
        line_4->setFrameShape(QFrame::HLine);

        settings_verticalLayout->addWidget(line_4);

        alignment_gridLayout = new QGridLayout();
        alignment_gridLayout->setObjectName(QString("alignment_gridLayout"));
        alignment_options_label = new QLabel(centralwidget);
        alignment_options_label->setObjectName(QString("alignment_options_label"));
        alignment_options_label->setFont(font);
        alignment_options_label->setAlignment(Qt::AlignCenter);

        alignment_gridLayout->addWidget(alignment_options_label, 0, 0, 1, 2);

        clear_last_pushButton = new QPushButton(centralwidget);
        clear_last_pushButton->setObjectName(QString("clear_last_pushButton"));

        alignment_gridLayout->addWidget(clear_last_pushButton, 1, 0, 1, 1);

        initial_guess_pushButton = new QPushButton(centralwidget);
        initial_guess_pushButton->setObjectName(QString("initial_guess_pushButton"));

        alignment_gridLayout->addWidget(initial_guess_pushButton, 3, 0, 1, 1);

        claer_all_pushButton = new QPushButton(centralwidget);
        claer_all_pushButton->setObjectName(QString("claer_all_pushButton"));

        alignment_gridLayout->addWidget(claer_all_pushButton, 1, 1, 1, 1);

        optimize_pushButton = new QPushButton(centralwidget);
        optimize_pushButton->setObjectName(QString("optimize_pushButton"));

        alignment_gridLayout->addWidget(optimize_pushButton, 3, 1, 1, 1);

        correspondences_pushButton = new QPushButton(centralwidget);
        correspondences_pushButton->setObjectName(QString("correspondences_pushButton"));

        alignment_gridLayout->addWidget(correspondences_pushButton, 2, 1, 1, 1);

        merge_pushButton = new QPushButton(centralwidget);
        merge_pushButton->setObjectName(QString("merge_pushButton"));

        alignment_gridLayout->addWidget(merge_pushButton, 4, 0, 1, 2);

        step_by_step_checkBox = new QCheckBox(centralwidget);
        step_by_step_checkBox->setObjectName(QString("step_by_step_checkBox"));

        alignment_gridLayout->addWidget(step_by_step_checkBox, 2, 0, 1, 1, Qt::AlignHCenter);


        settings_verticalLayout->addLayout(alignment_gridLayout);

        line_5 = new QFrame(centralwidget);
        line_5->setObjectName(QString("line_5"));
        line_5->setFrameShadow(QFrame::Plain);
        line_5->setFrameShape(QFrame::HLine);

        settings_verticalLayout->addWidget(line_5);

        saving_gridLayout = new QGridLayout();
        saving_gridLayout->setObjectName(QString("saving_gridLayout"));
        png_snapshot_pushButton = new QPushButton(centralwidget);
        png_snapshot_pushButton->setObjectName(QString("png_snapshot_pushButton"));

        saving_gridLayout->addWidget(png_snapshot_pushButton, 1, 0, 1, 1);

        nicp_snapshot_pushButton = new QPushButton(centralwidget);
        nicp_snapshot_pushButton->setObjectName(QString("nicp_snapshot_pushButton"));

        saving_gridLayout->addWidget(nicp_snapshot_pushButton, 1, 1, 1, 1);

        saving_options_label = new QLabel(centralwidget);
        saving_options_label->setObjectName(QString("saving_options_label"));
        saving_options_label->setFont(font);
        saving_options_label->setAlignment(Qt::AlignCenter);

        saving_gridLayout->addWidget(saving_options_label, 0, 0, 1, 2);


        settings_verticalLayout->addLayout(saving_gridLayout);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        settings_verticalLayout->addItem(verticalSpacer_2);


        main_horizontalLayout->addLayout(settings_verticalLayout);

        visualization_verticalLayout = new QVBoxLayout();
        visualization_verticalLayout->setObjectName(QString("visualization_verticalLayout"));
        viewer_horizontalLayout = new QHBoxLayout();
        viewer_horizontalLayout->setObjectName(QString("viewer_horizontalLayout"));
        viewer_verticalLayout = new QVBoxLayout();
        viewer_verticalLayout->setObjectName(QString("viewer_verticalLayout"));
        qglviewer_horizontalLayout = new QHBoxLayout();
        qglviewer_horizontalLayout->setObjectName(QString("qglviewer_horizontalLayout"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString("verticalLayout_3"));
        viewer_label = new QLabel(centralwidget);
        viewer_label->setObjectName(QString("viewer_label"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(viewer_label->sizePolicy().hasHeightForWidth());
        viewer_label->setSizePolicy(sizePolicy1);
        viewer_label->setFont(font);
        viewer_label->setAlignment(Qt::AlignCenter);

        verticalLayout_3->addWidget(viewer_label);

        viewer = new nicp_viewer::NICPQGLViewer(centralwidget);
        viewer->setObjectName(QString("viewer"));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(viewer->sizePolicy().hasHeightForWidth());
        viewer->setSizePolicy(sizePolicy2);

        verticalLayout_3->addWidget(viewer);


        qglviewer_horizontalLayout->addLayout(verticalLayout_3);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString("gridLayout"));
        reference_graphicsView = new QGraphicsView(centralwidget);
        reference_graphicsView->setObjectName(QString("reference_graphicsView"));
        QSizePolicy sizePolicy3(QSizePolicy::Minimum, QSizePolicy::Expanding);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(reference_graphicsView->sizePolicy().hasHeightForWidth());
        reference_graphicsView->setSizePolicy(sizePolicy3);
        reference_graphicsView->setMinimumSize(QSize(320, 240));
        reference_graphicsView->setAutoFillBackground(false);
        QBrush brush(QColor(0, 0, 0, 255));
        brush.setStyle(Qt::SolidPattern);
        reference_graphicsView->setBackgroundBrush(brush);

        gridLayout->addWidget(reference_graphicsView, 2, 0, 1, 1);

        current_graphicsView = new QGraphicsView(centralwidget);
        current_graphicsView->setObjectName(QString("current_graphicsView"));
        sizePolicy3.setHeightForWidth(current_graphicsView->sizePolicy().hasHeightForWidth());
        current_graphicsView->setSizePolicy(sizePolicy3);
        current_graphicsView->setMinimumSize(QSize(320, 240));
        current_graphicsView->setBackgroundBrush(brush);

        gridLayout->addWidget(current_graphicsView, 5, 0, 1, 1);

        reference_depth_image_label = new QLabel(centralwidget);
        reference_depth_image_label->setObjectName(QString("reference_depth_image_label"));
        QSizePolicy sizePolicy4(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(reference_depth_image_label->sizePolicy().hasHeightForWidth());
        reference_depth_image_label->setSizePolicy(sizePolicy4);
        reference_depth_image_label->setFont(font);
        reference_depth_image_label->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(reference_depth_image_label, 1, 0, 1, 1);

        reference_depth_image_label_2 = new QLabel(centralwidget);
        reference_depth_image_label_2->setObjectName(QString("reference_depth_image_label_2"));
        sizePolicy4.setHeightForWidth(reference_depth_image_label_2->sizePolicy().hasHeightForWidth());
        reference_depth_image_label_2->setSizePolicy(sizePolicy4);
        reference_depth_image_label_2->setFont(font);
        reference_depth_image_label_2->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(reference_depth_image_label_2, 3, 0, 1, 1);


        qglviewer_horizontalLayout->addLayout(gridLayout);


        viewer_verticalLayout->addLayout(qglviewer_horizontalLayout);

        depth_images_horizontalLayout = new QHBoxLayout();
        depth_images_horizontalLayout->setObjectName(QString("depth_images_horizontalLayout"));
        reference_depth_image_verticalLayout = new QVBoxLayout();
        reference_depth_image_verticalLayout->setObjectName(QString("reference_depth_image_verticalLayout"));

        depth_images_horizontalLayout->addLayout(reference_depth_image_verticalLayout);

        current_depth_image_verticalLayout = new QVBoxLayout();
        current_depth_image_verticalLayout->setObjectName(QString("current_depth_image_verticalLayout"));

        depth_images_horizontalLayout->addLayout(current_depth_image_verticalLayout);


        viewer_verticalLayout->addLayout(depth_images_horizontalLayout);


        viewer_horizontalLayout->addLayout(viewer_verticalLayout);


        visualization_verticalLayout->addLayout(viewer_horizontalLayout);

        projectors_horizontalLayout = new QHBoxLayout();
        projectors_horizontalLayout->setObjectName(QString("projectors_horizontalLayout"));
        point_projector_gridLayout = new QGridLayout();
        point_projector_gridLayout->setObjectName(QString("point_projector_gridLayout"));
        point_projector_gridLayout->setSizeConstraint(QLayout::SetDefaultConstraint);
        cols_label = new QLabel(centralwidget);
        cols_label->setObjectName(QString("cols_label"));
        QSizePolicy sizePolicy5(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy5.setHorizontalStretch(0);
        sizePolicy5.setVerticalStretch(0);
        sizePolicy5.setHeightForWidth(cols_label->sizePolicy().hasHeightForWidth());
        cols_label->setSizePolicy(sizePolicy5);

        point_projector_gridLayout->addWidget(cols_label, 2, 0, 1, 1);

        max_distance_label = new QLabel(centralwidget);
        max_distance_label->setObjectName(QString("max_distance_label"));
        sizePolicy5.setHeightForWidth(max_distance_label->sizePolicy().hasHeightForWidth());
        max_distance_label->setSizePolicy(sizePolicy5);

        point_projector_gridLayout->addWidget(max_distance_label, 4, 0, 1, 1);

        min_distance_label = new QLabel(centralwidget);
        min_distance_label->setObjectName(QString("min_distance_label"));
        sizePolicy5.setHeightForWidth(min_distance_label->sizePolicy().hasHeightForWidth());
        min_distance_label->setSizePolicy(sizePolicy5);

        point_projector_gridLayout->addWidget(min_distance_label, 3, 0, 1, 1);

        rows_label = new QLabel(centralwidget);
        rows_label->setObjectName(QString("rows_label"));
        sizePolicy5.setHeightForWidth(rows_label->sizePolicy().hasHeightForWidth());
        rows_label->setSizePolicy(sizePolicy5);

        point_projector_gridLayout->addWidget(rows_label, 1, 0, 1, 1);

        max_distance_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        max_distance_doubleSpinBox->setObjectName(QString("max_distance_doubleSpinBox"));
        max_distance_doubleSpinBox->setDecimals(5);
        max_distance_doubleSpinBox->setMaximum(999.99);
        max_distance_doubleSpinBox->setSingleStep(0.001);
        max_distance_doubleSpinBox->setValue(4.5);

        point_projector_gridLayout->addWidget(max_distance_doubleSpinBox, 4, 1, 1, 1);

        verticalSpacer_8 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        point_projector_gridLayout->addItem(verticalSpacer_8, 7, 0, 1, 1);

        cols_spinBox = new QSpinBox(centralwidget);
        cols_spinBox->setObjectName(QString("cols_spinBox"));
        cols_spinBox->setMinimum(1);
        cols_spinBox->setMaximum(9999);
        cols_spinBox->setValue(640);

        point_projector_gridLayout->addWidget(cols_spinBox, 2, 1, 1, 1);

        min_distance_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        min_distance_doubleSpinBox->setObjectName(QString("min_distance_doubleSpinBox"));
        min_distance_doubleSpinBox->setDecimals(5);
        min_distance_doubleSpinBox->setMaximum(999.99);
        min_distance_doubleSpinBox->setSingleStep(0.001);
        min_distance_doubleSpinBox->setValue(0.5);

        point_projector_gridLayout->addWidget(min_distance_doubleSpinBox, 3, 1, 1, 1);

        point_projector_options_label = new QLabel(centralwidget);
        point_projector_options_label->setObjectName(QString("point_projector_options_label"));
        sizePolicy5.setHeightForWidth(point_projector_options_label->sizePolicy().hasHeightForWidth());
        point_projector_options_label->setSizePolicy(sizePolicy5);
        point_projector_options_label->setFont(font);
        point_projector_options_label->setAlignment(Qt::AlignCenter);

        point_projector_gridLayout->addWidget(point_projector_options_label, 0, 0, 1, 2);

        verticalSpacer_6 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        point_projector_gridLayout->addItem(verticalSpacer_6, 7, 1, 1, 1);

        rows_spinBox = new QSpinBox(centralwidget);
        rows_spinBox->setObjectName(QString("rows_spinBox"));
        rows_spinBox->setMinimum(1);
        rows_spinBox->setMaximum(9999);
        rows_spinBox->setValue(480);

        point_projector_gridLayout->addWidget(rows_spinBox, 1, 1, 1, 1);

        scale_label = new QLabel(centralwidget);
        scale_label->setObjectName(QString("scale_label"));

        point_projector_gridLayout->addWidget(scale_label, 5, 0, 1, 1);

        scale_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        scale_doubleSpinBox->setObjectName(QString("scale_doubleSpinBox"));
        scale_doubleSpinBox->setDecimals(5);
        scale_doubleSpinBox->setMinimum(1);
        scale_doubleSpinBox->setMaximum(99);
        scale_doubleSpinBox->setSingleStep(0.01);
        scale_doubleSpinBox->setValue(1);

        point_projector_gridLayout->addWidget(scale_doubleSpinBox, 5, 1, 1, 1);

        image_scale_label = new QLabel(centralwidget);
        image_scale_label->setObjectName(QString("image_scale_label"));

        point_projector_gridLayout->addWidget(image_scale_label, 6, 0, 1, 1);

        image_scale_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        image_scale_doubleSpinBox->setObjectName(QString("image_scale_doubleSpinBox"));
        image_scale_doubleSpinBox->setDecimals(5);
        image_scale_doubleSpinBox->setMinimum(0);
        image_scale_doubleSpinBox->setMaximum(100000);
        image_scale_doubleSpinBox->setValue(0.001);

        point_projector_gridLayout->addWidget(image_scale_doubleSpinBox, 6, 1, 1, 1);


        projectors_horizontalLayout->addLayout(point_projector_gridLayout);

        pinhole_gridLayout = new QGridLayout();
        pinhole_gridLayout->setObjectName(QString("pinhole_gridLayout"));
        cx_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        cx_doubleSpinBox->setObjectName(QString("cx_doubleSpinBox"));
        cx_doubleSpinBox->setDecimals(5);
        cx_doubleSpinBox->setMinimum(0.01);
        cx_doubleSpinBox->setMaximum(9999.99);
        cx_doubleSpinBox->setSingleStep(0.01);
        cx_doubleSpinBox->setValue(319.5);

        pinhole_gridLayout->addWidget(cx_doubleSpinBox, 3, 1, 1, 1);

        fy_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        fy_doubleSpinBox->setObjectName(QString("fy_doubleSpinBox"));
        fy_doubleSpinBox->setDecimals(5);
        fy_doubleSpinBox->setMinimum(0.01);
        fy_doubleSpinBox->setMaximum(9999.99);
        fy_doubleSpinBox->setSingleStep(0.01);
        fy_doubleSpinBox->setValue(525);

        pinhole_gridLayout->addWidget(fy_doubleSpinBox, 2, 1, 1, 1);

        fx_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        fx_doubleSpinBox->setObjectName(QString("fx_doubleSpinBox"));
        fx_doubleSpinBox->setDecimals(5);
        fx_doubleSpinBox->setMinimum(0.01);
        fx_doubleSpinBox->setMaximum(9999.99);
        fx_doubleSpinBox->setSingleStep(0.01);
        fx_doubleSpinBox->setValue(525);

        pinhole_gridLayout->addWidget(fx_doubleSpinBox, 1, 1, 1, 1);

        cy_label = new QLabel(centralwidget);
        cy_label->setObjectName(QString("cy_label"));
        sizePolicy5.setHeightForWidth(cy_label->sizePolicy().hasHeightForWidth());
        cy_label->setSizePolicy(sizePolicy5);

        pinhole_gridLayout->addWidget(cy_label, 4, 0, 1, 1);

        baseline_label = new QLabel(centralwidget);
        baseline_label->setObjectName(QString("baseline_label"));
        sizePolicy5.setHeightForWidth(baseline_label->sizePolicy().hasHeightForWidth());
        baseline_label->setSizePolicy(sizePolicy5);

        pinhole_gridLayout->addWidget(baseline_label, 5, 0, 1, 1);

        fy_label = new QLabel(centralwidget);
        fy_label->setObjectName(QString("fy_label"));
        sizePolicy5.setHeightForWidth(fy_label->sizePolicy().hasHeightForWidth());
        fy_label->setSizePolicy(sizePolicy5);

        pinhole_gridLayout->addWidget(fy_label, 2, 0, 1, 1);

        cy_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        cy_doubleSpinBox->setObjectName(QString("cy_doubleSpinBox"));
        cy_doubleSpinBox->setDecimals(5);
        cy_doubleSpinBox->setMinimum(0.01);
        cy_doubleSpinBox->setMaximum(9999.99);
        cy_doubleSpinBox->setSingleStep(0.01);
        cy_doubleSpinBox->setValue(239.5);

        pinhole_gridLayout->addWidget(cy_doubleSpinBox, 4, 1, 1, 1);

        baseline_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        baseline_doubleSpinBox->setObjectName(QString("baseline_doubleSpinBox"));
        baseline_doubleSpinBox->setDecimals(5);
        baseline_doubleSpinBox->setMinimum(0.001);
        baseline_doubleSpinBox->setMaximum(9999.99);
        baseline_doubleSpinBox->setSingleStep(0.001);
        baseline_doubleSpinBox->setValue(0.075);

        pinhole_gridLayout->addWidget(baseline_doubleSpinBox, 5, 1, 1, 1);

        fx_label = new QLabel(centralwidget);
        fx_label->setObjectName(QString("fx_label"));
        sizePolicy5.setHeightForWidth(fx_label->sizePolicy().hasHeightForWidth());
        fx_label->setSizePolicy(sizePolicy5);
        fx_label->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        pinhole_gridLayout->addWidget(fx_label, 1, 0, 1, 1);

        pinhole_projector_radioButton = new QRadioButton(centralwidget);
        pinhole_projector_radioButton->setObjectName(QString("pinhole_projector_radioButton"));
        pinhole_projector_radioButton->setChecked(true);

        pinhole_gridLayout->addWidget(pinhole_projector_radioButton, 0, 0, 1, 2, Qt::AlignHCenter);

        cx_label = new QLabel(centralwidget);
        cx_label->setObjectName(QString("cx_label"));
        sizePolicy5.setHeightForWidth(cx_label->sizePolicy().hasHeightForWidth());
        cx_label->setSizePolicy(sizePolicy5);

        pinhole_gridLayout->addWidget(cx_label, 3, 0, 1, 1);

        alpha_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        alpha_doubleSpinBox->setObjectName(QString("alpha_doubleSpinBox"));
        alpha_doubleSpinBox->setDecimals(5);
        alpha_doubleSpinBox->setMinimum(0.001);
        alpha_doubleSpinBox->setMaximum(9999.99);
        alpha_doubleSpinBox->setSingleStep(0.001);
        alpha_doubleSpinBox->setValue(0.1);

        pinhole_gridLayout->addWidget(alpha_doubleSpinBox, 6, 1, 1, 1);

        alpha_label = new QLabel(centralwidget);
        alpha_label->setObjectName(QString("alpha_label"));
        sizePolicy5.setHeightForWidth(alpha_label->sizePolicy().hasHeightForWidth());
        alpha_label->setSizePolicy(sizePolicy5);

        pinhole_gridLayout->addWidget(alpha_label, 6, 0, 1, 1);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        pinhole_gridLayout->addItem(verticalSpacer_3, 7, 0, 1, 1);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        pinhole_gridLayout->addItem(verticalSpacer, 7, 1, 1, 1);


        projectors_horizontalLayout->addLayout(pinhole_gridLayout);

        spherical_gridLayout = new QGridLayout();
        spherical_gridLayout->setObjectName(QString("spherical_gridLayout"));
        sph_horizontal_fov_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        sph_horizontal_fov_doubleSpinBox->setObjectName(QString("sph_horizontal_fov_doubleSpinBox"));
        sph_horizontal_fov_doubleSpinBox->setDecimals(5);
        sph_horizontal_fov_doubleSpinBox->setMinimum(0.01);
        sph_horizontal_fov_doubleSpinBox->setMaximum(3.14);
        sph_horizontal_fov_doubleSpinBox->setSingleStep(0.01);
        sph_horizontal_fov_doubleSpinBox->setValue(3.14);

        spherical_gridLayout->addWidget(sph_horizontal_fov_doubleSpinBox, 1, 1, 1, 1);

        sph_horizontal_fov_label = new QLabel(centralwidget);
        sph_horizontal_fov_label->setObjectName(QString("sph_horizontal_fov_label"));
        sizePolicy5.setHeightForWidth(sph_horizontal_fov_label->sizePolicy().hasHeightForWidth());
        sph_horizontal_fov_label->setSizePolicy(sizePolicy5);

        spherical_gridLayout->addWidget(sph_horizontal_fov_label, 1, 0, 1, 1);

        sph_vertical_fov_label = new QLabel(centralwidget);
        sph_vertical_fov_label->setObjectName(QString("sph_vertical_fov_label"));
        sizePolicy5.setHeightForWidth(sph_vertical_fov_label->sizePolicy().hasHeightForWidth());
        sph_vertical_fov_label->setSizePolicy(sizePolicy5);

        spherical_gridLayout->addWidget(sph_vertical_fov_label, 2, 0, 1, 1);

        sph_vertical_fov_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        sph_vertical_fov_doubleSpinBox->setObjectName(QString("sph_vertical_fov_doubleSpinBox"));
        sph_vertical_fov_doubleSpinBox->setDecimals(5);
        sph_vertical_fov_doubleSpinBox->setMinimum(0.01);
        sph_vertical_fov_doubleSpinBox->setMaximum(1.57);
        sph_vertical_fov_doubleSpinBox->setSingleStep(0.01);
        sph_vertical_fov_doubleSpinBox->setValue(0.78);

        spherical_gridLayout->addWidget(sph_vertical_fov_doubleSpinBox, 2, 1, 1, 1);

        spherical_projector_radioButton = new QRadioButton(centralwidget);
        spherical_projector_radioButton->setObjectName(QString("spherical_projector_radioButton"));

        spherical_gridLayout->addWidget(spherical_projector_radioButton, 0, 0, 1, 2, Qt::AlignHCenter);

        verticalSpacer_10 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        spherical_gridLayout->addItem(verticalSpacer_10, 5, 1, 1, 1);

        verticalSpacer_7 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        spherical_gridLayout->addItem(verticalSpacer_7, 5, 0, 1, 1);

        sph_vertical_center_label = new QLabel(centralwidget);
        sph_vertical_center_label->setObjectName(QString("sph_vertical_center_label"));

        spherical_gridLayout->addWidget(sph_vertical_center_label, 3, 0, 1, 1);

        sph_horizontal_center_label = new QLabel(centralwidget);
        sph_horizontal_center_label->setObjectName(QString("sph_horizontal_center_label"));

        spherical_gridLayout->addWidget(sph_horizontal_center_label, 4, 0, 1, 1);

        sph_vertical_center_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        sph_vertical_center_doubleSpinBox->setObjectName(QString("sph_vertical_center_doubleSpinBox"));
        sph_vertical_center_doubleSpinBox->setDecimals(5);
        sph_vertical_center_doubleSpinBox->setMaximum(9999.99);
        sph_vertical_center_doubleSpinBox->setValue(240);

        spherical_gridLayout->addWidget(sph_vertical_center_doubleSpinBox, 3, 1, 1, 1);

        sph_horizontal_center_doubleSpinBox = new QDoubleSpinBox(centralwidget);
        sph_horizontal_center_doubleSpinBox->setObjectName(QString("sph_horizontal_center_doubleSpinBox"));
        sph_horizontal_center_doubleSpinBox->setDecimals(5);
        sph_horizontal_center_doubleSpinBox->setMaximum(9999.99);
        sph_horizontal_center_doubleSpinBox->setValue(320);

        spherical_gridLayout->addWidget(sph_horizontal_center_doubleSpinBox, 4, 1, 1, 1);


        projectors_horizontalLayout->addLayout(spherical_gridLayout);

        cloud_selection_verticalLayout = new QVBoxLayout();
        cloud_selection_verticalLayout->setObjectName(QString("cloud_selection_verticalLayout"));
        cloud_selection_label = new QLabel(centralwidget);
        cloud_selection_label->setObjectName(QString("cloud_selection_label"));
        QSizePolicy sizePolicy6(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy6.setHorizontalStretch(0);
        sizePolicy6.setVerticalStretch(0);
        sizePolicy6.setHeightForWidth(cloud_selection_label->sizePolicy().hasHeightForWidth());
        cloud_selection_label->setSizePolicy(sizePolicy6);
        cloud_selection_label->setFont(font);
        cloud_selection_label->setAlignment(Qt::AlignCenter);

        cloud_selection_verticalLayout->addWidget(cloud_selection_label);

        cloud_selection_listWidget = new QListWidget(centralwidget);
        cloud_selection_listWidget->setObjectName(QString("cloud_selection_listWidget"));
        sizePolicy3.setHeightForWidth(cloud_selection_listWidget->sizePolicy().hasHeightForWidth());
        cloud_selection_listWidget->setSizePolicy(sizePolicy3);
        cloud_selection_listWidget->setMinimumSize(QSize(320, 0));

        cloud_selection_verticalLayout->addWidget(cloud_selection_listWidget);

        add_cloud_pushButton = new QPushButton(centralwidget);
        add_cloud_pushButton->setObjectName(QString("add_cloud_pushButton"));
        sizePolicy6.setHeightForWidth(add_cloud_pushButton->sizePolicy().hasHeightForWidth());
        add_cloud_pushButton->setSizePolicy(sizePolicy6);

        cloud_selection_verticalLayout->addWidget(add_cloud_pushButton);

        cloud_selection_verticalLayout->setStretch(2, 1);

        projectors_horizontalLayout->addLayout(cloud_selection_verticalLayout);


        visualization_verticalLayout->addLayout(projectors_horizontalLayout);

        visualization_verticalLayout->setStretch(0, 1);

        main_horizontalLayout->addLayout(visualization_verticalLayout);

        main_horizontalLayout->setStretch(1, 1);

        horizontalLayout_3->addLayout(main_horizontalLayout);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString("menubar"));
        menubar->setGeometry(QRect(0, 0, 1387, 27));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);
        QObject::connect(step_spinBox, SIGNAL(valueChanged(int)), MainWindow, SLOT(visualizationUpdate()));
        QObject::connect(points_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(visualizationUpdate()));
        QObject::connect(covariances_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(visualizationUpdate()));
        QObject::connect(normals_length_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(visualizationUpdate()));
        QObject::connect(correspondences_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(visualizationUpdate()));
        QObject::connect(min_image_radius_spinBox, SIGNAL(valueChanged(int)), MainWindow, SLOT(statsUpdate()));
        QObject::connect(max_image_radius_spinBox, SIGNAL(valueChanged(int)), MainWindow, SLOT(statsUpdate()));
        QObject::connect(world_radius_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(statsUpdate()));
        QObject::connect(curv_threshold_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(statsUpdate()));
        QObject::connect(min_points_spinBox, SIGNAL(valueChanged(int)), MainWindow, SLOT(statsUpdate()));
        QObject::connect(normal_angle_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(correspondencesUpdate()));
        QObject::connect(curv_ratio_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(correspondencesUpdate()));
        QObject::connect(curv_flatness_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(correspondencesUpdate()));
        QObject::connect(point_distance_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(correspondencesUpdate()));
        QObject::connect(inner_iter_spinBox, SIGNAL(valueChanged(int)), MainWindow, SLOT(alignerUpdate()));
        QObject::connect(outer_iter_spinBox, SIGNAL(valueChanged(int)), MainWindow, SLOT(alignerUpdate()));
        QObject::connect(min_inliers_spinBox, SIGNAL(valueChanged(int)), MainWindow, SLOT(alignerUpdate()));
        QObject::connect(max_chi2_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(alignerUpdate()));
        QObject::connect(rows_spinBox, SIGNAL(valueChanged(int)), MainWindow, SLOT(projectorsUpdate()));
        QObject::connect(cols_spinBox, SIGNAL(valueChanged(int)), MainWindow, SLOT(projectorsUpdate()));
        QObject::connect(min_distance_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(projectorsUpdate()));
        QObject::connect(max_distance_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(projectorsUpdate()));
        QObject::connect(fx_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(projectorsUpdate()));
        QObject::connect(fy_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(projectorsUpdate()));
        QObject::connect(cx_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(projectorsUpdate()));
        QObject::connect(cy_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(projectorsUpdate()));
        QObject::connect(baseline_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(projectorsUpdate()));
        QObject::connect(alpha_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(projectorsUpdate()));
        QObject::connect(sph_horizontal_fov_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(projectorsUpdate()));
        QObject::connect(sph_vertical_fov_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(projectorsUpdate()));
        QObject::connect(add_cloud_pushButton, SIGNAL(clicked()), MainWindow, SLOT(addCloud()));
        QObject::connect(nicp_snapshot_pushButton, SIGNAL(clicked()), MainWindow, SLOT(nicpSnapshot()));
        QObject::connect(png_snapshot_pushButton, SIGNAL(clicked()), MainWindow, SLOT(jpgSnapshot()));
        QObject::connect(clear_last_pushButton, SIGNAL(clicked()), MainWindow, SLOT(clearLast()));
        QObject::connect(claer_all_pushButton, SIGNAL(clicked()), MainWindow, SLOT(clearAll()));
        QObject::connect(correspondences_pushButton, SIGNAL(clicked()), MainWindow, SLOT(correspondences()));
        QObject::connect(initial_guess_pushButton, SIGNAL(clicked()), MainWindow, SLOT(initialGuess()));
        QObject::connect(optimize_pushButton, SIGNAL(clicked()), MainWindow, SLOT(optimize()));
        QObject::connect(merge_pushButton, SIGNAL(clicked()), MainWindow, SLOT(merge()));
        QObject::connect(normals_width_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(visualizationUpdate()));
        QObject::connect(scale_doubleSpinBox, SIGNAL(valueChanged(double)), MainWindow, SLOT(projectorsUpdate()));

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        normals_length_label->setText(QApplication::translate("MainWindow", "Normals Length", 0));
        step_label->setText(QApplication::translate("MainWindow", "Step", 0));
        points_label->setText(QApplication::translate("MainWindow", "Points", 0));
        vosualization_options_label->setText(QApplication::translate("MainWindow", "Visualization Options", 0));
        correspondences_label->setText(QApplication::translate("MainWindow", "Correspondences", 0));
        covariances_label->setText(QApplication::translate("MainWindow", "Covariances", 0));
        normals_width_label->setText(QApplication::translate("MainWindow", "Normals Width", 0));
        max_image_radius_label->setText(QApplication::translate("MainWindow", "Max Image Radius", 0));
        min_image_radius_label->setText(QApplication::translate("MainWindow", "Min Image Radius", 0));
        world_radius_label->setText(QApplication::translate("MainWindow", "World Radius", 0));
        min_points_label->setText(QApplication::translate("MainWindow", "Min Points", 0));
        stats_options_label->setText(QApplication::translate("MainWindow", "Stats Options", 0));
        curv_threshold_label->setText(QApplication::translate("MainWindow", "Curv. Threshold", 0));
        curva_flatness_label->setText(QApplication::translate("MainWindow", "Curv. Flatness", 0));
        normal_angle_label->setText(QApplication::translate("MainWindow", "Normal Angle", 0));
        curv_ratio_label->setText(QApplication::translate("MainWindow", "Curv. Ratio", 0));
        point_distance_label->setText(QApplication::translate("MainWindow", "Point Distance", 0));
        correspondences_threshold_label->setText(QApplication::translate("MainWindow", "Correspondences Thresholds", 0));
        min_inliers_label->setText(QApplication::translate("MainWindow", "Min Inliers", 0));
        inner_iter_label->setText(QApplication::translate("MainWindow", "Inner Iter.", 0));
        outer_iter_label->setText(QApplication::translate("MainWindow", "Outer Iter.", 0));
        aligner_options_label->setText(QApplication::translate("MainWindow", "Aligner Options", 0));
        max_chi2_label->setText(QApplication::translate("MainWindow", "Max Chi2", 0));
        lambda_label->setText(QApplication::translate("MainWindow", "Lambda", 0));
        alignment_options_label->setText(QApplication::translate("MainWindow", "Alignment Options", 0));
        clear_last_pushButton->setText(QApplication::translate("MainWindow", "Clear Last", 0));
        initial_guess_pushButton->setText(QApplication::translate("MainWindow", "Initial Guess", 0));
        claer_all_pushButton->setText(QApplication::translate("MainWindow", "Clear All", 0));
        optimize_pushButton->setText(QApplication::translate("MainWindow", "Optimize", 0));
        correspondences_pushButton->setText(QApplication::translate("MainWindow", "Correspondences", 0));
        merge_pushButton->setText(QApplication::translate("MainWindow", "Merge", 0));
        step_by_step_checkBox->setText(QApplication::translate("MainWindow", "Step-byStep", 0));
        png_snapshot_pushButton->setText(QApplication::translate("MainWindow", "JPG Sanpshot", 0));
        nicp_snapshot_pushButton->setText(QApplication::translate("MainWindow", "NICP Snapshot", 0));
        saving_options_label->setText(QApplication::translate("MainWindow", "Saving Options", 0));
        viewer_label->setText(QApplication::translate("MainWindow", "3D Viewer", 0));
        reference_depth_image_label->setText(QApplication::translate("MainWindow", "Reference Depth Image", 0));
        reference_depth_image_label_2->setText(QApplication::translate("MainWindow", "Current Depth Image", 0));
        cols_label->setText(QApplication::translate("MainWindow", "Cols", 0));
        max_distance_label->setText(QApplication::translate("MainWindow", "Max Distance", 0));
        min_distance_label->setText(QApplication::translate("MainWindow", "Min Distance", 0));
        rows_label->setText(QApplication::translate("MainWindow", "Rows", 0));
        point_projector_options_label->setText(QApplication::translate("MainWindow", "Point Projector Options", 0));
        scale_label->setText(QApplication::translate("MainWindow", "Projector Scale", 0));
        image_scale_label->setText(QApplication::translate("MainWindow", "Image Scale", 0));
        cy_label->setText(QApplication::translate("MainWindow", "cy", 0));
        baseline_label->setText(QApplication::translate("MainWindow", "Baseline", 0));
        fy_label->setText(QApplication::translate("MainWindow", "fy", 0));
        fx_label->setText(QApplication::translate("MainWindow", "fx", 0));
        pinhole_projector_radioButton->setText(QApplication::translate("MainWindow", "Pinhole Projector", 0));
        cx_label->setText(QApplication::translate("MainWindow", "cx", 0));
        alpha_label->setText(QApplication::translate("MainWindow", "Alpha", 0));
        sph_horizontal_fov_label->setText(QApplication::translate("MainWindow", "Horizontal FOV", 0));
        sph_vertical_fov_label->setText(QApplication::translate("MainWindow", "VerticalFOV", 0));
        spherical_projector_radioButton->setText(QApplication::translate("MainWindow", "Spherical Projector", 0));
        sph_vertical_center_label->setText(QApplication::translate("MainWindow", "Vertical Center", 0));
        sph_horizontal_center_label->setText(QApplication::translate("MainWindow", "Horizontal Center", 0));
        cloud_selection_label->setText(QApplication::translate("MainWindow", "Cloud Selection", 0));
        add_cloud_pushButton->setText(QApplication::translate("MainWindow", "Add Cloud", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_NICP_ALIGNER_GUI_UI_MAIN_WINDOW_H
