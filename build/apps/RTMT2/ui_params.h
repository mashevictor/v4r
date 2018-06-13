/********************************************************************************
** Form generated from reading UI file 'params.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PARAMS_H
#define UI_PARAMS_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Params
{
public:
    QTabWidget *Tracker;
    QWidget *sensorTab;
    QGroupBox *rgbGroupBox;
    QWidget *gridLayoutWidget_3;
    QGridLayout *rgbLayout;
    QLabel *fRgbLabel;
    QLineEdit *cuRgbEdit;
    QLineEdit *fuRgbEdit;
    QLineEdit *fvRgbEdit;
    QLabel *cRgbLabel;
    QLineEdit *cvRgbEdit;
    QGroupBox *groupBox_2;
    QLineEdit *editVGNfile;
    QPushButton *pushFindVGNfile;
    QPushButton *applyButton;
    QGroupBox *groupBox_3;
    QLineEdit *editCRFfile;
    QPushButton *pushFindCRFfile;
    QWidget *ioTab;
    QGroupBox *groupBox;
    QLineEdit *editRGBDPath;
    QPushButton *pushFindRGBDPath;
    QLineEdit *editModelName;
    QCheckBox *cb_store_cloud;
    QCheckBox *cb_store_views;
    QCheckBox *cb_store_mesh;
    QCheckBox *cb_store_tracker;
    QCheckBox *cb_store_tex_mesh;
    QWidget *tab_5;
    QLabel *label_14;
    QWidget *layoutWidget11_2;
    QFormLayout *formLayout_3;
    QLineEdit *roi_scale_xy;
    QLabel *label_15;
    QLineEdit *roi_scale_height;
    QLabel *label_16;
    QLineEdit *roi_offs;
    QLabel *label_17;
    QWidget *Modelling;
    QLabel *label_18;
    QLineEdit *voxel_grid_size;
    QLabel *label_19;
    QLineEdit *poisson_depth;
    QCheckBox *filter_largest_cluster;
    QLineEdit *poisson_samples;
    QLabel *label_20;
    QCheckBox *multiview_icp;
    QCheckBox *use_nguyen_noise_model;
    QLabel *label_21;
    QLineEdit *nb_frames_temp_filt;
    QLabel *label_22;
    QLabel *label_23;
    QLineEdit *min_cam_motion_kf;
    QLineEdit *nb_frames_ba;
    QLineEdit *min_cam_rot_kf;
    QLabel *min_cam_rot_kf_txt;
    QPushButton *okButton;

    void setupUi(QWidget *Params)
    {
        if (Params->objectName().isEmpty())
            Params->setObjectName(QStringLiteral("Params"));
        Params->resize(752, 469);
        Tracker = new QTabWidget(Params);
        Tracker->setObjectName(QStringLiteral("Tracker"));
        Tracker->setGeometry(QRect(10, 10, 731, 411));
        sensorTab = new QWidget();
        sensorTab->setObjectName(QStringLiteral("sensorTab"));
        rgbGroupBox = new QGroupBox(sensorTab);
        rgbGroupBox->setObjectName(QStringLiteral("rgbGroupBox"));
        rgbGroupBox->setGeometry(QRect(10, 10, 771, 341));
        gridLayoutWidget_3 = new QWidget(rgbGroupBox);
        gridLayoutWidget_3->setObjectName(QStringLiteral("gridLayoutWidget_3"));
        gridLayoutWidget_3->setGeometry(QRect(10, 30, 331, 81));
        rgbLayout = new QGridLayout(gridLayoutWidget_3);
        rgbLayout->setObjectName(QStringLiteral("rgbLayout"));
        rgbLayout->setContentsMargins(0, 0, 0, 0);
        fRgbLabel = new QLabel(gridLayoutWidget_3);
        fRgbLabel->setObjectName(QStringLiteral("fRgbLabel"));

        rgbLayout->addWidget(fRgbLabel, 0, 0, 1, 1);

        cuRgbEdit = new QLineEdit(gridLayoutWidget_3);
        cuRgbEdit->setObjectName(QStringLiteral("cuRgbEdit"));

        rgbLayout->addWidget(cuRgbEdit, 1, 1, 1, 1);

        fuRgbEdit = new QLineEdit(gridLayoutWidget_3);
        fuRgbEdit->setObjectName(QStringLiteral("fuRgbEdit"));

        rgbLayout->addWidget(fuRgbEdit, 0, 1, 1, 1);

        fvRgbEdit = new QLineEdit(gridLayoutWidget_3);
        fvRgbEdit->setObjectName(QStringLiteral("fvRgbEdit"));

        rgbLayout->addWidget(fvRgbEdit, 0, 2, 1, 1);

        cRgbLabel = new QLabel(gridLayoutWidget_3);
        cRgbLabel->setObjectName(QStringLiteral("cRgbLabel"));

        rgbLayout->addWidget(cRgbLabel, 1, 0, 1, 1);

        cvRgbEdit = new QLineEdit(gridLayoutWidget_3);
        cvRgbEdit->setObjectName(QStringLiteral("cvRgbEdit"));

        rgbLayout->addWidget(cvRgbEdit, 1, 2, 1, 1);

        groupBox_2 = new QGroupBox(rgbGroupBox);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(0, 130, 701, 61));
        editVGNfile = new QLineEdit(groupBox_2);
        editVGNfile->setObjectName(QStringLiteral("editVGNfile"));
        editVGNfile->setGeometry(QRect(20, 30, 621, 27));
        pushFindVGNfile = new QPushButton(groupBox_2);
        pushFindVGNfile->setObjectName(QStringLiteral("pushFindVGNfile"));
        pushFindVGNfile->setGeometry(QRect(650, 30, 31, 27));
        applyButton = new QPushButton(sensorTab);
        applyButton->setObjectName(QStringLiteral("applyButton"));
        applyButton->setGeometry(QRect(593, 320, 98, 27));
        groupBox_3 = new QGroupBox(sensorTab);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setGeometry(QRect(10, 210, 701, 61));
        editCRFfile = new QLineEdit(groupBox_3);
        editCRFfile->setObjectName(QStringLiteral("editCRFfile"));
        editCRFfile->setGeometry(QRect(20, 30, 621, 27));
        pushFindCRFfile = new QPushButton(groupBox_3);
        pushFindCRFfile->setObjectName(QStringLiteral("pushFindCRFfile"));
        pushFindCRFfile->setGeometry(QRect(650, 30, 31, 27));
        Tracker->addTab(sensorTab, QString());
        ioTab = new QWidget();
        ioTab->setObjectName(QStringLiteral("ioTab"));
        groupBox = new QGroupBox(ioTab);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(10, 10, 701, 61));
        editRGBDPath = new QLineEdit(groupBox);
        editRGBDPath->setObjectName(QStringLiteral("editRGBDPath"));
        editRGBDPath->setGeometry(QRect(20, 30, 621, 27));
        pushFindRGBDPath = new QPushButton(groupBox);
        pushFindRGBDPath->setObjectName(QStringLiteral("pushFindRGBDPath"));
        pushFindRGBDPath->setGeometry(QRect(650, 30, 31, 27));
        editModelName = new QLineEdit(ioTab);
        editModelName->setObjectName(QStringLiteral("editModelName"));
        editModelName->setGeometry(QRect(30, 73, 621, 27));
        cb_store_cloud = new QCheckBox(ioTab);
        cb_store_cloud->setObjectName(QStringLiteral("cb_store_cloud"));
        cb_store_cloud->setGeometry(QRect(40, 140, 301, 22));
        cb_store_cloud->setChecked(true);
        cb_store_views = new QCheckBox(ioTab);
        cb_store_views->setObjectName(QStringLiteral("cb_store_views"));
        cb_store_views->setGeometry(QRect(40, 170, 431, 22));
        cb_store_views->setChecked(true);
        cb_store_mesh = new QCheckBox(ioTab);
        cb_store_mesh->setObjectName(QStringLiteral("cb_store_mesh"));
        cb_store_mesh->setGeometry(QRect(40, 200, 231, 22));
        cb_store_mesh->setChecked(false);
        cb_store_tracker = new QCheckBox(ioTab);
        cb_store_tracker->setObjectName(QStringLiteral("cb_store_tracker"));
        cb_store_tracker->setGeometry(QRect(40, 260, 251, 22));
        cb_store_tracker->setChecked(false);
        cb_store_tex_mesh = new QCheckBox(ioTab);
        cb_store_tex_mesh->setObjectName(QStringLiteral("cb_store_tex_mesh"));
        cb_store_tex_mesh->setGeometry(QRect(40, 230, 231, 22));
        cb_store_tex_mesh->setChecked(false);
        Tracker->addTab(ioTab, QString());
        tab_5 = new QWidget();
        tab_5->setObjectName(QStringLiteral("tab_5"));
        label_14 = new QLabel(tab_5);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(9, 12, 281, 17));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        label_14->setFont(font);
        layoutWidget11_2 = new QWidget(tab_5);
        layoutWidget11_2->setObjectName(QStringLiteral("layoutWidget11_2"));
        layoutWidget11_2->setGeometry(QRect(31, 45, 581, 95));
        formLayout_3 = new QFormLayout(layoutWidget11_2);
        formLayout_3->setObjectName(QStringLiteral("formLayout_3"));
        formLayout_3->setContentsMargins(0, 0, 0, 0);
        roi_scale_xy = new QLineEdit(layoutWidget11_2);
        roi_scale_xy->setObjectName(QStringLiteral("roi_scale_xy"));

        formLayout_3->setWidget(0, QFormLayout::LabelRole, roi_scale_xy);

        label_15 = new QLabel(layoutWidget11_2);
        label_15->setObjectName(QStringLiteral("label_15"));

        formLayout_3->setWidget(0, QFormLayout::FieldRole, label_15);

        roi_scale_height = new QLineEdit(layoutWidget11_2);
        roi_scale_height->setObjectName(QStringLiteral("roi_scale_height"));

        formLayout_3->setWidget(1, QFormLayout::LabelRole, roi_scale_height);

        label_16 = new QLabel(layoutWidget11_2);
        label_16->setObjectName(QStringLiteral("label_16"));

        formLayout_3->setWidget(1, QFormLayout::FieldRole, label_16);

        roi_offs = new QLineEdit(layoutWidget11_2);
        roi_offs->setObjectName(QStringLiteral("roi_offs"));

        formLayout_3->setWidget(2, QFormLayout::LabelRole, roi_offs);

        label_17 = new QLabel(layoutWidget11_2);
        label_17->setObjectName(QStringLiteral("label_17"));

        formLayout_3->setWidget(2, QFormLayout::FieldRole, label_17);

        Tracker->addTab(tab_5, QString());
        Modelling = new QWidget();
        Modelling->setObjectName(QStringLiteral("Modelling"));
        label_18 = new QLabel(Modelling);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setEnabled(true);
        label_18->setGeometry(QRect(120, 174, 521, 27));
        voxel_grid_size = new QLineEdit(Modelling);
        voxel_grid_size->setObjectName(QStringLiteral("voxel_grid_size"));
        voxel_grid_size->setEnabled(true);
        voxel_grid_size->setGeometry(QRect(40, 174, 61, 27));
        label_19 = new QLabel(Modelling);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setEnabled(true);
        label_19->setGeometry(QRect(120, 207, 521, 27));
        poisson_depth = new QLineEdit(Modelling);
        poisson_depth->setObjectName(QStringLiteral("poisson_depth"));
        poisson_depth->setEnabled(true);
        poisson_depth->setGeometry(QRect(40, 207, 61, 27));
        filter_largest_cluster = new QCheckBox(Modelling);
        filter_largest_cluster->setObjectName(QStringLiteral("filter_largest_cluster"));
        filter_largest_cluster->setEnabled(true);
        filter_largest_cluster->setGeometry(QRect(40, 283, 301, 22));
        filter_largest_cluster->setChecked(true);
        poisson_samples = new QLineEdit(Modelling);
        poisson_samples->setObjectName(QStringLiteral("poisson_samples"));
        poisson_samples->setEnabled(true);
        poisson_samples->setGeometry(QRect(41, 240, 61, 27));
        label_20 = new QLabel(Modelling);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setEnabled(true);
        label_20->setGeometry(QRect(121, 240, 521, 27));
        multiview_icp = new QCheckBox(Modelling);
        multiview_icp->setObjectName(QStringLiteral("multiview_icp"));
        multiview_icp->setEnabled(true);
        multiview_icp->setGeometry(QRect(40, 313, 301, 22));
        multiview_icp->setChecked(false);
        use_nguyen_noise_model = new QCheckBox(Modelling);
        use_nguyen_noise_model->setObjectName(QStringLiteral("use_nguyen_noise_model"));
        use_nguyen_noise_model->setEnabled(true);
        use_nguyen_noise_model->setGeometry(QRect(40, 343, 301, 22));
        use_nguyen_noise_model->setChecked(false);
        label_21 = new QLabel(Modelling);
        label_21->setObjectName(QStringLiteral("label_21"));
        label_21->setEnabled(true);
        label_21->setGeometry(QRect(119, 26, 521, 27));
        nb_frames_temp_filt = new QLineEdit(Modelling);
        nb_frames_temp_filt->setObjectName(QStringLiteral("nb_frames_temp_filt"));
        nb_frames_temp_filt->setEnabled(true);
        nb_frames_temp_filt->setGeometry(QRect(39, 59, 61, 27));
        label_22 = new QLabel(Modelling);
        label_22->setObjectName(QStringLiteral("label_22"));
        label_22->setEnabled(true);
        label_22->setGeometry(QRect(119, 59, 521, 27));
        label_23 = new QLabel(Modelling);
        label_23->setObjectName(QStringLiteral("label_23"));
        label_23->setEnabled(true);
        label_23->setGeometry(QRect(120, 92, 521, 27));
        min_cam_motion_kf = new QLineEdit(Modelling);
        min_cam_motion_kf->setObjectName(QStringLiteral("min_cam_motion_kf"));
        min_cam_motion_kf->setEnabled(true);
        min_cam_motion_kf->setGeometry(QRect(40, 92, 61, 27));
        nb_frames_ba = new QLineEdit(Modelling);
        nb_frames_ba->setObjectName(QStringLiteral("nb_frames_ba"));
        nb_frames_ba->setEnabled(true);
        nb_frames_ba->setGeometry(QRect(39, 26, 61, 27));
        min_cam_rot_kf = new QLineEdit(Modelling);
        min_cam_rot_kf->setObjectName(QStringLiteral("min_cam_rot_kf"));
        min_cam_rot_kf->setEnabled(true);
        min_cam_rot_kf->setGeometry(QRect(40, 127, 61, 27));
        min_cam_rot_kf_txt = new QLabel(Modelling);
        min_cam_rot_kf_txt->setObjectName(QStringLiteral("min_cam_rot_kf_txt"));
        min_cam_rot_kf_txt->setEnabled(true);
        min_cam_rot_kf_txt->setGeometry(QRect(120, 127, 521, 27));
        Tracker->addTab(Modelling, QString());
        okButton = new QPushButton(Params);
        okButton->setObjectName(QStringLiteral("okButton"));
        okButton->setGeometry(QRect(642, 430, 98, 27));

        retranslateUi(Params);
        QObject::connect(okButton, SIGNAL(clicked()), Params, SLOT(hide()));

        Tracker->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(Params);
    } // setupUi

    void retranslateUi(QWidget *Params)
    {
        Params->setWindowTitle(QApplication::translate("Params", "Preferences", 0));
        rgbGroupBox->setTitle(QApplication::translate("Params", "OpenNI camera parameter", 0));
        fRgbLabel->setText(QApplication::translate("Params", "Focal lengths", 0));
        cuRgbEdit->setText(QApplication::translate("Params", "320", 0));
        fuRgbEdit->setText(QApplication::translate("Params", "525", 0));
        fvRgbEdit->setText(QApplication::translate("Params", "525", 0));
        cRgbLabel->setText(QApplication::translate("Params", "Principal point", 0));
        cvRgbEdit->setText(QApplication::translate("Params", "240", 0));
        groupBox_2->setTitle(QApplication::translate("Params", "RADICAL vgn-file for vignetting correction (optional)", 0));
        editVGNfile->setText(QString());
        pushFindVGNfile->setText(QApplication::translate("Params", "...", 0));
        applyButton->setText(QApplication::translate("Params", "Apply", 0));
        groupBox_3->setTitle(QApplication::translate("Params", "RADICAL crf-file for vignetting correction (optional)", 0));
        editCRFfile->setText(QString());
        pushFindCRFfile->setText(QApplication::translate("Params", "...", 0));
        Tracker->setTabText(Tracker->indexOf(sensorTab), QApplication::translate("Params", "Sensor", 0));
        groupBox->setTitle(QApplication::translate("Params", "Path and model name", 0));
        editRGBDPath->setText(QApplication::translate("Params", "./data", 0));
        pushFindRGBDPath->setText(QApplication::translate("Params", "...", 0));
        editModelName->setText(QApplication::translate("Params", "modelname", 0));
        cb_store_cloud->setText(QApplication::translate("Params", "Point cloud (for recognition)", 0));
        cb_store_views->setText(QApplication::translate("Params", "Organized point clouds (selected views for recognition)", 0));
        cb_store_mesh->setText(QApplication::translate("Params", "Triangle mesh", 0));
        cb_store_tracker->setText(QApplication::translate("Params", "Tracking model", 0));
        cb_store_tex_mesh->setText(QApplication::translate("Params", "Textured triangle mesh", 0));
        Tracker->setTabText(Tracker->indexOf(ioTab), QApplication::translate("Params", "Settings", 0));
        label_14->setText(QApplication::translate("Params", "Region of interesst (ROI)", 0));
        roi_scale_xy->setText(QApplication::translate("Params", "0.75", 0));
        label_15->setText(QApplication::translate("Params", "XY-scale depending on the selected plane", 0));
        roi_scale_height->setText(QApplication::translate("Params", "0.75", 0));
        label_16->setText(QApplication::translate("Params", "Height-scale depending on the selected plane", 0));
        roi_offs->setText(QApplication::translate("Params", "0.01", 0));
        label_17->setText(QApplication::translate("Params", "Offset for segmentation [m]", 0));
        Tracker->setTabText(Tracker->indexOf(tab_5), QApplication::translate("Params", "ROI Settings", 0));
        label_18->setText(QApplication::translate("Params", "Voxel size (resolution of the final point cloud model [m])", 0));
        voxel_grid_size->setText(QApplication::translate("Params", "0.001", 0));
        label_19->setText(QApplication::translate("Params", "Poisson depth (surface triangulation)", 0));
        poisson_depth->setText(QApplication::translate("Params", "7", 0));
        filter_largest_cluster->setText(QApplication::translate("Params", "Return largest cluster", 0));
        poisson_samples->setText(QApplication::translate("Params", "2", 0));
        label_20->setText(QApplication::translate("Params", "Poisson samples per node (surface triangulation)", 0));
        multiview_icp->setText(QApplication::translate("Params", "Multiview ICP refinement", 0));
        use_nguyen_noise_model->setText(QApplication::translate("Params", "Use Nguyen noise model", 0));
        label_21->setText(QApplication::translate("Params", "Number of nearby frames used for bundle adjustment", 0));
        nb_frames_temp_filt->setText(QApplication::translate("Params", "11", 0));
        label_22->setText(QApplication::translate("Params", "Number of frames for temporal filtering (batch size)", 0));
        label_23->setText(QApplication::translate("Params", "Min. camera motion to store a keyframe [m]", 0));
        min_cam_motion_kf->setText(QApplication::translate("Params", "0.5", 0));
        nb_frames_ba->setText(QApplication::translate("Params", "10", 0));
        min_cam_rot_kf->setText(QApplication::translate("Params", "7", 0));
        min_cam_rot_kf_txt->setText(QApplication::translate("Params", "Min. camera rotation to store a keyframe [deg]", 0));
        Tracker->setTabText(Tracker->indexOf(Modelling), QApplication::translate("Params", "Modelling", 0));
        okButton->setText(QApplication::translate("Params", "OK", 0));
    } // retranslateUi

};

namespace Ui {
    class Params: public Ui_Params {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PARAMS_H
