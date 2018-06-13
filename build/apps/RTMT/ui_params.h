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
    QPushButton *applyButton;
    QWidget *ioTab;
    QGroupBox *groupBox;
    QLineEdit *editRGBDPath;
    QPushButton *pushFindRGBDPath;
    QLabel *label_4;
    QWidget *layoutWidget10;
    QFormLayout *formLayout;
    QCheckBox *logPointClouds;
    QLineEdit *logDeltaAngle;
    QLabel *label;
    QLineEdit *logDeltaDistance;
    QLabel *label_2;
    QLineEdit *prevVoxelGridSize;
    QLabel *label_3;
    QLineEdit *prevZCutOff;
    QLabel *label_5;
    QCheckBox *createPreviewCloud;
    QLineEdit *editModelName;
    QWidget *tab;
    QLabel *label_6;
    QLabel *label_7;
    QLineEdit *distCamAddProjections;
    QLabel *label_8;
    QLabel *label_12;
    QLineEdit *vxSizeObject;
    QLabel *label_13;
    QWidget *layoutWidget11;
    QFormLayout *formLayout_2;
    QLineEdit *inlDistPlane;
    QLabel *label_9;
    QLineEdit *thrAngle;
    QLabel *label_11;
    QLineEdit *minPointsPlane;
    QLabel *label_10;
    QCheckBox *use_dense_mv;
    QLabel *label_18;
    QLineEdit *edge_radius_px;
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
    QCheckBox *use_roi_segm;
    QWidget *tab_2;
    QLabel *label_19;
    QLineEdit *model_rnn_thr;
    QCheckBox *model_create_cb;
    QLabel *label_20;
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
        rgbGroupBox->setGeometry(QRect(10, 10, 341, 341));
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

        applyButton = new QPushButton(sensorTab);
        applyButton->setObjectName(QStringLiteral("applyButton"));
        applyButton->setGeometry(QRect(590, 320, 98, 27));
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
        label_4 = new QLabel(ioTab);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(10, 120, 241, 17));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        label_4->setFont(font);
        layoutWidget10 = new QWidget(ioTab);
        layoutWidget10->setObjectName(QStringLiteral("layoutWidget10"));
        layoutWidget10->setGeometry(QRect(30, 145, 562, 184));
        formLayout = new QFormLayout(layoutWidget10);
        formLayout->setObjectName(QStringLiteral("formLayout"));
        formLayout->setContentsMargins(0, 0, 0, 0);
        logPointClouds = new QCheckBox(layoutWidget10);
        logPointClouds->setObjectName(QStringLiteral("logPointClouds"));
        logPointClouds->setChecked(true);

        formLayout->setWidget(0, QFormLayout::LabelRole, logPointClouds);

        logDeltaAngle = new QLineEdit(layoutWidget10);
        logDeltaAngle->setObjectName(QStringLiteral("logDeltaAngle"));

        formLayout->setWidget(2, QFormLayout::LabelRole, logDeltaAngle);

        label = new QLabel(layoutWidget10);
        label->setObjectName(QStringLiteral("label"));

        formLayout->setWidget(2, QFormLayout::FieldRole, label);

        logDeltaDistance = new QLineEdit(layoutWidget10);
        logDeltaDistance->setObjectName(QStringLiteral("logDeltaDistance"));

        formLayout->setWidget(3, QFormLayout::LabelRole, logDeltaDistance);

        label_2 = new QLabel(layoutWidget10);
        label_2->setObjectName(QStringLiteral("label_2"));

        formLayout->setWidget(3, QFormLayout::FieldRole, label_2);

        prevVoxelGridSize = new QLineEdit(layoutWidget10);
        prevVoxelGridSize->setObjectName(QStringLiteral("prevVoxelGridSize"));

        formLayout->setWidget(4, QFormLayout::LabelRole, prevVoxelGridSize);

        label_3 = new QLabel(layoutWidget10);
        label_3->setObjectName(QStringLiteral("label_3"));

        formLayout->setWidget(4, QFormLayout::FieldRole, label_3);

        prevZCutOff = new QLineEdit(layoutWidget10);
        prevZCutOff->setObjectName(QStringLiteral("prevZCutOff"));

        formLayout->setWidget(5, QFormLayout::LabelRole, prevZCutOff);

        label_5 = new QLabel(layoutWidget10);
        label_5->setObjectName(QStringLiteral("label_5"));

        formLayout->setWidget(5, QFormLayout::FieldRole, label_5);

        createPreviewCloud = new QCheckBox(layoutWidget10);
        createPreviewCloud->setObjectName(QStringLiteral("createPreviewCloud"));
        createPreviewCloud->setChecked(true);

        formLayout->setWidget(1, QFormLayout::LabelRole, createPreviewCloud);

        editModelName = new QLineEdit(ioTab);
        editModelName->setObjectName(QStringLiteral("editModelName"));
        editModelName->setGeometry(QRect(30, 73, 621, 27));
        Tracker->addTab(ioTab, QString());
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        label_6 = new QLabel(tab);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(10, 12, 281, 17));
        label_6->setFont(font);
        label_7 = new QLabel(tab);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(10, 92, 281, 17));
        label_7->setFont(font);
        distCamAddProjections = new QLineEdit(tab);
        distCamAddProjections->setObjectName(QStringLiteral("distCamAddProjections"));
        distCamAddProjections->setGeometry(QRect(32, 44, 146, 27));
        label_8 = new QLabel(tab);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(190, 44, 402, 27));
        label_12 = new QLabel(tab);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(10, 243, 281, 17));
        label_12->setFont(font);
        vxSizeObject = new QLineEdit(tab);
        vxSizeObject->setObjectName(QStringLiteral("vxSizeObject"));
        vxSizeObject->setGeometry(QRect(32, 273, 146, 27));
        label_13 = new QLabel(tab);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(190, 273, 402, 27));
        layoutWidget11 = new QWidget(tab);
        layoutWidget11->setObjectName(QStringLiteral("layoutWidget11"));
        layoutWidget11->setGeometry(QRect(32, 125, 581, 95));
        formLayout_2 = new QFormLayout(layoutWidget11);
        formLayout_2->setObjectName(QStringLiteral("formLayout_2"));
        formLayout_2->setContentsMargins(0, 0, 0, 0);
        inlDistPlane = new QLineEdit(layoutWidget11);
        inlDistPlane->setObjectName(QStringLiteral("inlDistPlane"));

        formLayout_2->setWidget(0, QFormLayout::LabelRole, inlDistPlane);

        label_9 = new QLabel(layoutWidget11);
        label_9->setObjectName(QStringLiteral("label_9"));

        formLayout_2->setWidget(0, QFormLayout::FieldRole, label_9);

        thrAngle = new QLineEdit(layoutWidget11);
        thrAngle->setObjectName(QStringLiteral("thrAngle"));

        formLayout_2->setWidget(1, QFormLayout::LabelRole, thrAngle);

        label_11 = new QLabel(layoutWidget11);
        label_11->setObjectName(QStringLiteral("label_11"));

        formLayout_2->setWidget(1, QFormLayout::FieldRole, label_11);

        minPointsPlane = new QLineEdit(layoutWidget11);
        minPointsPlane->setObjectName(QStringLiteral("minPointsPlane"));

        formLayout_2->setWidget(2, QFormLayout::LabelRole, minPointsPlane);

        label_10 = new QLabel(layoutWidget11);
        label_10->setObjectName(QStringLiteral("label_10"));

        formLayout_2->setWidget(2, QFormLayout::FieldRole, label_10);

        use_dense_mv = new QCheckBox(tab);
        use_dense_mv->setObjectName(QStringLiteral("use_dense_mv"));
        use_dense_mv->setGeometry(QRect(34, 344, 281, 22));
        label_18 = new QLabel(tab);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setGeometry(QRect(190, 306, 402, 27));
        edge_radius_px = new QLineEdit(tab);
        edge_radius_px->setObjectName(QStringLiteral("edge_radius_px"));
        edge_radius_px->setGeometry(QRect(32, 306, 146, 27));
        Tracker->addTab(tab, QString());
        tab_5 = new QWidget();
        tab_5->setObjectName(QStringLiteral("tab_5"));
        label_14 = new QLabel(tab_5);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(9, 12, 281, 17));
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

        use_roi_segm = new QCheckBox(tab_5);
        use_roi_segm->setObjectName(QStringLiteral("use_roi_segm"));
        use_roi_segm->setGeometry(QRect(30, 150, 311, 22));
        use_roi_segm->setChecked(true);
        Tracker->addTab(tab_5, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        label_19 = new QLabel(tab_2);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setGeometry(QRect(180, 80, 427, 27));
        model_rnn_thr = new QLineEdit(tab_2);
        model_rnn_thr->setObjectName(QStringLiteral("model_rnn_thr"));
        model_rnn_thr->setGeometry(QRect(28, 80, 146, 27));
        model_create_cb = new QCheckBox(tab_2);
        model_create_cb->setObjectName(QStringLiteral("model_create_cb"));
        model_create_cb->setGeometry(QRect(28, 48, 311, 22));
        model_create_cb->setChecked(true);
        label_20 = new QLabel(tab_2);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setGeometry(QRect(9, 12, 281, 17));
        label_20->setFont(font);
        Tracker->addTab(tab_2, QString());
        okButton = new QPushButton(Params);
        okButton->setObjectName(QStringLiteral("okButton"));
        okButton->setGeometry(QRect(642, 430, 98, 27));

        retranslateUi(Params);
        QObject::connect(okButton, SIGNAL(clicked()), Params, SLOT(hide()));

        Tracker->setCurrentIndex(2);


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
        applyButton->setText(QApplication::translate("Params", "Apply", 0));
        Tracker->setTabText(Tracker->indexOf(sensorTab), QApplication::translate("Params", "Sensor", 0));
        groupBox->setTitle(QApplication::translate("Params", "Path and model name", 0));
        editRGBDPath->setText(QApplication::translate("Params", "./data", 0));
        pushFindRGBDPath->setText(QApplication::translate("Params", "...", 0));
        label_4->setText(QApplication::translate("Params", "Data logging", 0));
        logPointClouds->setText(QApplication::translate("Params", "log point clouds", 0));
        logDeltaAngle->setText(QApplication::translate("Params", "15", 0));
        label->setText(QApplication::translate("Params", "Min. delta angle to log an image", 0));
        logDeltaDistance->setText(QApplication::translate("Params", "1.0", 0));
        label_2->setText(QApplication::translate("Params", "Min. delta camera distance to log an image", 0));
        prevVoxelGridSize->setText(QApplication::translate("Params", "0.01", 0));
        label_3->setText(QApplication::translate("Params", "Preview voxel grid size", 0));
        prevZCutOff->setText(QApplication::translate("Params", "2.0", 0));
        label_5->setText(QApplication::translate("Params", "Preview point cloud z-value cut off (camera coordinates)", 0));
        createPreviewCloud->setText(QApplication::translate("Params", "create prev. cloud", 0));
        editModelName->setText(QApplication::translate("Params", "modelname", 0));
        Tracker->setTabText(Tracker->indexOf(ioTab), QApplication::translate("Params", "Settings", 0));
        label_6->setText(QApplication::translate("Params", "Bundle adjustment (Optimize Poses)", 0));
        label_7->setText(QApplication::translate("Params", "Object segmentation", 0));
        distCamAddProjections->setText(QApplication::translate("Params", "0.02", 0));
        label_8->setText(QApplication::translate("Params", "Min. camera motion to select projections [m]", 0));
        label_12->setText(QApplication::translate("Params", "Object modelling", 0));
        vxSizeObject->setText(QApplication::translate("Params", "0.002", 0));
        label_13->setText(QApplication::translate("Params", "Voxelgrid size for the final object model [m]", 0));
        inlDistPlane->setText(QApplication::translate("Params", "0.01", 0));
        label_9->setText(QApplication::translate("Params", "Inlier distance for dominant plane segmentation [m]", 0));
        thrAngle->setText(QApplication::translate("Params", "30", 0));
        label_11->setText(QApplication::translate("Params", "Max. deviation of normals for smooth surface clustering [\302\260]", 0));
        minPointsPlane->setText(QApplication::translate("Params", "7000", 0));
        label_10->setText(QApplication::translate("Params", "Min. points to accept a dominant plane", 0));
        use_dense_mv->setText(QApplication::translate("Params", "Use dense multiview optimization", 0));
        label_18->setText(QApplication::translate("Params", "Radius for point removal at discontinuity edges  [px]", 0));
        edge_radius_px->setText(QApplication::translate("Params", "3", 0));
        Tracker->setTabText(Tracker->indexOf(tab), QApplication::translate("Params", "Postprocessing", 0));
        label_14->setText(QApplication::translate("Params", "Region of interesst (ROI)", 0));
        roi_scale_xy->setText(QApplication::translate("Params", "0.75", 0));
        label_15->setText(QApplication::translate("Params", "XY-scale depending on the selected plane", 0));
        roi_scale_height->setText(QApplication::translate("Params", "0.75", 0));
        label_16->setText(QApplication::translate("Params", "Height-scale depending on the selected plane", 0));
        roi_offs->setText(QApplication::translate("Params", "0.008", 0));
        label_17->setText(QApplication::translate("Params", "Offset for segmentation [m]", 0));
        use_roi_segm->setText(QApplication::translate("Params", "Use ROI for object segmentation", 0));
        Tracker->setTabText(Tracker->indexOf(tab_5), QApplication::translate("Params", "ROI Settings", 0));
        label_19->setText(QApplication::translate("Params", "Threshold for codebook clustering", 0));
        model_rnn_thr->setText(QApplication::translate("Params", "0.55", 0));
        model_create_cb->setText(QApplication::translate("Params", "Create codebook", 0));
        label_20->setText(QApplication::translate("Params", "Tracking model", 0));
        Tracker->setTabText(Tracker->indexOf(tab_2), QApplication::translate("Params", "Model", 0));
        okButton->setText(QApplication::translate("Params", "OK", 0));
    } // retranslateUi

};

namespace Ui {
    class Params: public Ui_Params {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PARAMS_H
