/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionExit;
    QAction *actionPreferences;
    QWidget *centralWidget;
    QGraphicsView *glView;
    QLabel *statusLabel;
    QWidget *layoutWidget4;
    QVBoxLayout *verticalLayout;
    QPushButton *imForward;
    QPushButton *imBackward;
    QWidget *layoutWidget5;
    QVBoxLayout *verticalLayout_2;
    QCheckBox *ShowImage;
    QCheckBox *ShowDepthMask;
    QCheckBox *ShowCameras;
    QCheckBox *ShowPointCloud;
    QCheckBox *ShowObjectModel;
    QPushButton *ResetView;
    QWidget *layoutWidget;
    QGridLayout *gridLayout;
    QLabel *label;
    QPushButton *CamStart;
    QPushButton *CamStop;
    QLabel *label_3;
    QPushButton *setROI;
    QPushButton *ActivateROI;
    QPushButton *ResetROI;
    QLabel *label_2;
    QPushButton *TrackerStart;
    QPushButton *TrackerStop;
    QPushButton *ResetTracker;
    QWidget *layoutWidget1;
    QGridLayout *gridLayout_2;
    QPushButton *SegmentObject;
    QLabel *label_4;
    QPushButton *OptimizePoses;
    QPushButton *undoOptimize;
    QLabel *label_5;
    QLabel *label_7;
    QPushButton *okSegmentation;
    QPushButton *SessionAdd;
    QPushButton *SessionAlign;
    QPushButton *SessionClear;
    QPushButton *SessionOptimize;
    QPushButton *OptimizeObject;
    QWidget *layoutWidget2;
    QGridLayout *gridLayout_3;
    QLabel *label_6;
    QPushButton *SaveTrackerModel;
    QPushButton *SavePointClouds;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->setEnabled(true);
        MainWindow->resize(1100, 551);
        actionExit = new QAction(MainWindow);
        actionExit->setObjectName(QStringLiteral("actionExit"));
        actionPreferences = new QAction(MainWindow);
        actionPreferences->setObjectName(QStringLiteral("actionPreferences"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        glView = new QGraphicsView(centralWidget);
        glView->setObjectName(QStringLiteral("glView"));
        glView->setGeometry(QRect(10, 0, 640, 480));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(glView->sizePolicy().hasHeightForWidth());
        glView->setSizePolicy(sizePolicy);
        glView->setMaximumSize(QSize(640, 480));
        glView->setFrameShape(QFrame::NoFrame);
        glView->setFrameShadow(QFrame::Plain);
        glView->setLineWidth(0);
        glView->setSceneRect(QRectF(0, 0, 640, 480));
        glView->setViewportUpdateMode(QGraphicsView::NoViewportUpdate);
        statusLabel = new QLabel(centralWidget);
        statusLabel->setObjectName(QStringLiteral("statusLabel"));
        statusLabel->setGeometry(QRect(13, 488, 640, 17));
        layoutWidget4 = new QWidget(centralWidget);
        layoutWidget4->setObjectName(QStringLiteral("layoutWidget4"));
        layoutWidget4->setGeometry(QRect(653, 410, 16, 62));
        verticalLayout = new QVBoxLayout(layoutWidget4);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        imForward = new QPushButton(layoutWidget4);
        imForward->setObjectName(QStringLiteral("imForward"));
        imForward->setEnabled(false);

        verticalLayout->addWidget(imForward);

        imBackward = new QPushButton(layoutWidget4);
        imBackward->setObjectName(QStringLiteral("imBackward"));
        imBackward->setEnabled(false);

        verticalLayout->addWidget(imBackward);

        layoutWidget5 = new QWidget(centralWidget);
        layoutWidget5->setObjectName(QStringLiteral("layoutWidget5"));
        layoutWidget5->setGeometry(QRect(707, 346, 166, 136));
        verticalLayout_2 = new QVBoxLayout(layoutWidget5);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        ShowImage = new QCheckBox(layoutWidget5);
        ShowImage->setObjectName(QStringLiteral("ShowImage"));
        ShowImage->setChecked(true);

        verticalLayout_2->addWidget(ShowImage);

        ShowDepthMask = new QCheckBox(layoutWidget5);
        ShowDepthMask->setObjectName(QStringLiteral("ShowDepthMask"));
        ShowDepthMask->setChecked(true);

        verticalLayout_2->addWidget(ShowDepthMask);

        ShowCameras = new QCheckBox(layoutWidget5);
        ShowCameras->setObjectName(QStringLiteral("ShowCameras"));

        verticalLayout_2->addWidget(ShowCameras);

        ShowPointCloud = new QCheckBox(layoutWidget5);
        ShowPointCloud->setObjectName(QStringLiteral("ShowPointCloud"));
        ShowPointCloud->setEnabled(true);
        ShowPointCloud->setChecked(false);

        verticalLayout_2->addWidget(ShowPointCloud);

        ShowObjectModel = new QCheckBox(layoutWidget5);
        ShowObjectModel->setObjectName(QStringLiteral("ShowObjectModel"));
        ShowObjectModel->setEnabled(true);
        ShowObjectModel->setChecked(false);

        verticalLayout_2->addWidget(ShowObjectModel);

        ResetView = new QPushButton(centralWidget);
        ResetView->setObjectName(QStringLiteral("ResetView"));
        ResetView->setGeometry(QRect(710, 316, 161, 27));
        layoutWidget = new QWidget(centralWidget);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(660, 20, 331, 95));
        gridLayout = new QGridLayout(layoutWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(layoutWidget);
        label->setObjectName(QStringLiteral("label"));

        gridLayout->addWidget(label, 0, 0, 1, 1);

        CamStart = new QPushButton(layoutWidget);
        CamStart->setObjectName(QStringLiteral("CamStart"));

        gridLayout->addWidget(CamStart, 0, 1, 1, 1);

        CamStop = new QPushButton(layoutWidget);
        CamStop->setObjectName(QStringLiteral("CamStop"));

        gridLayout->addWidget(CamStop, 0, 2, 1, 1);

        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout->addWidget(label_3, 1, 0, 1, 1);

        setROI = new QPushButton(layoutWidget);
        setROI->setObjectName(QStringLiteral("setROI"));

        gridLayout->addWidget(setROI, 1, 1, 1, 1);

        ActivateROI = new QPushButton(layoutWidget);
        ActivateROI->setObjectName(QStringLiteral("ActivateROI"));

        gridLayout->addWidget(ActivateROI, 1, 2, 1, 1);

        ResetROI = new QPushButton(layoutWidget);
        ResetROI->setObjectName(QStringLiteral("ResetROI"));

        gridLayout->addWidget(ResetROI, 1, 3, 1, 1);

        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout->addWidget(label_2, 2, 0, 1, 1);

        TrackerStart = new QPushButton(layoutWidget);
        TrackerStart->setObjectName(QStringLiteral("TrackerStart"));

        gridLayout->addWidget(TrackerStart, 2, 1, 1, 1);

        TrackerStop = new QPushButton(layoutWidget);
        TrackerStop->setObjectName(QStringLiteral("TrackerStop"));

        gridLayout->addWidget(TrackerStop, 2, 2, 1, 1);

        ResetTracker = new QPushButton(layoutWidget);
        ResetTracker->setObjectName(QStringLiteral("ResetTracker"));

        gridLayout->addWidget(ResetTracker, 2, 3, 1, 1);

        layoutWidget1 = new QWidget(centralWidget);
        layoutWidget1->setObjectName(QStringLiteral("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(660, 133, 421, 91));
        gridLayout_2 = new QGridLayout(layoutWidget1);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        SegmentObject = new QPushButton(layoutWidget1);
        SegmentObject->setObjectName(QStringLiteral("SegmentObject"));

        gridLayout_2->addWidget(SegmentObject, 1, 1, 1, 1);

        label_4 = new QLabel(layoutWidget1);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout_2->addWidget(label_4, 0, 0, 1, 1);

        OptimizePoses = new QPushButton(layoutWidget1);
        OptimizePoses->setObjectName(QStringLiteral("OptimizePoses"));
        OptimizePoses->setEnabled(true);

        gridLayout_2->addWidget(OptimizePoses, 0, 1, 1, 1);

        undoOptimize = new QPushButton(layoutWidget1);
        undoOptimize->setObjectName(QStringLiteral("undoOptimize"));
        undoOptimize->setEnabled(true);

        gridLayout_2->addWidget(undoOptimize, 0, 2, 1, 1);

        label_5 = new QLabel(layoutWidget1);
        label_5->setObjectName(QStringLiteral("label_5"));

        gridLayout_2->addWidget(label_5, 1, 0, 1, 1);

        label_7 = new QLabel(layoutWidget1);
        label_7->setObjectName(QStringLiteral("label_7"));

        gridLayout_2->addWidget(label_7, 2, 0, 1, 1);

        okSegmentation = new QPushButton(layoutWidget1);
        okSegmentation->setObjectName(QStringLiteral("okSegmentation"));
        okSegmentation->setEnabled(true);

        gridLayout_2->addWidget(okSegmentation, 1, 2, 1, 1);

        SessionAdd = new QPushButton(layoutWidget1);
        SessionAdd->setObjectName(QStringLiteral("SessionAdd"));

        gridLayout_2->addWidget(SessionAdd, 2, 1, 1, 1);

        SessionAlign = new QPushButton(layoutWidget1);
        SessionAlign->setObjectName(QStringLiteral("SessionAlign"));

        gridLayout_2->addWidget(SessionAlign, 2, 2, 1, 1);

        SessionClear = new QPushButton(layoutWidget1);
        SessionClear->setObjectName(QStringLiteral("SessionClear"));

        gridLayout_2->addWidget(SessionClear, 2, 4, 1, 1);

        SessionOptimize = new QPushButton(layoutWidget1);
        SessionOptimize->setObjectName(QStringLiteral("SessionOptimize"));

        gridLayout_2->addWidget(SessionOptimize, 2, 3, 1, 1);

        OptimizeObject = new QPushButton(layoutWidget1);
        OptimizeObject->setObjectName(QStringLiteral("OptimizeObject"));
        OptimizeObject->setEnabled(true);

        gridLayout_2->addWidget(OptimizeObject, 1, 3, 1, 1);

        layoutWidget2 = new QWidget(centralWidget);
        layoutWidget2->setObjectName(QStringLiteral("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(660, 254, 251, 29));
        gridLayout_3 = new QGridLayout(layoutWidget2);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        label_6 = new QLabel(layoutWidget2);
        label_6->setObjectName(QStringLiteral("label_6"));

        gridLayout_3->addWidget(label_6, 0, 0, 1, 1);

        SaveTrackerModel = new QPushButton(layoutWidget2);
        SaveTrackerModel->setObjectName(QStringLiteral("SaveTrackerModel"));
        SaveTrackerModel->setEnabled(true);

        gridLayout_3->addWidget(SaveTrackerModel, 0, 1, 1, 1);

        SavePointClouds = new QPushButton(layoutWidget2);
        SavePointClouds->setObjectName(QStringLiteral("SavePointClouds"));

        gridLayout_3->addWidget(SavePointClouds, 0, 2, 1, 1);

        MainWindow->setCentralWidget(centralWidget);
        layoutWidget->raise();
        layoutWidget->raise();
        layoutWidget->raise();
        glView->raise();
        statusLabel->raise();
        layoutWidget4->raise();
        layoutWidget5->raise();
        ResetView->raise();
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1100, 25));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuFile->addSeparator();
        menuFile->addAction(actionPreferences);
        menuFile->addSeparator();
        menuFile->addAction(actionExit);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
#ifndef QT_NO_STATUSTIP
        MainWindow->setStatusTip(QString());
#endif // QT_NO_STATUSTIP
        actionExit->setText(QApplication::translate("MainWindow", "Exit", 0));
        actionPreferences->setText(QApplication::translate("MainWindow", "Preferences", 0));
        statusLabel->setText(QApplication::translate("MainWindow", "Status: Ready", 0));
        imForward->setText(QApplication::translate("MainWindow", ">", 0));
        imBackward->setText(QApplication::translate("MainWindow", "<", 0));
        ShowImage->setText(QApplication::translate("MainWindow", "image", 0));
        ShowDepthMask->setText(QApplication::translate("MainWindow", "mask missing depth", 0));
        ShowCameras->setText(QApplication::translate("MainWindow", "camera trajectory", 0));
        ShowPointCloud->setText(QApplication::translate("MainWindow", "point cloud", 0));
        ShowObjectModel->setText(QApplication::translate("MainWindow", "object model", 0));
        ResetView->setText(QApplication::translate("MainWindow", "Reset View", 0));
        label->setText(QApplication::translate("MainWindow", "Camera", 0));
        CamStart->setText(QApplication::translate("MainWindow", "Start", 0));
        CamStop->setText(QApplication::translate("MainWindow", "Stop", 0));
        label_3->setText(QApplication::translate("MainWindow", "ROI", 0));
        setROI->setText(QApplication::translate("MainWindow", "Set", 0));
        ActivateROI->setText(QApplication::translate("MainWindow", "Activate", 0));
        ResetROI->setText(QApplication::translate("MainWindow", "Reset", 0));
        label_2->setText(QApplication::translate("MainWindow", "Tracker", 0));
        TrackerStart->setText(QApplication::translate("MainWindow", "Start", 0));
        TrackerStop->setText(QApplication::translate("MainWindow", "Stop", 0));
        ResetTracker->setText(QApplication::translate("MainWindow", "Reset", 0));
        SegmentObject->setText(QApplication::translate("MainWindow", "Segment", 0));
        label_4->setText(QApplication::translate("MainWindow", "Pose", 0));
        OptimizePoses->setText(QApplication::translate("MainWindow", "Optimize", 0));
        undoOptimize->setText(QApplication::translate("MainWindow", "... Undo", 0));
        label_5->setText(QApplication::translate("MainWindow", "Object", 0));
        label_7->setText(QApplication::translate("MainWindow", "Session", 0));
        okSegmentation->setText(QApplication::translate("MainWindow", "... Ok", 0));
        SessionAdd->setText(QApplication::translate("MainWindow", "Add", 0));
        SessionAlign->setText(QApplication::translate("MainWindow", "Align", 0));
        SessionClear->setText(QApplication::translate("MainWindow", "Clear", 0));
        SessionOptimize->setText(QApplication::translate("MainWindow", "Optimize", 0));
        OptimizeObject->setText(QApplication::translate("MainWindow", "Optimize", 0));
        label_6->setText(QApplication::translate("MainWindow", "Store for", 0));
        SaveTrackerModel->setText(QApplication::translate("MainWindow", "Tracker", 0));
        SavePointClouds->setText(QApplication::translate("MainWindow", "Recognizer", 0));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
