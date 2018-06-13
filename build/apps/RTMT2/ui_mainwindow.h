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
    QGridLayout *gridLayout_2;
    QPushButton *Reset;
    QPushButton *CreateAndSaveModel;
    QWidget *layoutWidget1;
    QGridLayout *gridLayout;
    QLabel *label;
    QPushButton *CamStart;
    QPushButton *CamStop;
    QLabel *label_3;
    QPushButton *setROI;
    QPushButton *ResetROI;
    QLabel *label_2;
    QPushButton *TrackerStart;
    QPushButton *TrackerStop;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->setEnabled(true);
        MainWindow->resize(907, 551);
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
        layoutWidget5->setGeometry(QRect(694, 336, 166, 136));
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
        ShowDepthMask->setEnabled(true);
        ShowDepthMask->setChecked(false);

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
        ResetView->setGeometry(QRect(720, 290, 129, 27));
        layoutWidget = new QWidget(centralWidget);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(685, 160, 188, 71));
        gridLayout_2 = new QGridLayout(layoutWidget);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        Reset = new QPushButton(layoutWidget);
        Reset->setObjectName(QStringLiteral("Reset"));

        gridLayout_2->addWidget(Reset, 1, 0, 1, 1);

        CreateAndSaveModel = new QPushButton(layoutWidget);
        CreateAndSaveModel->setObjectName(QStringLiteral("CreateAndSaveModel"));
        CreateAndSaveModel->setEnabled(true);

        gridLayout_2->addWidget(CreateAndSaveModel, 0, 0, 1, 1);

        layoutWidget1 = new QWidget(centralWidget);
        layoutWidget1->setObjectName(QStringLiteral("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(660, 30, 237, 95));
        gridLayout = new QGridLayout(layoutWidget1);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(layoutWidget1);
        label->setObjectName(QStringLiteral("label"));

        gridLayout->addWidget(label, 0, 0, 1, 1);

        CamStart = new QPushButton(layoutWidget1);
        CamStart->setObjectName(QStringLiteral("CamStart"));

        gridLayout->addWidget(CamStart, 0, 1, 1, 1);

        CamStop = new QPushButton(layoutWidget1);
        CamStop->setObjectName(QStringLiteral("CamStop"));

        gridLayout->addWidget(CamStop, 0, 2, 1, 1);

        label_3 = new QLabel(layoutWidget1);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout->addWidget(label_3, 1, 0, 1, 1);

        setROI = new QPushButton(layoutWidget1);
        setROI->setObjectName(QStringLiteral("setROI"));

        gridLayout->addWidget(setROI, 1, 1, 1, 1);

        ResetROI = new QPushButton(layoutWidget1);
        ResetROI->setObjectName(QStringLiteral("ResetROI"));

        gridLayout->addWidget(ResetROI, 1, 2, 1, 1);

        label_2 = new QLabel(layoutWidget1);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout->addWidget(label_2, 2, 0, 1, 1);

        TrackerStart = new QPushButton(layoutWidget1);
        TrackerStart->setObjectName(QStringLiteral("TrackerStart"));

        gridLayout->addWidget(TrackerStart, 2, 1, 1, 1);

        TrackerStop = new QPushButton(layoutWidget1);
        TrackerStop->setObjectName(QStringLiteral("TrackerStop"));

        gridLayout->addWidget(TrackerStop, 2, 2, 1, 1);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 907, 25));
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
        Reset->setText(QApplication::translate("MainWindow", "Clear Memory", 0));
        CreateAndSaveModel->setText(QApplication::translate("MainWindow", "Create and Store Models", 0));
        label->setText(QApplication::translate("MainWindow", "Camera", 0));
        CamStart->setText(QApplication::translate("MainWindow", "Start", 0));
        CamStop->setText(QApplication::translate("MainWindow", "Stop", 0));
        label_3->setText(QApplication::translate("MainWindow", "ROI", 0));
        setROI->setText(QApplication::translate("MainWindow", "Set", 0));
        ResetROI->setText(QApplication::translate("MainWindow", "Reset", 0));
        label_2->setText(QApplication::translate("MainWindow", "Tracker", 0));
        TrackerStart->setText(QApplication::translate("MainWindow", "Start", 0));
        TrackerStop->setText(QApplication::translate("MainWindow", "Stop", 0));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
