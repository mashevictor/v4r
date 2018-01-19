/****************************************************************************
**
** Copyright (C) 2017 TU Wien, ACIN, Vision 4 Robotics (V4R) group
** Contact: v4r.acin.tuwien.ac.at
**
** This file is part of V4R
**
** V4R is distributed under dual licenses - GPLv3 or closed source.
**
** GNU General Public License Usage
** V4R is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published
** by the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** V4R is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** Please review the following information to ensure the GNU General Public
** License requirements will be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
**
** Commercial License Usage
** If GPL is not suitable for your project, you must purchase a commercial
** license to use V4R. Licensees holding valid commercial V4R licenses may
** use this file in accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the
** terms contained in a written agreement between you and TU Wien, ACIN, V4R.
** For licensing terms and conditions please contact office<at>acin.tuwien.ac.at.
**
**
** The copyright holder additionally grants the author(s) of the file the right
** to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of their contributions without any restrictions.
**
****************************************************************************/

/**
 * @file main.cpp
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#ifndef Q_MOC_RUN
#include "mainwindow.h"
#include <v4r/keypoints/impl/toString.hpp>
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QMessageBox>
#endif

Q_DECLARE_METATYPE(cv::Mat)

using namespace std;

MainWindow::MainWindow(QWidget *parent)
: QMainWindow(parent), m_ui(new Ui::MainWindow), m_params(new Params(this)), m_sensor(new Sensor()),
  m_segmentation(new ObjectSegmentation()), is_model_stored(true), m_num_saves_disp(0), m_num_saves_pcd(0), idx_seg(-1),
  num_clouds(0), bbox_active(false) {
  m_ui->setupUi(this);

  m_glviewer = new GLViewer(this);
  m_glview = new GLGraphicsView(m_ui->centralWidget);
  m_ui->glView = m_glview;
  m_glview->setGeometry(10, 0, 640, 480);
  m_glview->setViewport(m_glviewer);

  // input signals
  connect(m_glview, SIGNAL(mouse_moved(QMouseEvent *)), m_glviewer, SLOT(mouse_moved(QMouseEvent *)));
  connect(m_glview, SIGNAL(mouse_pressed(QMouseEvent *)), m_glviewer, SLOT(mouse_pressed(QMouseEvent *)));
  connect(m_glview, SIGNAL(key_pressed(QKeyEvent *)), m_glviewer, SLOT(key_pressed(QKeyEvent *)));
  connect(m_glview, SIGNAL(wheel_event(QWheelEvent *)), m_glviewer, SLOT(wheel_event(QWheelEvent *)));

  // param signals
  connect(m_params, SIGNAL(cam_params_changed(const RGBDCameraParameter)), m_glviewer,
          SLOT(cam_params_changed(const RGBDCameraParameter)));
  connect(m_params, SIGNAL(cam_params_changed(const RGBDCameraParameter)), m_sensor,
          SLOT(cam_params_changed(const RGBDCameraParameter)));
  connect(m_params, SIGNAL(set_roi_params(const double, const double, const double)), m_sensor,
          SLOT(set_roi_params(const double, const double, const double)));
  connect(m_params, SIGNAL(set_roi_params(const double, const double, const double)), m_segmentation,
          SLOT(set_roi_params(const double, const double, const double)));

  // sensor signals
  qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>("pcl::PointCloud<pcl::PointXYZRGB>::Ptr");
  qRegisterMetaType<cv::Mat_<cv::Vec3b>>("cv::Mat_<cv::Vec3b>");
  qRegisterMetaType<boost::shared_ptr<std::vector<Sensor::CameraLocation>>>(
      "boost::shared_ptr< std::vector<Sensor::CameraLocation> >");
  qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>("pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr");
  qRegisterMetaType<std::string>("std::string");
  qRegisterMetaType<std::vector<Eigen::Vector3f>>("std::vector<Eigen::Vector3f>");
  qRegisterMetaType<Eigen::Matrix4f>("Eigen::Matrix4f");
  qRegisterMetaType<Eigen::Vector3f>("Eigen::Vector3f");

  connect(m_sensor, SIGNAL(new_image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const cv::Mat_<cv::Vec3b>)),
          m_glviewer, SLOT(new_image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const cv::Mat_<cv::Vec3b>)));
  connect(m_sensor, SIGNAL(update_model_cloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr)), m_glviewer,
          SLOT(update_model_cloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr)));
  connect(m_sensor, SIGNAL(update_cam_trajectory(const boost::shared_ptr<std::vector<Sensor::CameraLocation>>)),
          m_glviewer, SLOT(update_cam_trajectory(const boost::shared_ptr<std::vector<Sensor::CameraLocation>>)));
  connect(m_sensor, SIGNAL(update_visualization()), m_glviewer, SLOT(update_visualization()));
  connect(m_sensor, SIGNAL(printStatus(const std::string)), this, SLOT(printStatus(const std::string)));
  connect(m_glviewer, SIGNAL(select_roi(int, int)), m_sensor, SLOT(select_roi(int, int)));

  connect(m_segmentation, SIGNAL(update_model_cloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr)), m_glviewer,
          SLOT(update_model_cloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr)));
  connect(m_segmentation, SIGNAL(new_image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const cv::Mat_<cv::Vec3b>)),
          m_glviewer, SLOT(new_image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const cv::Mat_<cv::Vec3b>)));
  connect(m_segmentation, SIGNAL(printStatus(const std::string)), this, SLOT(printStatus(const std::string)));
  connect(m_segmentation, SIGNAL(update_visualization()), m_glviewer, SLOT(update_visualization()));
  connect(m_segmentation, SIGNAL(finishedModelling()), this, SLOT(finishedModelling()));

  // object segmentation
  connect(m_sensor, SIGNAL(update_boundingbox(const std::vector<Eigen::Vector3f>, const Eigen::Matrix4f)), m_glviewer,
          SLOT(update_boundingbox(const std::vector<Eigen::Vector3f>, const Eigen::Matrix4f)));
  connect(m_params, SIGNAL(vignetting_calibration_file_changed()), this, SLOT(vignetting_calibration_file_changed()));

  m_params->apply_cam_params();
  m_params->apply_params();

  setWindowTitle(tr("RTM-Toolbox"));
}

MainWindow::~MainWindow() {
  delete m_ui;
  delete m_params;
  delete m_glviewer;
  delete m_glview;
  delete m_sensor;
  delete m_segmentation;
}

/**
 * @brief MainWindow::activateAllButtons
 */
void MainWindow::activateAllButtons() {
  m_ui->CamStart->setEnabled(true);
  m_ui->TrackerStart->setEnabled(true);
  m_ui->CamStop->setEnabled(true);
  m_ui->TrackerStop->setEnabled(true);

  m_ui->CreateAndSaveModel->setEnabled(true);
  m_ui->Reset->setEnabled(true);
  m_ui->setROI->setEnabled(true);
  m_ui->ResetROI->setEnabled(true);
}

/**
 * @brief MainWindow::deactivateAllButtons
 */
void MainWindow::deactivateAllButtons() {
  m_ui->CamStart->setEnabled(false);
  m_ui->TrackerStart->setEnabled(false);
  m_ui->CamStop->setEnabled(false);
  m_ui->TrackerStop->setEnabled(false);

  m_ui->CreateAndSaveModel->setEnabled(false);
  m_ui->Reset->setEnabled(false);
  m_ui->setROI->setEnabled(false);
  m_ui->ResetROI->setEnabled(false);
}

/**
 * @brief MainWindow::setStartVis
 */
void MainWindow::setStartVis() {
  m_ui->ShowImage->setChecked(true);
  m_ui->ShowDepthMask->setChecked(false);
  m_ui->ShowCameras->setChecked(false);
  m_ui->ShowPointCloud->setChecked(false);
  m_ui->ShowObjectModel->setChecked(false);

  m_glviewer->showImage(m_ui->ShowImage->isChecked());
  m_glviewer->showCameras(m_ui->ShowCameras->isChecked());
  m_glviewer->showCloud(m_ui->ShowPointCloud->isChecked());
  m_glviewer->showObject(m_ui->ShowObjectModel->isChecked());
  m_sensor->showDepthMask(m_ui->ShowDepthMask->isChecked());

  m_glviewer->resetView();
}

void MainWindow::printStatus(const std::string &_txt) {
  m_ui->statusLabel->setText(_txt.c_str());
}

void MainWindow::vignetting_calibration_file_changed() {
  if (m_params->getVGNfile().size() > 0 && m_params->getCRFfile().size() > 0) {
    m_segmentation->setCRFfile(m_params->getCRFfile());
    m_sensor->setVignettingCalibrationFiles(m_params->getVGNfile(), m_params->getCRFfile());
  }
}

void MainWindow::on_actionPreferences_triggered() {
  m_params->show();
}

void MainWindow::on_actionExit_triggered() {
  if (!is_model_stored) {
    QMessageBox::StandardButton res_btn =
        QMessageBox::question(this, "Exit dialog", tr("You did not store the model! Are you sure?"),
                              QMessageBox::Cancel | QMessageBox::No | QMessageBox::Yes, QMessageBox::Yes);
    if (res_btn == QMessageBox::Yes)
      QApplication::exit();
  } else
    QApplication::exit();
}

void MainWindow::on_CamStart_clicked() {
  if (m_params->getVGNfile().size() > 0 && m_params->getCRFfile().size() > 0) {
    m_segmentation->setCRFfile(m_params->getCRFfile());
    m_sensor->setVignettingCalibrationFiles(m_params->getVGNfile(), m_params->getCRFfile());
  }

  setStartVis();

  m_sensor->start(0);
  m_ui->statusLabel->setText("Status: Started camera");
}

void MainWindow::on_CamStop_clicked() {
  m_sensor->stop();

  activateAllButtons();

  m_glviewer->drawBoundingBox(false);

  m_ui->statusLabel->setText("Status: Stopped camera");
}

void MainWindow::on_TrackerStart_clicked() {
  if (!m_sensor->isRunning())
    setStartVis();

  m_sensor->setTSFParameter(m_params->getTsfNbFramesBA(), m_params->getTsfBatchSize(), m_params->getMinCamMotionKF(),
                            m_params->getMinCamRotatationKF());

  m_sensor->startTracker(0);

  // m_glviewer->drawBoundingBox(false);
  m_ui->statusLabel->setText("Status: Started tracker");

  is_model_stored = false;
}

void MainWindow::on_TrackerStop_clicked() {
  m_sensor->stopTracker();

  activateAllButtons();
  m_glviewer->drawBoundingBox(false);

  m_ui->statusLabel->setText("Status: Stopped tracker");
}

void MainWindow::on_imForward_clicked() {
  if (idx_seg < num_clouds - 1)
    idx_seg++;
  emit set_image(idx_seg);
}

void MainWindow::on_imBackward_clicked() {
  if (idx_seg > 0)
    idx_seg--;
  emit set_image(idx_seg);
}

void MainWindow::on_ResetView_clicked() {
  m_glviewer->resetView();
}

void MainWindow::on_ShowImage_clicked() {
  m_glviewer->showImage(m_ui->ShowImage->isChecked());
}

void MainWindow::on_ShowCameras_clicked() {
  m_glviewer->showCameras(m_ui->ShowCameras->isChecked());
  m_sensor->showCameras(m_ui->ShowCameras->isChecked());
}

void MainWindow::on_ShowPointCloud_clicked() {
  m_glviewer->showCloud(m_ui->ShowPointCloud->isChecked());
}

void MainWindow::on_ShowObjectModel_clicked() {
  m_glviewer->showObject(m_ui->ShowObjectModel->isChecked());
}

/**
 * @brief MainWindow::finishedObjectSegmentation
 */
void MainWindow::finishedModelling() {
  m_glviewer->drawBoundingBox(false);

  activateAllButtons();

  m_ui->ShowImage->setChecked(false);
  m_ui->ShowDepthMask->setChecked(false);
  m_ui->ShowCameras->setChecked(false);
  m_ui->ShowPointCloud->setChecked(false);
  m_ui->ShowObjectModel->setChecked(true);

  m_glviewer->showImage(m_ui->ShowImage->isChecked());
  m_glviewer->showCameras(m_ui->ShowCameras->isChecked());
  m_glviewer->showCloud(m_ui->ShowPointCloud->isChecked());
  m_glviewer->showObject(m_ui->ShowObjectModel->isChecked());
  m_sensor->showDepthMask(m_ui->ShowDepthMask->isChecked());
  m_glviewer->resetView(-1.);

  m_ui->imBackward->setEnabled(false);
  m_ui->imForward->setEnabled(false);

  m_ui->statusLabel->setText("Status: Finished modelling and stored object models");
}

void MainWindow::on_CreateAndSaveModel_clicked() {
  bool ok;
  QString text = QString::fromStdString(m_params->get_object_name());

  QString object_name =
      QInputDialog::getText(this, tr("Store model"), tr("Object name:"), QLineEdit::Normal, text, &ok);

  if (ok && object_name.isNull() == false) {
    if (boost::filesystem::exists(m_params->get_rgbd_path() + "/" + object_name.toStdString() + "/tracking_model.ao")) {
      int ret = QMessageBox::warning(this, tr("Store model"), tr("The object file exists!\n"
                                                                 "Do you want to overwrite the file?"),
                                     QMessageBox::Save, QMessageBox::Cancel);
      if (ret != QMessageBox::Save)
        ok = false;
    }

    if (ok) {
      m_params->set_object_name(object_name);

      // create model
      m_sensor->stop();
      m_ui->statusLabel->setText("Status: Optimize cameras and create object models... be patient...");
      deactivateAllButtons();
      Eigen::Matrix4f base_transform;
      Eigen::Vector3f bb_min, bb_max;
      cv::Mat intrinsic, dist_coeffs;
      m_segmentation->createPointCloud(m_params->createPointCloud());
      m_segmentation->createViews(m_params->createViews());
      m_segmentation->createMesh(m_params->createMesh());
      m_segmentation->createTexturedMesh(m_params->createTexturedMesh());
      m_segmentation->createTrackingModel(m_params->createTrackingModel());
      m_segmentation->useNoiseModel(m_params->useNoiseModel());
      m_segmentation->useMultiviewICP(m_params->useMultiviewICP());
      m_segmentation->filterLargestCluster(m_params->filterLargestCluster());
      m_segmentation->setParameter(m_params->getVoxelGridSize(), m_params->getPoissonDepth(),
                                   m_params->getPoissonSamples());
      m_sensor->getObjectTransform(base_transform, bb_min, bb_max);
      m_segmentation->setDirectories(m_params->get_rgbd_path(), object_name.toStdString());
      m_segmentation->setData(m_sensor->getMap(), base_transform, bb_min, bb_max);
      m_sensor->getCameraParameter(intrinsic, dist_coeffs);
      m_segmentation->setCameraParameter(intrinsic, dist_coeffs);
      m_segmentation->finishModelling();  // starts the modelling thread

      is_model_stored = true;
    }
  }
}

void MainWindow::on_Reset_clicked() {
  m_sensor->reset();
  m_ui->statusLabel->setText("Status: Reset tracker");
}

void MainWindow::on_ShowDepthMask_clicked() {
  m_sensor->showDepthMask(m_ui->ShowDepthMask->isChecked());
}

void MainWindow::on_setROI_clicked() {
  bbox_active = true;
  m_sensor->start(0);
  // m_glviewer->drawBoundingBox(true);
  m_glviewer->selectROI(true);
  m_ui->statusLabel->setText(
      "For automatic ROI generation click to the supporting surface (e.g. top surface of a turntable)");
}

void MainWindow::on_ResetROI_clicked() {
  bbox_active = false;
  m_glviewer->drawBoundingBox(false);
  m_glviewer->selectROI(false);
  m_sensor->activateROI(0);
}

void MainWindow::closeEvent(QCloseEvent *event) {
  if (!is_model_stored) {
    QMessageBox::StandardButton res_btn =
        QMessageBox::question(this, "Exit dialog", tr("You did not store the model! Are you sure?"),
                              QMessageBox::Cancel | QMessageBox::No | QMessageBox::Yes, QMessageBox::Yes);
    if (res_btn != QMessageBox::Yes)
      event->ignore();
    else
      event->accept();
  }
}
