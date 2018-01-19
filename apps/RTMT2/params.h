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

#ifndef _GRAB_PCD_PARAMS_H_
#define _GRAB_PCD_PARAMS_H_

#ifndef Q_MOC_RUN
#include <Eigen/Dense>
#include <QDialog>
#include <opencv2/core/core.hpp>
#endif

namespace Ui {
class Params;
}

class RGBDCameraParameter {
 public:
  double f[2];
  double c[2];
  int width, height;
  double range[2];
  RGBDCameraParameter() {
    f[0] = 525;
    f[1] = 525;
    c[0] = 320;
    c[1] = 240;
    width = 640;
    height = 480;
    range[0] = 0.1;
    range[1] = 3.14;
  }
};

/**
 * @brief The Params class
 */
class Params : public QDialog {
  Q_OBJECT

 public:
  //! Constructor.
  explicit Params(QWidget *parent = 0);

  //! Destructor.
  ~Params();

  void apply_params();
  void apply_cam_params();
  std::string get_rgbd_path();
  std::string get_object_name();
  std::string getVGNfile();
  std::string getCRFfile();
  void set_object_name(const QString &txt);
  bool createPointCloud();
  bool createViews();
  bool createMesh();
  bool createTexturedMesh();
  bool createTrackingModel();
  int getTsfNbFramesBA();
  int getTsfBatchSize();
  double getMinCamMotionKF();
  double getMinCamRotatationKF();
  double getVoxelGridSize();
  int getPoissonDepth();
  int getPoissonSamples();
  bool filterLargestCluster();
  bool useMultiviewICP();
  bool useNoiseModel();

  void saveSettings();
  void loadSettings();

 signals:

  void cam_params_changed(const RGBDCameraParameter &cam);
  void rgbd_path_changed();
  void set_roi_params(const double &_bbox_scale_xy, const double &_bbox_scale_height, const double &_seg_offs);
  void vignetting_calibration_file_changed();

 private slots:

  void on_okButton_clicked();
  void on_pushFindRGBDPath_pressed();
  void on_pushFindVGNfile_pressed();
  void on_pushFindCRFfile_pressed();

  void on_applyButton_clicked();

 private:
  Ui::Params *ui;

  RGBDCameraParameter cam_params;

  QString settings_file;
};

#endif  // PARAMS_H
