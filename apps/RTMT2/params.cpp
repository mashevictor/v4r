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
#include "params.h"
#include "ui_params.h"

#include <opencv2/opencv.hpp>

#include <fstream>
#include <vector>

#include <QFileDialog>
#endif

using namespace cv;
using namespace std;

Params::Params(QWidget *parent) : QDialog(parent), ui(new Ui::Params) {
  ui->setupUi(this);
}

Params::~Params() {
  delete ui;
}

bool Params::createPointCloud() {
  return ui->cb_store_cloud->isChecked();
}
bool Params::createViews() {
  return ui->cb_store_views->isChecked();
}
bool Params::createMesh() {
  return ui->cb_store_mesh->isChecked();
}
bool Params::createTexturedMesh() {
  return ui->cb_store_tex_mesh->isChecked();
}
bool Params::createTrackingModel() {
  return ui->cb_store_tracker->isChecked();
}

int Params::getTsfNbFramesBA() {
  return ui->nb_frames_ba->text().toInt();
}
int Params::getTsfBatchSize() {
  return ui->nb_frames_temp_filt->text().toInt();
}
double Params::getMinCamMotionKF() {
  return ui->min_cam_motion_kf->text().toFloat();
}
double Params::getMinCamRotatationKF() {
  return ui->min_cam_rot_kf->text().toFloat();
}

double Params::getVoxelGridSize() {
  return ui->voxel_grid_size->text().toFloat();
}
int Params::getPoissonDepth() {
  return ui->poisson_depth->text().toFloat();
}
int Params::getPoissonSamples() {
  return ui->poisson_samples->text().toFloat();
}
bool Params::filterLargestCluster() {
  return ui->filter_largest_cluster->isChecked();
}
bool Params::useMultiviewICP() {
  return ui->multiview_icp->isChecked();
}
bool Params::useNoiseModel() {
  return ui->use_nguyen_noise_model->isChecked();
}

void Params::apply_params() {
  emit set_roi_params(ui->roi_scale_xy->text().toFloat(), ui->roi_scale_height->text().toFloat(),
                      ui->roi_offs->text().toFloat());
}

void Params::apply_cam_params() {
  cam_params.f[0] = ui->fuRgbEdit->text().toFloat();
  cam_params.f[1] = ui->fvRgbEdit->text().toFloat();
  cam_params.c[0] = ui->cuRgbEdit->text().toFloat();
  cam_params.c[1] = ui->cvRgbEdit->text().toFloat();

  emit cam_params_changed(cam_params);
}

std::string Params::get_rgbd_path() {
  std::string path = ui->editRGBDPath->text().toStdString();

  if (path.empty())
    path += ".";

  return (path);
}

void Params::set_object_name(const QString &txt) {
  ui->editModelName->setText(txt);
}

std::string Params::get_object_name() {
  std::string path = ui->editModelName->text().toStdString();

  if (path.empty())
    path = "objectmodel";

  return (path);
}

void Params::on_pushFindRGBDPath_pressed() {
  QString filename = QFileDialog::getExistingDirectory(this, tr("RGBD Path"), tr("./log"));
  if (filename.size() != 0) {
    ui->editRGBDPath->setText(filename);
    emit rgbd_path_changed();
  }
}

void Params::on_okButton_clicked() {
  apply_params();
}

void Params::on_applyButton_clicked() {
  apply_cam_params();
}
