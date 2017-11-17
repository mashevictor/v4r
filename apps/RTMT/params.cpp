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
 * @file params.cpp
 * @author Johann Prankl (prankl@acin.tuwien.ac.at), Aitor Aldoma (aldoma@acin.tuwien.ac.at)
 * @date 2015
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

void Params::apply_params() {
  cam_tracker_params.log_point_clouds = ui->logPointClouds->isChecked();
  cam_tracker_params.create_prev_cloud = ui->createPreviewCloud->isChecked();
  cam_tracker_params.min_delta_angle = ui->logDeltaAngle->text().toFloat();
  cam_tracker_params.min_delta_distance = ui->logDeltaDistance->text().toFloat();
  cam_tracker_params.prev_voxegrid_size = ui->prevVoxelGridSize->text().toFloat();
  cam_tracker_params.prev_z_cutoff = ui->prevZCutOff->text().toFloat();

  ba_params.dist_cam_add_projections = ui->distCamAddProjections->text().toFloat();
  seg_params.inl_dist_plane = ui->inlDistPlane->text().toFloat();
  seg_params.thr_angle = ui->thrAngle->text().toFloat();
  seg_params.min_points_plane = ui->minPointsPlane->text().toFloat();
  om_params.vx_size_object = ui->vxSizeObject->text().toFloat();

  emit cam_tracker_params_changed(cam_tracker_params);
  emit bundle_adjustment_parameter_changed(ba_params);
  emit segmentation_parameter_changed(seg_params);
  emit object_modelling_parameter_changed(om_params);
  emit set_roi_params(ui->roi_scale_xy->text().toFloat(), ui->roi_scale_height->text().toFloat(),
                      ui->roi_offs->text().toFloat());
  emit set_segmentation_params(ui->use_roi_segm->isChecked(), ui->roi_offs->text().toFloat(),
                               ui->use_dense_mv->isChecked(), ui->edge_radius_px->text().toFloat());
  emit set_cb_param(ui->model_create_cb->isChecked(), ui->model_rnn_thr->text().toFloat());
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
