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
#include <QSettings>
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

  settings_file = "rtmt2";
  loadSettings();
}

Params::~Params() {
  saveSettings();

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
std::string Params::getVGNfile() {
  return ui->editVGNfile->text().toStdString();
}
std::string Params::getCRFfile() {
  return ui->editCRFfile->text().toStdString();
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

void Params::on_pushFindVGNfile_pressed() {
  QString filename = QFileDialog::getOpenFileName(this, tr("VGN Filename"));
  if (filename.size() != 0) {
    ui->editVGNfile->setText(filename);
    emit vignetting_calibration_file_changed();
  }
}

void Params::on_pushFindCRFfile_pressed() {
  QString filename = QFileDialog::getOpenFileName(this, tr("CRF Filename"));
  if (filename.size() != 0) {
    ui->editCRFfile->setText(filename);
    emit vignetting_calibration_file_changed();
  }
}

void Params::on_okButton_clicked() {
  apply_params();
}

void Params::on_applyButton_clicked() {
  apply_cam_params();
}

void Params::loadSettings() {
  QSettings settings(settings_file);

  if (settings.contains("store_cloud"))
    ui->cb_store_cloud->setChecked(settings.value("store_cloud", "").toBool());
  if (settings.contains("store_views"))
    ui->cb_store_views->setChecked(settings.value("store_views", "").toBool());
  if (settings.contains("store_mesh"))
    ui->cb_store_mesh->setChecked(settings.value("store_mesh", "").toBool());
  if (settings.contains("store_tex_mesh"))
    ui->cb_store_tex_mesh->setChecked(settings.value("store_tex_mesh", "").toBool());
  if (settings.contains("store_tracker"))
    ui->cb_store_tracker->setChecked(settings.value("store_tracker", "").toBool());

  if (settings.contains("nb_frames_ba"))
    ui->nb_frames_ba->setText(settings.value("nb_frames_ba", "").toString());
  if (settings.contains("nb_frames_temp_filt"))
    ui->nb_frames_temp_filt->setText(settings.value("nb_frames_temp_filt", "").toString());
  if (settings.contains("min_cam_motion_kf"))
    ui->min_cam_motion_kf->setText(settings.value("min_cam_motion_kf", "").toString());
  if (settings.contains("min_cam_rot_kf"))
    ui->min_cam_rot_kf->setText(settings.value("min_cam_rot_kf", "").toString());

  if (settings.contains("voxel_grid_size"))
    ui->voxel_grid_size->setText(settings.value("voxel_grid_size", "").toString());
  if (settings.contains("poisson_depth"))
    ui->poisson_depth->setText(settings.value("poisson_depth", "").toString());
  if (settings.contains("poisson_samples"))
    ui->poisson_samples->setText(settings.value("poisson_samples", "").toString());
  if (settings.contains("filter_largest_cluster"))
    ui->filter_largest_cluster->setChecked(settings.value("filter_largest_cluster", "").toBool());

  if (settings.contains("multiview_icp"))
    ui->multiview_icp->setChecked(settings.value("multiview_icp", "").toBool());
  if (settings.contains("use_nguyen_noise_model"))
    ui->use_nguyen_noise_model->setChecked(settings.value("use_nguyen_noise_model", "").toBool());

  if (settings.contains("roi_scale_xy"))
    ui->roi_scale_xy->setText(settings.value("roi_scale_xy", "").toString());
  if (settings.contains("roi_scale_height"))
    ui->roi_scale_height->setText(settings.value("roi_scale_height", "").toString());
  if (settings.contains("roi_offs"))
    ui->roi_offs->setText(settings.value("roi_offs", "").toString());

  if (settings.contains("fuRgbEdit"))
    ui->fuRgbEdit->setText(settings.value("fuRgbEdit", "").toString());
  if (settings.contains("fvRgbEdit"))
    ui->fvRgbEdit->setText(settings.value("fvRgbEdit", "").toString());
  if (settings.contains("cuRgbEdit"))
    ui->cuRgbEdit->setText(settings.value("cuRgbEdit", "").toString());
  if (settings.contains("cvRgbEdit"))
    ui->cvRgbEdit->setText(settings.value("cvRgbEdit", "").toString());

  if (settings.contains("editVGNfile"))
    ui->editVGNfile->setText(settings.value("editVGNfile", "").toString());
  if (settings.contains("editCRFfile"))
    ui->editCRFfile->setText(settings.value("editCRFfile", "").toString());

  if (settings.contains("editRGBDPath"))
    ui->editRGBDPath->setText(settings.value("editRGBDPath", "").toString());

  cout << "Loaded settings from: " << settings.fileName().toStdString() << endl;
}

void Params::saveSettings() {
  QSettings settings(settings_file);

  settings.setValue("store_cloud", ui->cb_store_cloud->isChecked());
  settings.setValue("store_views", ui->cb_store_views->isChecked());
  settings.setValue("store_mesh", ui->cb_store_mesh->isChecked());
  settings.setValue("store_tex_mesh", ui->cb_store_tex_mesh->isChecked());
  settings.setValue("store_tracker", ui->cb_store_tracker->isChecked());

  settings.setValue("nb_frames_ba", ui->nb_frames_ba->text());
  settings.setValue("nb_frames_temp_filt", ui->nb_frames_temp_filt->text());
  settings.setValue("min_cam_motion_kf", ui->min_cam_motion_kf->text());
  settings.setValue("min_cam_rot_kf", ui->min_cam_rot_kf->text());

  settings.setValue("voxel_grid_size", ui->voxel_grid_size->text());
  settings.setValue("poisson_depth", ui->poisson_depth->text());
  settings.setValue("poisson_samples", ui->poisson_samples->text());
  settings.setValue("filter_largest_cluster", ui->filter_largest_cluster->isChecked());

  settings.setValue("multiview_icp", ui->multiview_icp->isChecked());
  settings.setValue("use_nguyen_noise_model", ui->use_nguyen_noise_model->isChecked());

  settings.setValue("roi_scale_xy", ui->roi_scale_xy->text());
  settings.setValue("roi_scale_height", ui->roi_scale_height->text());
  settings.setValue("roi_offs", ui->roi_offs->text());

  settings.setValue("fuRgbEdit", ui->fuRgbEdit->text());
  settings.setValue("fvRgbEdit", ui->fvRgbEdit->text());
  settings.setValue("cuRgbEdit", ui->cuRgbEdit->text());
  settings.setValue("cvRgbEdit", ui->cvRgbEdit->text());

  settings.setValue("editVGNfile", ui->editVGNfile->text());
  settings.setValue("editCRFfile", ui->editCRFfile->text());

  settings.setValue("editRGBDPath", ui->editRGBDPath->text());

  cout << "Stored settings to: " << settings.fileName().toStdString() << endl;
}
