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

#ifndef _GRAB_PCD_QT_MAINWINDOW_H_
#define _GRAB_PCD_QT_MAINWINDOW_H_

#ifndef Q_MOC_RUN
#include <QInputDialog>
#include <QMainWindow>
#include "ObjectSegmentation.h"
#include "glviewer.h"
#include "params.h"
#include "sensor.h"

#undef Success
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "OctreeVoxelCentroidContainerXYZRGB.hpp"
#endif

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

 signals:
  void set_image(int idx);

 private slots:

  void on_actionPreferences_triggered();
  void on_actionExit_triggered();

  void on_CamStart_clicked();
  void on_CamStop_clicked();
  void on_TrackerStart_clicked();
  void on_TrackerStop_clicked();

  void on_CreateAndSaveModel_clicked();
  void on_Reset_clicked();

  void on_ResetView_clicked();

  void on_ShowImage_clicked();
  void on_ShowCameras_clicked();
  void on_ShowPointCloud_clicked();
  void on_ShowObjectModel_clicked();
  void on_imForward_clicked();
  void on_imBackward_clicked();

  void finishedModelling();

  void on_ShowDepthMask_clicked();
  void on_setROI_clicked();
  void on_ResetROI_clicked();

 public slots:
  void printStatus(const std::string &_txt);

 private:
  //  void make_extension(std::string& filename, std::string ext);

  Ui::MainWindow *m_ui;
  GLViewer *m_glviewer;
  GLGraphicsView *m_glview;
  Params *m_params;
  Sensor *m_sensor;
  ObjectSegmentation *m_segmentation;

  bool have_multi_session;

  size_t m_num_saves_disp;
  size_t m_num_saves_pcd;
  int idx_seg, num_clouds;
  bool bbox_active;

  void activateAllButtons();
  void deactivateAllButtons();

  void setStartVis();
};

#endif  // MAINWINDOW_H
