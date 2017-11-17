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
 * @file EPBase.cpp
 * @author Potapova
 * @date August 2013
 * @version 0.1
 * @brief Base class.
 */

#include "v4r/attention_segmentation/EPBase.h"

namespace v4r {

/************************************************************************************
 * Constructor/Destructor
 */

EPBase::EPBase() {
  have_cloud = false;
  have_normals = false;
  have_indices = false;

  computed = false;

  ClassName = "EPBase";
}

EPBase::~EPBase() {}

void EPBase::setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& _cloud) {
  if ((_cloud->height <= 1) || (_cloud->width <= 1) || (!_cloud->isOrganized())) {
    char* error_message = new char[200];
    sprintf(error_message,
            "[%s::setInputCloud()]: Invalid point cloud (height must be > 1; width must be > 1; point cloud should be "
            "organized).",
            ClassName.c_str());
    throw std::runtime_error(error_message);
  }

  cloud = _cloud;
  width = cloud->width;
  height = cloud->height;

  have_cloud = true;
  have_normals = false;
  have_indices = false;
  computed = false;
}

void EPBase::setNormals(const pcl::PointCloud<pcl::Normal>::Ptr& _normals) {
  if (!have_cloud) {
    char* error_message = new char[200];
    sprintf(error_message, "[%s::setNormals()]: I suggest you first set the point cloud.", ClassName.c_str());
    throw std::runtime_error(error_message);
  }

  if ((_normals->height != (unsigned int)height) || (_normals->width != (unsigned int)width)) {
    char* error_message = new char[200];
    sprintf(error_message, "[%s::setNormals()]: Invalid normals (not for this point cloud).", ClassName.c_str());
    throw std::runtime_error(error_message);
  }

  normals = _normals;
  have_normals = true;
}

void EPBase::setIndices(const pcl::PointIndices::Ptr& _indices) {
  if (!have_cloud) {
    char* error_message = new char[200];
    sprintf(error_message, "[%s::setNormals()]: I suggest you first set the point cloud.", ClassName.c_str());
    throw std::runtime_error(error_message);
  }

  indices = _indices->indices;
  have_indices = true;
}

void EPBase::setIndices(const std::vector<int>& _indices) {
  if (!have_cloud) {
    char* error_message = new char[200];
    sprintf(error_message, "[%s::setNormals()]: I suggest you first set the point cloud.", ClassName.c_str());
    throw std::runtime_error(error_message);
  }

  indices = _indices;
  have_indices = true;
}

void EPBase::compute() {
  char* error_message = new char[200];
  sprintf(error_message, "[%s::setNormals()]: Why do you call me? I am just the base class!.", ClassName.c_str());
  throw std::runtime_error(error_message);
}

}  // end v4r
