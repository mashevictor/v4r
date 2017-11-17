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

#ifndef PYRAMID_BASE_HPP
#define PYRAMID_BASE_HPP

#include <v4r/core/macros.h>
#include "v4r/attention_segmentation/headers.h"

namespace v4r {

class V4R_EXPORTS BasePyramid {
 public:
  BasePyramid();
  typedef boost::shared_ptr<BasePyramid> Ptr;

  void setStartLevel(int start_level_);
  int getStartLevel();

  void setMaxLevel(int max_level_);
  int getMaxLevel();

  void setSMLevel(int sm_level_);
  int getSMLevel();

  void setNormalizationType(int normalization_type_);
  int getNormalizationType();

  void setWidth(int width_);
  int getWidth();
  int getWidth(unsigned int level);

  void setHeight(int height_);
  int getHeight();
  int getHeight(unsigned int level);

  void setCombinationType(int combination_type_);
  int getCombinationType();

  void setImage(cv::Mat &image_);
  bool getImage(cv::Mat &image_);
  bool getImage(unsigned int level, cv::Mat &image_);

  void setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_);
  bool getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_);
  bool getCloud(unsigned int level, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_);

  void setIndices(pcl::PointIndices::Ptr &indices_);
  bool getIndices(pcl::PointIndices::Ptr &indices_);
  bool getIndices(unsigned int level, pcl::PointIndices::Ptr &indices_);

  void setNormals(pcl::PointCloud<pcl::Normal>::Ptr &normals_);
  bool getNormals(pcl::PointCloud<pcl::Normal>::Ptr &normals_);
  bool getNormals(unsigned int level, pcl::PointCloud<pcl::Normal>::Ptr &normals_);

  void setMaxMapValue(float max_map_value_);
  float getMaxMapValue();

  std::vector<cv::Mat> getPyramidImages();
  std::vector<cv::Mat> getPyramidFeatures();
  bool setFeatureMap(unsigned int level, cv::Mat &featureMap_);

  bool getMap(cv::Mat &map_);

  virtual void print();
  virtual void reset();

  virtual int checkParameters(bool isDepth = false);

  int buildPyramid();
  int buildDepthPyramid();
  virtual void combinePyramid(bool standard = false);

 protected:
  int start_level;
  int max_level;
  int sm_level;
  int normalization_type;
  int width;
  int height;
  int combination_type;

  cv::Mat image;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  pcl::PointIndices::Ptr indices;
  pcl::PointCloud<pcl::Normal>::Ptr normals;

  std::vector<cv::Mat> pyramidImages;
  std::vector<cv::Mat> pyramidX;
  std::vector<cv::Mat> pyramidY;
  std::vector<cv::Mat> pyramidZ;
  std::vector<cv::Mat> pyramidNx;
  std::vector<cv::Mat> pyramidNy;
  std::vector<cv::Mat> pyramidNz;
  std::vector<pcl::PointIndices::Ptr> pyramidIndices;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pyramidCloud;
  std::vector<pcl::PointCloud<pcl::Normal>::Ptr> pyramidNormals;

  std::vector<cv::Mat> pyramidFeatures;
  float max_map_value;
  cv::Mat map;

  bool calculated;

  bool haveImage;
  bool haveCloud;
  bool haveIndices;
  bool haveNormals;
  bool haveImagePyramid;
  bool haveDepthPyramid;
  bool haveNormalPyramid;
  bool haveIndicePyramid;

  std::string pyramidName;

  virtual void calculate();
  virtual void checkLevels();
  virtual void combineConspicuityMaps(cv::Mat &sm_map, cv::Mat &consp_map);
};
}
#endif  // PYRAMID_BASE_HPP
