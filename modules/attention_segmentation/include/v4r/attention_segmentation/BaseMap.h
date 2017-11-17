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

#ifndef BASE_MAP_HPP
#define BASE_MAP_HPP

#include <v4r/core/macros.h>
#include "v4r/attention_segmentation/headers.h"

#include "v4r/attention_segmentation/pyramidBase.h"
#include "v4r/attention_segmentation/pyramidFrintrop.h"
#include "v4r/attention_segmentation/pyramidItti.h"
#include "v4r/attention_segmentation/pyramidSimple.h"

namespace v4r {

enum PyramidType {
  SIMPLE_PYRAMID = 0,
  ITTI_PYRAMID = 1,
  FRINTROP_PYRAMID,
};

class V4R_EXPORTS BaseMap {
 public:
  BaseMap();
  virtual ~BaseMap();

  void setImage(const cv::Mat &image_);
  bool getImage(cv::Mat &image_);

  void setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_);
  bool getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_);

  void setNormals(pcl::PointCloud<pcl::Normal>::Ptr normals_);
  bool getNormals(pcl::PointCloud<pcl::Normal>::Ptr &normals_);

  void setIndices(pcl::PointIndices::Ptr indices_);
  bool getIndices(pcl::PointIndices::Ptr &indices_);

  void setMask(const cv::Mat &mask_);
  bool getMask(cv::Mat &mask_);

  void setNormalizationType(int normalization_type_);
  int getNormalizationType();

  void setCombinationType(int combination_type_);
  int getCombinationType();

  void setFilterSize(int filter_size_);
  int getFilterSize();

  void setWidth(int width_);
  int getWidth();

  void setHeight(int height_);
  int getHeight();

  void setRefine(bool refine_);
  bool getRefine();

  void setMapName(std::string mapName_);
  std::string getMapName();

  virtual int calculate() = 0;
  virtual int calculatePyramid(int pyramidType = SIMPLE_PYRAMID);

  bool getMap(cv::Mat &map_);

  virtual void reset();
  virtual void print();

 protected:
  /**
   * parameters for base map
   * */

  cv::Mat mask;                                  //
  cv::Mat image;                                 //
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;  //
  pcl::PointCloud<pcl::Normal>::Ptr normals;     //
  pcl::PointIndices::Ptr indices;                //
  int filter_size;                               //
  int normalization_type;                        //
  int combination_type;                          //
  int width;                                     //
  int height;                                    //
  bool calculated;                               //
  cv::Mat map;                                   //
  bool refine;
  // BasePyramid::Ptr                         pyramid;

  std::string mapName;

  bool haveImage;
  bool haveCloud;
  bool haveNormals;
  bool haveIndices;
  bool haveMask;

  virtual int checkParameters();

  virtual int calculatePyramidSimple();
  virtual int calculatePyramidItti();
  virtual int calculatePyramidFrintrop();

  virtual int combinePyramid(BasePyramid::Ptr pyramid);

  virtual void refineMap();
};

}  // namespace v4r

#endif  // BASE_MAP_HPP
