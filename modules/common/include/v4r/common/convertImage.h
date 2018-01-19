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

#ifndef KP_CONVERT_CLOUDTOIMAGE_HPP
#define KP_CONVERT_CLOUDTOIMAGE_HPP

#include <float.h>
#include <v4r/core/macros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>

namespace v4r {

// DEPRECATED( inline void convertImage(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, cv::Mat &image) );

inline void convertImage(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, cv::Mat &image) {
  image = cv::Mat_<cv::Vec3b>(cloud.height, cloud.width);

  for (unsigned v = 0; v < cloud.height; v++) {
    for (unsigned u = 0; u < cloud.width; u++) {
      cv::Vec3b &cv_pt = image.at<cv::Vec3b>(v, u);
      const pcl::PointXYZRGB &pt = cloud(u, v);

      cv_pt[2] = pt.r;
      cv_pt[1] = pt.g;
      cv_pt[0] = pt.b;
    }
  }
}

inline void setImage(const cv::Mat &image, pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  if (image.rows != (int)cloud.height || image.cols != (int)cloud.width) {
    std::cout << "Upps" << std::endl;
    return;
  }

  for (unsigned v = 0; v < cloud.height; v++) {
    for (unsigned u = 0; u < cloud.width; u++) {
      const cv::Vec3b &cv_pt = image.at<cv::Vec3b>(v, u);
      pcl::PointXYZRGB &pt = cloud(u, v);

      pt.r = cv_pt[2];
      pt.g = cv_pt[1];
      pt.b = cv_pt[0];
    }
  }
}

}  //--END--

#endif
