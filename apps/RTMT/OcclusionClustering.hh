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
 * @file OcclusionClustering.hh
 * @author Johann Prankl@acin.tuwien.ac.at)
 * @date 2016
 * @brief
 *
 */

#ifndef KP_OCCLUSION_CLUSTERING_HH
#define KP_OCCLUSION_CLUSTERING_HH

#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#ifndef Q_MOC_RUN
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#endif


namespace v4r
{



/**
 * OcclusionClustering
 */
class OcclusionClustering
{
public:

  /**
   * @brief The Parameter class
   */
  class Parameter
  {
  public:
    double thr_std_dev;
    Parameter()
      : thr_std_dev(0.02) {}
  };


private:
  std::vector<cv::Point> queue;
  std::vector<Eigen::Vector3f> contour;
  std::vector<cv::Point> points;
  std::vector<cv::Point> nbs;

  void clusterNaNs(const cv::Point &_start, const pcl::PointCloud<pcl::PointXYZRGB> &_cloud, cv::Mat_<unsigned char> &_mask, std::vector<Eigen::Vector3f> &_contour, std::vector<cv::Point> &_points);
  double getDepthVariance(const std::vector<Eigen::Vector3f> &_contour);

  inline bool isnan(const pcl::PointXYZRGB &pt);

public:
  Parameter param;
  OcclusionClustering(const Parameter &_p=Parameter());
  ~OcclusionClustering();

  /** get occlusion mask (occlusion from stereo) **/
  void compute(const pcl::PointCloud<pcl::PointXYZRGB> &_cloud, cv::Mat_<unsigned char> &_mask);

  typedef boost::shared_ptr< ::v4r::OcclusionClustering> Ptr;
  typedef boost::shared_ptr< ::v4r::OcclusionClustering const> ConstPtr;
};


/**
 * @brief OcclusionClustering::isnan
 * @param pt
 * @return
 */
inline bool OcclusionClustering::isnan(const pcl::PointXYZRGB &pt)
{
  if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))
    return true;
  return false;
}

}

#endif

