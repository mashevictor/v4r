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
 * @file OcclusionClustering.cc
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2016
 * @brief
 *
 */
#include "OcclusionClustering.hh"
#include "v4r/common/impl/Vector.hpp"

namespace v4r
{

using namespace std;
  
/********************** OcclusionClustering ************************
 * Constructor/Destructor
 */
OcclusionClustering::OcclusionClustering(const Parameter &_p)
 : param(_p)
{
  nbs.resize(4);
  nbs[0] = cv::Point(-1,0);
  nbs[1] = cv::Point(1,0);
  nbs[2] = cv::Point(0,-1);
  nbs[3] = cv::Point(0,1);
}

OcclusionClustering::~OcclusionClustering()
{
}

/************************** PRIVATE ************************/


/**
 * @brief OcclusionClustering::clusterNaNs
 * @param _cloud
 * @param _mask
 * @param _contour
 */
void OcclusionClustering::clusterNaNs(const cv::Point &_start, const pcl::PointCloud<pcl::PointXYZRGB> &_cloud, cv::Mat_<unsigned char> &_mask, std::vector<Eigen::Vector3f> &_contour, std::vector<cv::Point> &_points)
{
  cv::Point pt0, pt;
  int queue_idx = 0;
  int width = _cloud.width;
  int height = _cloud.height;

  _contour.clear();
  _mask(_start) = 1;
  _points.assign(1,_start);
  queue.assign(1,_start);

  // start clustering
  while (((int)queue.size()) > queue_idx)
  {
    // extract current index
    pt0 = queue.at(queue_idx);
    queue_idx++;


    for (unsigned i=0; i<nbs.size(); i++)
    {
      pt = pt0+nbs[i];

      if ( (pt.x < 0) || (pt.y < 0) || pt.x >= width || pt.y >= height )
        continue;

      if (_mask(pt)!=0) // used point
        continue;

      if(isnan(_cloud(pt.x,pt.y)))
      {
        _mask(pt)=1;
        queue.push_back(pt);
        _points.push_back(pt);
      }
      else _contour.push_back(_cloud(pt.x,pt.y).getVector3fMap());
    }
  }
}

/**
 * @brief OcclusionClustering::getDepthVariance
 * @param _contour
 * @return
 */
double OcclusionClustering::getDepthVariance(const std::vector<Eigen::Vector3f> &_contour)
{
  double var=0, mean = 0;
  for (unsigned i=0; i<_contour.size(); i++)
    mean += _contour[i][2];
  mean /= (double)_contour.size();
  for (unsigned i=0; i<_contour.size(); i++)
    var += sqr((_contour[i][2]-mean));
  var /= (double)_contour.size();
  return var;
}


/************************** PUBLIC *************************/


/**
 * Compute
 */
void OcclusionClustering::compute(const pcl::PointCloud<pcl::PointXYZRGB> &_cloud, cv::Mat_<unsigned char> &_mask)
{
  _mask = cv::Mat_<unsigned char>::zeros(_cloud.height, _cloud.width);
  double thr_var_depth = param.thr_std_dev*param.thr_std_dev;

  for (int v=0; v<(int)_cloud.height; v++)
  {
    for (int u=0; u<(int)_cloud.width; u++)
    {
      if (isnan(_cloud(u,v)) && _mask(v,u)==0)
      {
        clusterNaNs(cv::Point(u,v), _cloud, _mask, contour, points);
        if (contour.size()>=3 && getDepthVariance(contour)>thr_var_depth)
        {
          for (unsigned i=0; i<points.size(); i++)
            _mask(points[i]) = 255;
        }
      }
    }
  }
}


} //-- THE END --

