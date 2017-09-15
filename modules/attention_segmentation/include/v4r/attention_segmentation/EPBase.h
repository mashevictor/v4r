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


#ifndef EP_BASE_HH
#define EP_BASE_HH

#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
//#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <v4r/core/macros.h>

namespace v4r
{

/**
 * EPBase
 */
class V4R_EXPORTS EPBase
{
public:
  
  typedef boost::shared_ptr<EPBase> Ptr;

protected:
  
  bool computed;
  bool have_cloud;
  bool have_normals;
  bool have_indices;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;               ///< Input cloud
  pcl::PointCloud<pcl::Normal>::Ptr normals;                  ///< Normals (set from outside or from surfaces)
  std::vector<int> indices;                             ///< Indices to be used
  
  int width, height;

  inline int getIdx(int x, int y) const;
  inline int X(int idx);
  inline int Y(int idx);
  inline bool isNaN(const pcl::PointXYZRGB p);
  inline bool isNaN(const pcl::Normal n);
  inline bool isNaN(const Eigen::Vector3f p);
  
  std::string ClassName;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EPBase();
  ~EPBase();

  /** Set input cloud **/
  virtual void setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);
  /** Set normals **/
  void setNormals(const pcl::PointCloud<pcl::Normal>::Ptr &_normals);
  /** Set indices **/
  void setIndices(const pcl::PointIndices::Ptr &_indices);
  void setIndices(const std::vector<int> &_indices);

  /** General compute method **/
  virtual void compute();
  
  /** Return normals **/
  inline pcl::PointCloud<pcl::Normal>::Ptr getNormals();

};


/*********************** INLINE METHODES **************************/
inline bool EPBase::isNaN(const pcl::PointXYZRGB p)
{
  if(std::isnan(p.x) ||
     std::isnan(p.y) ||
     std::isnan(p.z))
  {
    return(true);
  }
  return(false);
}

inline bool EPBase::isNaN(const pcl::Normal n)
{
  if(std::isnan(n.normal[0]) ||
     std::isnan(n.normal[1]) ||
     std::isnan(n.normal[2]))
  {
    return(true);
  }
  return(false);
}

inline bool EPBase::isNaN(const Eigen::Vector3f p)
{
  if(std::isnan(p[0]) ||
     std::isnan(p[1]) ||
     std::isnan(p[2]))
  {
    return(true);
  }
  return(false);
}

inline int EPBase::getIdx(int x, int y) const
{
  return y*width+x; 
}

inline int EPBase::X(int idx)
{
  return idx%width;
}

inline int EPBase::Y(int idx)
{
  return idx/width;
}

inline pcl::PointCloud<pcl::Normal>::Ptr EPBase::getNormals()
{
  return normals;
}

}

#endif //BASE_HH

