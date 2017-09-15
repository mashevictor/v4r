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



#include "v4r/attention_segmentation/AttentionExampleUtils.h"

namespace v4r 
{
bool checkIsNaN(const pcl::PointXYZRGB &p)
{
  if(std::isnan(p.x) ||
     std::isnan(p.y) ||
     std::isnan(p.z))
  {
    return(true);
  }
  return(false);
}

V4R_EXPORTS int preparePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients, 
		      pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointIndices::Ptr object_indices_in_the_hull, bool useStandartNormals)
{
  // create filtered point cloud
  pcl::PointIndices::Ptr indices(new pcl::PointIndices());
  if(!pclAddOns::FilterPointCloud<pcl::PointXYZRGB>(cloud,indices))
  {
    return(pclAddOns::FILTER);
  }
  
  // segment plane
  pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices());
  pcl::PointIndices::Ptr objects_indices(new pcl::PointIndices());
  if(!pclAddOns::SegmentPlane<pcl::PointXYZRGB>(cloud,indices,plane_indices,objects_indices,coefficients))
  {
    return(pclAddOns::SEGMENT);
  }
  
  pcl::PointIndices::Ptr object_indices_in_the_hull_all(new pcl::PointIndices());
  if(!pclAddOns::ConvexHullExtract<pcl::PointXYZRGB>(cloud,plane_indices,objects_indices,object_indices_in_the_hull_all,coefficients))
  {
    return(pclAddOns::CONVEXHULL);
  }
  
  //calculate point cloud normals
  if(useStandartNormals)
  {
    if(!pclAddOns::ComputePointNormals<pcl::PointXYZRGB>(cloud,object_indices_in_the_hull_all,normals))
    {
      return(pclAddOns::NORMALS);
    }
    object_indices_in_the_hull->indices = object_indices_in_the_hull_all->indices;
  }
  else
  {
    // calcuate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals_all(new pcl::PointCloud<pcl::Normal>);
    v4r::ZAdaptiveNormals<pcl::PointXYZRGB>::Parameter param;
    param.adaptive = true;
    v4r::ZAdaptiveNormals<pcl::PointXYZRGB> nor(param);
    nor.setInputCloud(cloud);
    nor.compute();
    nor.getNormals(normals_all);
    
    normals->points.reserve(object_indices_in_the_hull_all->indices.size());
    object_indices_in_the_hull->indices.reserve(object_indices_in_the_hull_all->indices.size());
    
    for(unsigned int i = 0; i < object_indices_in_the_hull_all->indices.size(); ++i)
    {
      int idx = object_indices_in_the_hull_all->indices.at(i);
      
      if( checkIsNaN(cloud->points.at(idx)) )
	continue;
      
      normals->points.push_back(normals_all->points.at(idx));
      object_indices_in_the_hull->indices.push_back(idx);
    }
    
  }
  
  return(0);
}

V4R_EXPORTS int preparePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointIndices::Ptr object_indices, bool useStandartNormals)
{
  // create filtered point cloud
  pcl::PointIndices::Ptr indices(new pcl::PointIndices());
  if(!pclAddOns::FilterPointCloud<pcl::PointXYZRGB>(cloud,indices))
  {
    return(pclAddOns::FILTER);
  }
  
  if(useStandartNormals)
  {
    if(!pclAddOns::ComputePointNormals<pcl::PointXYZRGB>(cloud,indices,normals))
    {
      return(pclAddOns::NORMALS);
    }
    object_indices->indices = indices->indices;
  }
  else
  {
    // calcuate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals_all(new pcl::PointCloud<pcl::Normal>);
    v4r::ZAdaptiveNormals<pcl::PointXYZRGB>::Parameter param;
    param.adaptive = true;
    v4r::ZAdaptiveNormals<pcl::PointXYZRGB> nor(param);
    nor.setInputCloud(cloud);
    nor.compute();
    nor.getNormals(normals_all);
    
    normals->points.reserve(indices->indices.size());
    object_indices->indices.reserve(indices->indices.size());
    
    for(unsigned int i = 0; i < indices->indices.size(); ++i)
    {
      int idx = indices->indices.at(i);
      
      if( checkIsNaN(cloud->points.at(idx)) )
	continue;
      
      normals->points.push_back(normals_all->points.at(idx));
      object_indices->indices.push_back(idx);
    }
    
  }
  
  return(0);
}

} // namespace v4r
